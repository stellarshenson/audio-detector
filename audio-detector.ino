/*
  Audio detector that triggers on the audio signal at the RCA socket and turns on (and off)
  your audio system using Infrared Remote signal and/or a Relay

                     +-----+
        +------------| USB |------------+
        |            +-----+            |
   B5   | [ ]D13/SCK        MISO/D12[ ] |   B4
        | [ ]3.3V           MOSI/D11[ ]~|   B3
        | [ ]V.ref     ___    SS/D10[ ]~|   B2
   C0   | [ ]A0       / N \       D9[ ]~|   B1
   C1   | [ ]A1      /  A  \      D8[ ] |   B0
   C2   | [ ]A2      \  N  /      D7[ ] |   D7
   C3   | [ ]A3       \_0_/       D6[ ]~|   D6
   C4   | [ ]A4/SDA               D5[ ]~|   D5
   C5   | [ ]A5/SCL               D4[ ] |   D4
        | [ ]A6              INT1/D3[ ]~|   D3
        | [ ]A7              INT0/D2[ ] |   D2
        | [ ]5V                  GND[ ] |
   C6   | [ ]RST                 RST[ ] |   C6
        | [ ]GND   5V MOSI GND   TX1[ ] |   D0
        | [ ]Vin   [ ] [ ] [ ]   RX1[ ] |   D1
        |          [ ] [ ] [ ]          |
        |          MISO SCK RST         |
        | NANO-V3                       |
        +-------------------------------+

  26 Mar 2020: code has been tested and all works with NAD326bee amplifier and Yamaha WXAD-10 streamer
  20 Aug 2022: updated the code with the moving average to elliminate voltage spikes

  Copyright by Stellars Henson 2020-2022
*/

#include <EEPROM.h>
#include <IRremote.h>         //version 2.6.1
#include <Fsm.h>              //arduino-fsm fork by Stellars Henson https://github.com/stellarshenson/arduino-fsm
#include <EnableInterrupt.h>  //version 1.1.0
#include <jled.h>             //version 4.11
#include <Smoothed.h>         //version 1.2.0

#define OUTPUT_IRCODE_SEND_PIN 3 //pin to send the IR code with via optocoupler
#define INPUT_IRCODE_RECV_PIN 2 //pin connected to IR phototransistor
#define INPUT_BUTTON_PIN 8 //learn signal level with single press, learn IR codes on holdpress
#define OUTPUT_STATUS_LED_PIN 9 //single LED
#define OUTPUT_AUDIO_ENABLED_PIN 4 //set to high when audio is enabled
#define INPUT_AUDIOTRIGGER_PIN 7 //connected to the optocoupler that detects the 12V trigger from the amp
#define INPUT_AUDIOSENSE_ADC_PIN A1  //sense audio with ADC. Vref should be 3v3
#define INPUT_CONFIG_AUTOSTANDBY_PIN 6 //configuration pin pullup. If GND than autostandby will be used. 

#define DEBUG_LEVEL 2 //0 - debug off, 1 - essential messages, 2 - full diagnostics
#define AUDIO_START_TIMEOUT 3000 //timeout for the audio start detection
#define AUDIO_STANDBY_TIMEOUT 10* 60000 //timeout for the audio shutdown if no signal (10min)
#define AUDIO_ENABLED_SENSE_INTERVAL 1* 60000 //timeout for checking audio signal in AUDIO_ENABLED (1min)
#define AUDIOSENSE_INIT_THRESHOLD 300 //initial threshold if no config available
#define AUDIOSENSE_AVG_SAMPLES 50 //number of samples for averaging
#define AUDIOSENSE_ADC_INTERVAL 50 //time between samples
#define STARTUP_STABILISE_DURATION 3000 //let the system tabilise for a while (3s)

//triggers for the FSM
#define TRIGGER_IRCODE_RECORD 1
#define TRIGGER_IRCODE_RECORDED 2
#define TRIGGER_AUDIO_DETECTED 3
#define TRIGGER_AUDIO_ENABLED 4
#define TRIGGER_AUDIO_DISABLED 5
#define TRIGGER_AUDIO_LEARN 6
#define TRIGGER_AUDIO_LEARNED 7

//IR receiver setup
#define EXCLUDE_EXOTIC_PROTOCOLS // saves around 900 bytes program space with IRRemote v3.2+
IRrecv irrecv(INPUT_IRCODE_RECV_PIN);
IRsend irsend;
decode_results results;

//nonblocking LED setup
auto led_sense_noconfig = JLed(OUTPUT_STATUS_LED_PIN).Blink(250, 1000).Forever(); //quick blinking if no config
auto led_sense_configok = JLed(OUTPUT_STATUS_LED_PIN).Blink(1000, 2000).Forever(); //long blinking if everything is ok
auto led_audio_enabled =  JLed(OUTPUT_STATUS_LED_PIN).On();
auto led_audio_disabled = JLed(OUTPUT_STATUS_LED_PIN).Off();
auto led_audio_learning = JLed(OUTPUT_STATUS_LED_PIN).Blink(750, 250).Forever();
auto led_ir_recording_1 = JLed(OUTPUT_STATUS_LED_PIN).Blink(100, 100).Forever();
auto led_ir_recording_2 = JLed(OUTPUT_STATUS_LED_PIN).Blink(500, 500).Forever();
JLed led_active = led_sense_noconfig; //this will be set to the active led scheme

// Storage for the recorded code
void sendCode(uint8_t repeat, uint8_t codeType, uint8_t codeLen, unsigned long codeValue);
void storeCode(decode_results *results, int8_t &codeType, uint8_t &codeLen, unsigned long &codeValue);


uint8_t startAudioCodeType = 255; // The type of code
uint8_t startAudioCodeLen; // The length of the code
unsigned long startAudioCodeValue; // The code value if not raw

uint8_t stopAudioCodeType = 255; // The type of code
uint8_t stopAudioCodeLen; // The length of the code
uint32_t stopAudioCodeValue; // The code value if not raw

uint8_t toggle = 0; // The RC5/6 toggle state
uint16_t rawCodes[RAW_BUFFER_LENGTH]; // The durations if raw
uint8_t irCodesAvailable = 0; //if ircodes were recorded

uint8_t audioTriggerAvailable = 0; //if 12V trigger is available. Audio enable timeout sets this to 0, detection of the 12V trigger sets this to 1. We can also detect this with an audio jack (it has detect input feature)
uint8_t autoStandbyEnabled = 0; //enabled by the configuration pin INPUT_CONFIG_AUTOSTANDBY_PIN

//interrupt driver
#define BUTTON_HOLDPRESS 2
#define BUTTON_SINGLEPRESS 1
#define HOLDPRESS_TIMEOUT 600
volatile uint8_t button_detected = 0;
void on_button_irq(); //irq driver
void on_button_update(); //updated in the loop button sensing for single and long press

uint32_t audiosense_millis = 0; //used to measure time before consecutive audio signal checks in AUDIO_ENABLED state

//ADC smoothing setup
Smoothed <float> audioSenseMovingAvg;
Smoothed <float> audioSenseMovingAvg_learn;
uint16_t audioSenseLearnCounter = 0;
uint16_t audioSenseThreshold = AUDIOSENSE_INIT_THRESHOLD;


//FSM - set up of the state machine
void on_ircode_record_enter();
void on_ircode_record_loop();
void on_ircode_record_exit();
void on_audio_learn_enter();
void on_audio_learn_loop();
void on_audio_learn_exit();
void on_audio_sense_enter();
void on_audio_sense_loop();
void on_audio_sense_exit();
void on_audio_start_enter();
void on_audio_start_loop();
void on_audio_start_exit();
void on_audio_enabled_enter();
void on_audio_enabled_loop();
void on_audio_enabled_exit();
void on_audio_start_timed_trans_audio_enabled();
State state_audio_sense(on_audio_sense_enter, on_audio_sense_loop, on_audio_sense_exit); //idle state, waiting for audio signal
State state_ircode_record(on_ircode_record_enter, on_ircode_record_loop, on_ircode_record_exit); //state where IR codes are recorded for start and stop audio
State state_audio_learn(on_audio_learn_enter, on_audio_learn_loop, on_audio_learn_exit); //state where threshold for audio detection is set
State state_audio_start(on_audio_start_enter, on_audio_start_loop, on_audio_start_exit); //state where audio was detected and amplifier is switched on. this state waits for 12v standby-on or timeout (if not 12v trigger available)
State state_audio_enabled(on_audio_enabled_enter, on_audio_enabled_loop, on_audio_enabled_exit); //idle state where audio is enabled. keeps checking if the audio is still on
Fsm   fsm(&state_audio_sense); //set up the bootstrap state to start with

//debug and commandline
uint8_t debug_plot_enabled = 0; //plot disabled by default
void cmd_poll(HardwareSerial &serial, void *handler(int, String*));
void cmd_handler(int argc, String* argv);

//reboot function
void(* reboot) (void) = 0;

void setup() {
  //start serial and command listener
  Serial.begin(9600);

  //setup pins
  pinMode(INPUT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(INPUT_AUDIOTRIGGER_PIN, INPUT_PULLUP);
  pinMode(OUTPUT_STATUS_LED_PIN, OUTPUT);
  pinMode(OUTPUT_AUDIO_ENABLED_PIN, OUTPUT);
  pinMode(INPUT_AUDIOSENSE_ADC_PIN, INPUT);
  pinMode(INPUT_CONFIG_AUTOSTANDBY_PIN, INPUT_PULLUP);
  digitalWrite(OUTPUT_AUDIO_ENABLED_PIN, LOW);

  //read ircode from EEPROM
  startAudioCodeValue = ( (unsigned long)EEPROM.read(0)) | ( (unsigned long)EEPROM.read(1) << 8) | ((unsigned long) EEPROM.read(2) << 16) | ( (unsigned long)EEPROM.read(3) << 24);
  startAudioCodeLen = EEPROM.read(4);
  startAudioCodeType = EEPROM.read(5);

  stopAudioCodeValue = ( (unsigned long)EEPROM.read(6)) | ( (unsigned long)EEPROM.read(7) << 8) | ((unsigned long) EEPROM.read(8) << 16) | ( (unsigned long)EEPROM.read(9) << 24);
  stopAudioCodeLen = EEPROM.read(10);
  stopAudioCodeType = EEPROM.read(11);

  //recover audio sense threshold for ADC
  audioSenseThreshold = ( (uint16_t)EEPROM.read(12)) | ( (uint16_t)EEPROM.read(13) << 8);

  //initiate status led to indicate if config ok or not available
  if (startAudioCodeType != 255 && stopAudioCodeType != 255) irCodesAvailable = 1;
  else irCodesAvailable = 0;

  //read autostandby status. pin has a pullup.
  autoStandbyEnabled = digitalRead(INPUT_CONFIG_AUTOSTANDBY_PIN) == LOW;

  if (DEBUG_LEVEL) Serial.print(F("[INIT] Restoring IR codes, AUDIO START: "));
  if (DEBUG_LEVEL) Serial.print(startAudioCodeValue, HEX);
  if (DEBUG_LEVEL) Serial.print(F(" , AUDIO STOP: "));
  if (DEBUG_LEVEL) Serial.print(stopAudioCodeValue, HEX);
  if (DEBUG_LEVEL) Serial.print(F(" , SENSE THRESHOLD: "));
  if (DEBUG_LEVEL) Serial.println(audioSenseThreshold, DEC);
  if (DEBUG_LEVEL && irCodesAvailable != 1) Serial.println(F("[INIT] IR codes not available"));
  if (DEBUG_LEVEL) Serial.print(F("[INIT] Automatic standby is: "));
  if (DEBUG_LEVEL) Serial.println( digitalRead(autoStandbyEnabled) ? "OFF" : "ON" );

  //initialise state machine
  fsm.add_transition(&state_audio_sense, &state_ircode_record, TRIGGER_IRCODE_RECORD, NULL);
  fsm.add_transition(&state_ircode_record, &state_audio_sense, TRIGGER_IRCODE_RECORDED, NULL);
  fsm.add_transition(&state_audio_sense, &state_audio_learn, TRIGGER_AUDIO_LEARN, NULL);
  fsm.add_transition(&state_audio_learn, &state_audio_sense, TRIGGER_AUDIO_LEARNED, NULL);
  fsm.add_transition(&state_audio_sense, &state_audio_start, TRIGGER_AUDIO_DETECTED, NULL);
  fsm.add_transition(&state_audio_start, &state_audio_enabled, TRIGGER_AUDIO_ENABLED, NULL);
  fsm.add_transition(&state_audio_enabled, &state_audio_sense, TRIGGER_AUDIO_DISABLED, NULL);
  fsm.add_transition(&state_audio_enabled, &state_audio_learn, TRIGGER_AUDIO_LEARN, NULL);
  fsm.add_transition(&state_audio_enabled, &state_ircode_record, TRIGGER_IRCODE_RECORD, NULL);
  fsm.add_timed_transition(&state_audio_start, &state_audio_enabled, AUDIO_START_TIMEOUT, on_audio_start_timed_trans_audio_enabled);
  fsm.add_timed_transition(&state_audio_enabled, &state_audio_sense, AUDIO_STANDBY_TIMEOUT, on_audio_enabled_timed_trans_audio_sense);

  //enable audiosense averaging
  audioSenseMovingAvg.begin(SMOOTHED_AVERAGE, AUDIOSENSE_AVG_SAMPLES);
  audioSenseMovingAvg_learn.begin(SMOOTHED_AVERAGE, AUDIOSENSE_AVG_SAMPLES);

  //print command prompt
  Serial.println(F("[INIT] type 'help' for commands list"));
}

void loop() {
  fsm.run_machine(); //run state loops
  led_active.Update(); //run led driver
  on_button_update(); //run button sensing
  cmd_poll(Serial, cmd_handler); //run command interpreter
}

// *******************************
// STATE MACHINE
// *******************************


// STATE_AUDIOSENSE

/**
  only announces the transition
*/
void on_audio_sense_enter() {
  if (DEBUG_LEVEL) Serial.println(F("[AUDIO SENSE] Start. Listening for audio and ircode record button"));

  //interrupt initialisation. check initial audio signal status
  button_detected = 0;
  enableInterrupt(INPUT_BUTTON_PIN, on_button_irq, CHANGE);

  //light LED for status and configuration
  if (irCodesAvailable == 1) led_active = led_sense_configok;
  else led_active = led_sense_noconfig;
}

/**
  waits for either ircode record button trigger or for audio signal
  ircode button -> goes to ircode recording
  audiosense signal -> goes to start audio
  audio and ircode button detection is driven by interrupts
*/
void on_audio_sense_loop() {
  boolean _audio_sensed = senseAudio() > audioSenseThreshold;
  if ( millis() < STARTUP_STABILISE_DURATION ) _audio_sensed = false; //ignore signal if still stabilising the system

  // hold button to enable recording ircode
  if (button_detected == BUTTON_HOLDPRESS) {
    if (DEBUG_LEVEL) Serial.println(F("[AUDIO SENSE] IRCode recording detected"));
    fsm.trigger(TRIGGER_IRCODE_RECORD);
  }

  // button to enable learning audio threshold
  if (button_detected == BUTTON_SINGLEPRESS) {
    if (DEBUG_LEVEL) Serial.println(F("[AUDIO SENSE] Audiosense learning detected"));
    fsm.trigger(TRIGGER_AUDIO_LEARN);
  }

  // this is detected with ADC pin INPUT_AUDIOSENSE_ADC_PIN
  if (_audio_sensed) {
    if (DEBUG_LEVEL) Serial.println(F("[AUDIO SENSE] Audio signal detected"));
    fsm.trigger(TRIGGER_AUDIO_DETECTED);
  }
}

/**
  signals exit from the state
*/
void on_audio_sense_exit() {
  if (DEBUG_LEVEL == 2) Serial.println(F("[AUDIO SENSE] Exit. Disabling audio signal detection"));
  disableInterrupt(INPUT_BUTTON_PIN);

  //clear audiosense averaging
  audioSenseMovingAvg.clear();
}


// IRCODE RECORD STATE

/**
  enables IR receiver and turns recording LED on
  it also resets codes recorded previously
*/
void on_ircode_record_enter() {
  // enable receiver
  irrecv.enableIRIn();

  //enable recording led
  if (DEBUG_LEVEL) Serial.println(F("[IRCODE RECORD] Start.  Enabling IR receiver"));
  led_active = led_ir_recording_1;

  //reset all previosu codes
  startAudioCodeValue = 0;
  startAudioCodeType = 255;
  stopAudioCodeValue = 0;
  stopAudioCodeType = 255;
}

/**
  waits until IR remote sends the IR code
  - records audio start code and saves to EEPROM and waits for recoding of the audio stop
  - records audio stop code and saves to EEPROM
  after this goes back to audio sense state

  the STORED_LED will be flashing with 1HZ when only one code was recorded and will
  be turned on permanently when all codes were recorded
*/
void on_ircode_record_loop() {
  //read and decode the ircode, ircode was received with the interrupts
  if (irrecv.decode(&results)) {
    //record start audio code
    if ( startAudioCodeType == 255 ) {
      if (DEBUG_LEVEL) Serial.println(F("[IRCODE RECORD][1/2] - AUDIO START ircode detected"));
      storeCode(&results, startAudioCodeType, startAudioCodeLen, startAudioCodeValue);

      //write the to eeprom and report on serial
      EEPROM.update(0, startAudioCodeValue);
      EEPROM.update(1, startAudioCodeValue >> 8);
      EEPROM.update(2, startAudioCodeValue >> 16);
      EEPROM.update(3, startAudioCodeValue >> 24);
      EEPROM.update(4, startAudioCodeLen);
      EEPROM.update(5, startAudioCodeType);
      if (DEBUG_LEVEL == 2) Serial.print(F("[IRCODE RECORD][1/2] Saved value: "));
      if (DEBUG_LEVEL) Serial.println(startAudioCodeValue, HEX);
      if (DEBUG_LEVEL == 2) Serial.println(F("[IRCODE RECORD][1/2] Waiting for AUDIO STOP ircode"));

      delay(500);
      irrecv.resume(); //resume recording for the stop code

      //enable recording part #2 led
      led_active = led_ir_recording_2;
    }
    //record stop audio code
    else if ( startAudioCodeType != 255 && stopAudioCodeType == 255 ) {
      if (DEBUG_LEVEL) Serial.println(F("[IRCODE RECORD][2/2] AUDIO STOP ircode detected"));
      storeCode(&results, stopAudioCodeType, stopAudioCodeLen, stopAudioCodeValue);

      //write the to eeprom and report on serial
      EEPROM.update(6, stopAudioCodeValue);
      EEPROM.update(7, stopAudioCodeValue >> 8);
      EEPROM.update(8, stopAudioCodeValue >> 16);
      EEPROM.update(9, stopAudioCodeValue >> 24);
      EEPROM.update(10, stopAudioCodeLen);
      EEPROM.update(11, stopAudioCodeType);
      if (DEBUG_LEVEL == 2) Serial.print(F("[IRCODE RECORD][2/2] Saved value: "));
      if (DEBUG_LEVEL) Serial.println(stopAudioCodeValue, HEX);

      //trigger transition
      fsm.trigger(TRIGGER_IRCODE_RECORDED);  //trigger transition only after audio stop code was recorded
    }
  }
}

/**
  announces exit from ircode recording state and turns off recording LED
*/
void on_ircode_record_exit() {
  if (DEBUG_LEVEL == 2) Serial.println(F("[IRCODE RECORD] Exit. Disabling receiver"));

  //mark that ir codes are available now
  irCodesAvailable = 1;
}

// AUDIO LEARN STATE

/**
  reset learning and indicate with LED that learningg started
*/
void on_audio_learn_enter() {
  if (DEBUG_LEVEL == 2) Serial.println(F("[AUDIOSENSE LEARN] Entering state. Resetting threshold and counter"));
  led_active = led_audio_learning;
  audioSenseLearnCounter = 0;
  audioSenseThreshold = 0;
}

/**
  start smooting the audio over the next SAMPLES and record result to EEPROM
*/
void on_audio_learn_loop() {
  static uint32_t _lastSenseMillis = millis();

  if (millis() > _lastSenseMillis + AUDIOSENSE_ADC_INTERVAL) {
    if (audioSenseLearnCounter++ < AUDIOSENSE_AVG_SAMPLES) {
      float _value = analogRead(INPUT_AUDIOSENSE_ADC_PIN);

      //reset last sense millis
      _lastSenseMillis = millis();
      audioSenseMovingAvg_learn.add(_value);

      //plot only if enabled
      if (debug_plot_enabled) {
        Serial.print(F("[Learn]  minimum: 0, maximum: 1024, sensed_value: "));
        Serial.print(_value);
        Serial.print(F(", average_value:  "));
        Serial.println(audioSenseMovingAvg_learn.get());
      }
    } else {
      //set threshold to the measured average but it must be at least 10
      audioSenseThreshold = audioSenseMovingAvg_learn.get() > 10 ? audioSenseMovingAvg_learn.get() : 10;

      //save to EEPROM
      EEPROM.update(12, audioSenseThreshold);
      EEPROM.update(13, audioSenseThreshold >> 8);

      //trigger transition
      fsm.trigger(TRIGGER_AUDIO_LEARNED);
    }
  }
}

/**
  announce recorded value and exit
*/
void on_audio_learn_exit() {
  if (DEBUG_LEVEL == 2) {
    Serial.print(F("[AUDIOSENSE LEARN] Exit. New threshold set to: "));
    Serial.println(audioSenseThreshold, DEC);
  }

  //disable audiosense averaging
  audioSenseMovingAvg_learn.clear();
}

// AUDIO START STATE

/**
  sends recorded IR code on enter
*/
void on_audio_start_enter() {
  Serial.println(F("[AUDIO START] Start. Sending AUDIO START IR code and monitoring"));

  //initialise trigger sense
  audioTriggerAvailable = 0;

  //send audio start IR signal
  sendCode(0, startAudioCodeType, startAudioCodeLen, startAudioCodeValue); //send recorded or saved IR code without repeat
}

/**
  waits for the external 12V trigger (typically output from an amplifier
  MK2 - if the external 12V trigger was not detected in AUDIOTRIGGER_TIMEOUT, system enables its own 12V trigger relay
  once 12V trigger was enabled or detected, move to AUDIO_ENABLED state
*/
void on_audio_start_loop() {
  uint8_t audioTriggerState = digitalRead(INPUT_AUDIOTRIGGER_PIN);  //read the status of the external 12V trigger via optocoupler circuit connected to GND

  //detecting status of the 12v trigger line
  //it stays this way, we are not trying to detect the signal, just the state
  if (audioTriggerState == LOW) {
    if (DEBUG_LEVEL) Serial.println(F("[AUDIO START] Audio trigger detected"));
    audioTriggerAvailable = 1; //12V audio trigger is available, no need for timeout
    fsm.trigger(TRIGGER_AUDIO_ENABLED); //initiate state transition
  }
}

/**
  announce exit from the audio start state
*/
void on_audio_start_exit() {
  if (DEBUG_LEVEL == 2) Serial.println(F("[AUDIO START] Exit. Audio is on"));
}

/**
  timed transition to audio enabled state after AUDIO_START_TIMEOUT (in miliseconds)
*/
void on_audio_start_timed_trans_audio_enabled() {
  if (DEBUG_LEVEL) Serial.println(F("[AUDIO START] Timeout. Trigger not available"));
  audioTriggerAvailable = 0; //12V audio trigger is not available
}

// AUDIO ENABLED STATE

/**
  announce enter audio enabled state
*/
void on_audio_enabled_enter() {
  if (DEBUG_LEVEL) Serial.println(F("[AUDIO ENABLED] Start. Monitoring for standby"));

  //if audiotrigger is not available, enable interrupts to listen to audio signal
  if (audioTriggerAvailable == 0) {
    audiosense_millis = millis();

    if (DEBUG_LEVEL == 2) {
      Serial.print(F("[AUDIO ENABLED] Trigger not available. Enabling audio signal timeout for "));
      Serial.print(AUDIO_STANDBY_TIMEOUT / 60000, DEC);
      Serial.println(F("min"));
    }
  }

  //led for audio on
  led_active = led_audio_enabled;

  //enable output pin to indicate that audio is on
  digitalWrite(OUTPUT_AUDIO_ENABLED_PIN, HIGH);

  //interrupt initialisation for ir code recording and for threshold learning
  button_detected = 0;
  enableInterrupt(INPUT_BUTTON_PIN, on_button_irq, CHANGE);
}

/**
  listens for the 12V trigger to go LOW (INPUT_AUDIOTRIGGER_PIN will go up becasue of optocoupler connected to GND and the pullup sense pin)
  when 12V LOW state is detected, it indicates that the amp is in standby. System goes back to AUDIO SENSE state

  if 12V trigger is not available, system refreshes  the timer with the audio signal detection
*/
void on_audio_enabled_loop() {
  boolean _audio_sensed = senseAudio() > audioSenseThreshold;;

  //listen for the trigger only if available
  if (audioTriggerAvailable == 1) {
    //read 12V audio trigger from the optocoupler connected to GND (inverts the signal)
    uint8_t audioTriggerState = digitalRead(INPUT_AUDIOTRIGGER_PIN);  //read the status of the external 12V trigger via optocoupler circuit connected to GND

    //if audiotrigger is available we await the 12V trigger to go LOW
    //INPUT_AUDIOTRIGGER_PIN goes HIGH inverted by the optocoupler
    if (audioTriggerState == HIGH) {
      if (DEBUG_LEVEL) Serial.println(F("[AUDIO ENABLED] Standby trigger detected"));
      fsm.trigger(TRIGGER_AUDIO_DISABLED); //initiate state transition
    }
    //when 12V trigger is not present, check for audio signal every AUDIOSENSE_ENABLED_SENSE_INTERVAL
  } else if ( audioTriggerAvailable == 0 &&  _audio_sensed && ((audiosense_millis + AUDIO_ENABLED_SENSE_INTERVAL <= millis())) || audiosense_millis > millis() ) {
    //if signal was detected - reset timer and the signal
    if (DEBUG_LEVEL == 2) Serial.println(F("[AUDIO ENABLED] Audio detected. Timeout reset"));

    fsm.reset_timed_transition(&state_audio_sense);
    audiosense_millis = millis(); //update sense timer
  }

  // button to enable learning audio threshold
  // driven by interrupts now on INPUT_BUTTON_PIN
  if (button_detected == BUTTON_SINGLEPRESS) {
    if (DEBUG_LEVEL) Serial.println(F("[AUDIO ENABLED] Audiosense learning detected"));
    fsm.trigger(TRIGGER_AUDIO_LEARN);
  }

  // button to enable ir code recording
  if (button_detected == BUTTON_HOLDPRESS) {
    if (DEBUG_LEVEL) Serial.println(F("[AUDIO ENABLED] IRcode recording detected"));
    fsm.trigger(TRIGGER_IRCODE_RECORD);
  }
}

/**
  announce leaving the audio enabled state
*/
void on_audio_enabled_exit() {
  if (DEBUG_LEVEL == 2) Serial.println(F("[AUDIO ENABLED] Exit. Sending AUDIO STOP ircode"));

  //send AUDIO STOP IR signal if autostandby enabled
  if (autoStandbyEnabled) sendCode(0, stopAudioCodeType, stopAudioCodeLen, stopAudioCodeValue); //send recorded or saved IR code without repeat

  //disable audio sense interrupt
  if (DEBUG_LEVEL == 2 && audioTriggerAvailable == 0) Serial.println(F("[AUDIO ENABLED] Disabling shutdown timer"));
  disableInterrupt(INPUT_BUTTON_PIN);

  //indicate LED audio disabled
  led_active = led_audio_disabled;

  //disable output pin to indicate that audio is off
  digitalWrite(OUTPUT_AUDIO_ENABLED_PIN, LOW);

  //clear audiosense averaging
  audioSenseMovingAvg.clear();
}


/**
  timed transition to audio sense
*/
void on_audio_enabled_timed_trans_audio_sense() {
  if (DEBUG_LEVEL) Serial.println(F("[AUDIO ENABLED] Timeout. No audio signal"));

  //indicate LED audio disabled
  led_active = led_audio_disabled;

  //disable output pin to indicate that audio is off
  digitalWrite(OUTPUT_AUDIO_ENABLED_PIN, LOW);
}

//===================== UTILS ============================

/**
  performs sensing of the audio signal with ADC
*/
int senseAudio() {
  static uint32_t _lastSenseMillis = millis();
  float _senseValue = analogRead(INPUT_AUDIOSENSE_ADC_PIN);
  float _smoothedValue = audioSenseMovingAvg.get();

  if (millis() > _lastSenseMillis + AUDIOSENSE_ADC_INTERVAL) {
    audioSenseMovingAvg.add(_senseValue);
    _lastSenseMillis = millis();

    // Output the smoothed values to the serial stream.
    // Open the Arduino IDE Serial plotter to see the effects of the smoothing methods.
    if (debug_plot_enabled) {
      Serial.print(F("[ADC] minimum: 0, maximum: 1024, current_value: "));
      Serial.print(_senseValue);
      Serial.print(F(", smoothed_value: "));
      Serial.print(_smoothedValue);
      Serial.print(F(", threshold_value: "));
      Serial.println(audioSenseThreshold);
    }
  }

  return _smoothedValue;
}

/**
  Stores the code for later playback
  Most of this code is just logging

  @param results - pointer to the structure with the results decoded
  @param codeType - type of the IR code (NEC, SONY etc..)
  @codeLen - length of the code in bytes
  @codeValue - 4 bytes long code value
*/
void storeCode(decode_results *results, uint8_t &codeType, uint8_t &codeLen, unsigned long &codeValue) {
  codeType = results->decode_type;
  uint8_t count = results->rawlen;
  if (codeType == UNKNOWN) {
    if (DEBUG_LEVEL) Serial.println(F("Received unknown code, saving as raw"));
    codeLen = results->rawlen - 1;
    // To store raw codes:
    // Drop first value (gap)
    // Convert from ticks to microseconds
    // Tweak marks shorter, and spaces longer to cancel out IR receiver distortion
    for (int i = 1; i <= codeLen; i++) {
      if (i % 2) {
        // Mark
        rawCodes[i - 1] = results->rawbuf[i] * MICROS_PER_TICK - MARK_EXCESS_MICROS;
        if (DEBUG_LEVEL) Serial.print(F(" m"));
      }
      else {
        // Space
        rawCodes[i - 1] = results->rawbuf[i] * MICROS_PER_TICK - MARK_EXCESS_MICROS;
        if (DEBUG_LEVEL) Serial.print(F(" s"));
      }
      if (DEBUG_LEVEL) Serial.print(rawCodes[i - 1], DEC);
    }
    if (DEBUG_LEVEL) Serial.println();
  }
  else {
    if (codeType == NEC) {
      if (DEBUG_LEVEL) Serial.print(F("Received NEC: "));
      if (results->value == REPEAT) {
        // Don't record a NEC repeat value as that's useless.
        if (DEBUG_LEVEL == 2) Serial.println(F("repeat; ignoring."));
        return;
      }
    }
    else if (codeType == SONY) {
      if (DEBUG_LEVEL == 2) Serial.print(F("Received SONY: "));
    }
    else {
      if (DEBUG_LEVEL) Serial.print(F("Unexpected"));
      if (DEBUG_LEVEL) Serial.print(codeType, DEC);
      if (DEBUG_LEVEL) Serial.println();
    }
    if (DEBUG_LEVEL == 2) Serial.println(results->value, HEX);
    codeValue = results->value;
    codeLen = results->bits;
  }
}

/**
  sends the IR code recorded in the global variables: codeType, codeValue, codeLen
  the IR code is sent using PWM pin and IR LED

*/
void sendCode(uint8_t repeat, uint8_t codeType, uint8_t codeLen, unsigned long codeValue) {
  if (codeType == NEC) {
    if (repeat) {
      irsend.sendNEC(REPEAT, codeLen);
      if (DEBUG_LEVEL) Serial.println(F("Sent NEC repeat"));
    }
    else {
      irsend.sendNEC(codeValue, codeLen);
      if (DEBUG_LEVEL) Serial.print(F("Sent NEC "));
      if (DEBUG_LEVEL) Serial.println(codeValue, HEX);
    }
  }
  else if (codeType == SONY) {
    irsend.sendSony(codeValue, codeLen);
    if (DEBUG_LEVEL) Serial.print(F("Sent Sony "));
    if (DEBUG_LEVEL) Serial.println(codeValue, HEX);
  }
  else if (codeType == UNKNOWN /* i.e. raw */) {
    // Assume 38 KHz
    irsend.sendRaw(rawCodes, codeLen, 38);
    if (DEBUG_LEVEL) Serial.println(F("Sent raw"));
  }
}

/**
   interrupt handler for audio learn detection
*/
void on_button_irq() {
  on_button_update();
}

/**
   drives update of the button status for short and long press
*/
void on_button_update() {
  static uint32_t _lastSenseMillis = millis(); //set timer
  static float _lastSenseValue = HIGH;  //start with button released

  //button pullup and on press shorted to GND
  float _senseValue = digitalRead(INPUT_BUTTON_PIN);


  //if single press detected
  if (_lastSenseValue == HIGH && _senseValue == LOW) {
    _lastSenseValue = LOW;
    _lastSenseMillis = millis();
  }
  //if release detected and no holdpress
  else if (_lastSenseValue == LOW && _senseValue == HIGH) {
    if (_lastSenseMillis + HOLDPRESS_TIMEOUT < millis() && button_detected == BUTTON_HOLDPRESS) {
      if (DEBUG_LEVEL) Serial.println(F("[IRQ] Holdpress ended"));
      button_detected = 0;
      _lastSenseValue = _senseValue;
    }
    else {
      button_detected = BUTTON_SINGLEPRESS;
      _lastSenseValue = _senseValue;
      if (DEBUG_LEVEL) Serial.println(F("[IRQ] Singlepress detected"));
    }
  }
  //if no release but holdpress
  else if (_lastSenseValue == LOW && _senseValue == LOW) {
    if (_lastSenseMillis + HOLDPRESS_TIMEOUT < millis() && button_detected != BUTTON_HOLDPRESS) {
      button_detected = BUTTON_HOLDPRESS;
      if (DEBUG_LEVEL) Serial.println(F("[IRQ] Holdpress detected"));
    }
  }
}


/**
   drives the commandline
*/
void cmd_poll(HardwareSerial &serial, void *handler(int, String*)) {
  static const uint8_t MAX_CMD_TOKENS = 10; //max tokens to process
  static const String _delimiter = " "; //delimit commands by space
  static const String _argv[MAX_CMD_TOKENS]; //fixed array for tokens
  String _cmd; //will contain the full command line
  String _token; //current token
  int _ptr = 0;
  int _argc = 0;

  //only if command available
  if (serial.available()) {
    _cmd = serial.readString();
    _cmd.trim();
    serial.println(_cmd);

    //tokenise the string
    while (_ptr < _cmd.length()) {
      //find next token
      int _delIndex = _cmd.indexOf(_delimiter, _ptr);
      if (_delIndex == -1) {
        _token = _cmd.substring(_ptr);
        _token.trim();
        _ptr = _cmd.length();
      } else {
        _token = _cmd.substring(_ptr, _delIndex);
        _token.trim();
        _ptr = _delIndex + _delimiter.length();
      }

      //save the token
      _argv[_argc++] = _token;

      //if argc is MAX_CMD_TOKENS, stop processing
      if (_argc == MAX_CMD_TOKENS) break;
    }

    //call handler function passed as argument
    handler(_argc, _argv);
  }
}

/**
   command handler
   @param argc number of tokens
   @param argv Strings array with tokens
*/
void cmd_handler(int argc, String * argv) {
  //set commands
  if ( argv[0] == "set" ) {

    //threshold
    if ( argc > 1 && argv[1] == "threshold") {
      if ( argc > 2 && argv[2].toInt() != 0 ) {
        Serial.print( F("[CONFIG] setting Audiosense threshold to ") );
        Serial.println(argv[2].toInt());
        audioSenseThreshold = argv[2].toInt();

        //save to EEPROM
        EEPROM.update(12, audioSenseThreshold);
        EEPROM.update(13, audioSenseThreshold >> 8);
      }
    }

    //audio state
    if ( argc > 1 && argv[1] == "audio" ) {
      if (argc > 2 && argv[2] == "0") {
        Serial.println(F("[CONFIG] turning audio off"));
        fsm.trigger(TRIGGER_AUDIO_DISABLED);
      } else if (argc > 2 && argv[2] == "1") {
        Serial.println(F("[CONFIG] turning audio on"));
        fsm.trigger(TRIGGER_AUDIO_DETECTED);
      }
    }

    //debug plot
    if ( argc > 1 && argv[1] == "debug_plot" ) {
      if ( argc > 2 && argv[2] == "0"  ) {
        Serial.println(F("[CONFIG] setting debug_plot off"));
        debug_plot_enabled = 0;
      } else if ( argc > 2 && argv[2] == "1" ) {
        Serial.println(F("[CONFIG] setting debug_plot on"));
        debug_plot_enabled = 1;
      }
    }
  }

  //status
  if ( argv[0] == "status") {
    Serial.println(F("[STATUS] Printing device status:"));
    Serial.print( F("Audiosense threshold: ") );
    Serial.println(audioSenseThreshold);
    Serial.print(F("Audio level is "));
    Serial.println(senseAudio());
    Serial.print(F("Audio is "));
    Serial.println(digitalRead(OUTPUT_AUDIO_ENABLED_PIN) == 0 ? "off" : "on");
    Serial.print(F("Autostandby config: "));
    Serial.println(autoStandbyEnabled == 0 ? "off" : "on");
    Serial.print(F("External 12V Trigger: "));
    Serial.println(digitalRead(INPUT_AUDIOTRIGGER_PIN) == 0 ? "on" : "off");
    Serial.print( F("Debug plot: ") );
    Serial.println( debug_plot_enabled == 0 ? "off" : "on" );
  }

  //reboot
  if ( argv[0] == "reboot" ) reboot();

  //help
  if ( argv[0] == "help" ) {
    Serial.println( F("[HELP] available commands")  );
    Serial.println( F("set threshold <n> - to set the audio sense threshold")  );
    Serial.println( F("set audio [0|1] - to enable or disable audio control") );
    Serial.println( F("set debug_plot [0|1] - to enable audio signal plot (use arduino tools->serial plotter)")  );
    Serial.println( F("status - to print device status") );
    Serial.println( F("reboot - to reboot the device") );
  }

}
