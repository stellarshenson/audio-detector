/*
 * Audio detector that listens for an audio signal with the SPDIF TOSLINK or COAX receiver and controls audio components with the 12V trigger and IR signal
 * The board used in the project is the Arduino Nano and the libraries:
 * State Machine was implemented with *arduino_fsm* state machine library https://github.com/jonblack/arduino-fsm
 * IR code recording and sending was implemented with *IRRemote* library https://github.com/z3t0/Arduino-IRremote
 * 
 * Copyright by Stellars Henson 2020
 */

#include <IRremote.h>
#include <EEPROM.h>
#include <Fsm.h>

#define SEND_PIN 3
#define RECV_PIN 2
#define RECORD_PIN 12
#define STATUS_LED_PIN 4
#define STORED_LED_PIN 10
#define RECORD_LED_PIN 9
#define AUDIOSENSE_DIGITAL_PIN 8 //output from the detector circuit. MK1 has it as a SPDIF decoder serial output
#define AUDIOTRIGGER_PIN 7 //connected to the optocoupler that detects the 12V trigger from the amp
#define AUDIOTRIGGER_LED_PIN 6 //lights the LED if the amp is up and 12V trigger is high

#define TRIGGER_IRCODE_RECORD 1
#define TRIGGER_IRCODE_RECORDED 2
#define TRIGGER_AUDIO_SENSED 3
#define TRIGGER_AUDIO_ENABLED 4
#define TRIGGER_AUDIO_DISABLED 5

#define DEBUG_ENABLED 1
#define AUDIO_START_TIMEOUT 5000 //timeout for the audio start detection
#define AUDIO_NOSIGNAL_TIMEOUT 60000 //timeout for the audio shutdown if no signal


IRrecv irrecv(RECV_PIN);
IRsend irsend;
decode_results results;

// Storage for the recorded code
void sendCode(uint8_t repeat, uint8_t codeType, uint8_t codeLen, unsigned long codeValue);
void storeCode(decode_results *results, int8_t &codeType, uint8_t &codeLen, unsigned long &codeValue);


uint8_t startAudioCodeType = 255; // The type of code
uint8_t startAudioCodeLen; // The length of the code
unsigned long startAudioCodeValue; // The code value if not raw

uint8_t stopAudioCodeType = 255; // The type of code
uint8_t stopAudioCodeLen; // The length of the code
unsigned long stopAudioCodeValue; // The code value if not raw

uint8_t toggle = 0; // The RC5/6 toggle state
unsigned int rawCodes[RAWBUF]; // The durations if raw
uint8_t irCodesAvailable = 0; //if ircodes were recorded

//set up of the state machine
void on_ircode_record_enter(); 
void on_ircode_record_loop();
void on_ircode_record_exit();
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

State state_audio_sense(on_audio_sense_enter, on_audio_sense_loop, on_audio_sense_exit);
State state_ircode_record(on_ircode_record_enter, on_ircode_record_loop, on_ircode_record_exit);
State state_audio_start(on_audio_start_enter, on_audio_start_loop, on_audio_start_exit);
State state_audio_enabled(on_audio_enabled_enter, on_audio_enabled_loop, on_audio_enabled_exit);
Fsm   fsm(&state_audio_sense);

void setup()
{
  Serial.begin(9600);
  //Serial.println("\033[2J");
  
  pinMode(RECORD_PIN, INPUT_PULLUP);
  pinMode(AUDIOSENSE_DIGITAL_PIN, INPUT_PULLUP);
  pinMode(AUDIOTRIGGER_PIN, INPUT_PULLUP);
  pinMode(AUDIOTRIGGER_LED_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(STORED_LED_PIN, OUTPUT);
  pinMode(RECORD_LED_PIN, OUTPUT);

  //read ircode from EEPROM
  startAudioCodeValue = ( (unsigned long)EEPROM.read(0)) | ( (unsigned long)EEPROM.read(1)<<8) | ((unsigned long) EEPROM.read(2)<<16) | ( (unsigned long)EEPROM.read(3)<<24);
  startAudioCodeLen = EEPROM.read(4);
  startAudioCodeType = EEPROM.read(5);

  stopAudioCodeValue = ( (unsigned long)EEPROM.read(6)) | ( (unsigned long)EEPROM.read(7)<<8) | ((unsigned long) EEPROM.read(8)<<16) | ( (unsigned long)EEPROM.read(9)<<24);
  stopAudioCodeLen = EEPROM.read(10);
  stopAudioCodeType = EEPROM.read(11);

  //light "ircode stored" LED only if both start and stop codes were retrieved
  if(startAudioCodeType != 255 && stopAudioCodeType != 255) { 
    digitalWrite(STORED_LED_PIN, HIGH);
    irCodesAvailable = 1;
  }
  
  if(DEBUG_ENABLED) Serial.print("[INIT] reading stored code: [AUDIO START]  ");
  if(DEBUG_ENABLED) Serial.print(startAudioCodeValue, HEX);
  if(DEBUG_ENABLED) Serial.print(" , [AUDIO STOP]  ");
  if(DEBUG_ENABLED) Serial.println(stopAudioCodeValue, HEX);
  if(DEBUG_ENABLED && irCodesAvailable != 1) Serial.println("[INIT] ir codes are not properly recorded");
  if(DEBUG_ENABLED) Serial.println("[INIT] system initialised");

  //initialise state machine
  fsm.add_transition(&state_audio_sense, &state_ircode_record, TRIGGER_IRCODE_RECORD, NULL);
  fsm.add_transition(&state_ircode_record, &state_audio_sense, TRIGGER_IRCODE_RECORDED, NULL);
  fsm.add_transition(&state_audio_sense, &state_audio_start, TRIGGER_AUDIO_SENSED, NULL);
  fsm.add_transition(&state_audio_start, &state_audio_enabled, TRIGGER_AUDIO_ENABLED, NULL);
  fsm.add_transition(&state_audio_enabled, &state_audio_sense, TRIGGER_AUDIO_DISABLED, NULL);
  //fsm.add_timed_transition(&state_audio_start, &state_audio_enabled, AUDIO_START_TIMEOUT, &on_audio_start_timed_trans_audio_enabled);
}

void loop() {
  fsm.run_machine(); //run state loops
  delay(10);
}


// *******************************
// STATE MACHINE 
// *******************************


// STATE_AUDIO_SENSE

/**
* only announces the transition
*/
void on_audio_sense_enter() {
  if(DEBUG_ENABLED) Serial.println("[AUDIO SENSE] entering state, enabling audio sense and ircode record button");  
}

/**
* waits for either ircode record button trigger or for audio signal
* ircode button -> goes to ircode recording
* audiosense signal -> goes to start audio
*/
void on_audio_sense_loop() {
  // If button pressed, record the code.
  uint8_t recordButtonState = digitalRead(RECORD_PIN);
  uint8_t audioSenseState = digitalRead(AUDIOSENSE_DIGITAL_PIN);

  //blink status 2hz to indicate listening for audio
  blink(STATUS_LED_PIN, 1, 500);

  // button to enable recording ircode
  if (recordButtonState == LOW) {
    if(DEBUG_ENABLED) Serial.println("[AUDIO SENSE] ircode recording enabled...");
    blink(STATUS_LED_PIN, 3, 300);
    fsm.trigger(TRIGGER_IRCODE_RECORD);
  } 

  // optocoupler connected to GND and AUDIOSENSE_PIN
  if (audioSenseState == LOW) {
    if(DEBUG_ENABLED) Serial.println("[AUDIO SENSE] digital audio sensed...");
    blink(STATUS_LED_PIN, 3, 300);
    fsm.trigger(TRIGGER_AUDIO_SENSED);
  }
}

/**
* signals exit from the state
*/
void on_audio_sense_exit() {
  if(DEBUG_ENABLED) Serial.println("[AUDIO SENSE] exiting state");  
}


// IRCODE RECORD STATE

/**
* enables IR receiver and turns recording LED on
* it also resets codes recorded previously
*/
void on_ircode_record_enter() {
  // enable receiver
  irrecv.enableIRIn(); 
  
  //reset stored codes to allow saving new ones
  digitalWrite(STORED_LED_PIN, LOW);
  if(DEBUG_ENABLED) Serial.println("[IRCODE RECORD] entering state and enabling receiver");

  //reset all previosu codes
  startAudioCodeValue = 0;
  startAudioCodeType = -1;
  stopAudioCodeValue = 0;
  stopAudioCodeType = -1;
}

/**
* waits until IR remote sends the IR code 
* - records audio start code and saves to EEPROM and waits for recoding of the audio stop
* - records audio stop code and saves to EEPROM
* after this goes back to audio sense state
* 
* the STORED_LED will be flashing with 1HZ when only one code was recorded and will 
* be turned on permanently when all codes were recorded
*/
void on_ircode_record_loop() {

  //blink pattern for recording: twice for the first code, once for the second code
  if(startAudioCodeType == 255 && stopAudioCodeType == 255) { blink(STORED_LED_PIN, 2, 500); delay(500);  }
  if(startAudioCodeType != 255 && stopAudioCodeType == 255) { blink(STORED_LED_PIN, 1, 500); delay(500);  }
  
  //read and decode the ircode, ircode was received with the interrupts
  if (irrecv.decode(&results)) {
    //record start audio code
    if( startAudioCodeType == 255 ) {
      if(DEBUG_ENABLED) Serial.println("[IRCODE RECORD][1/2] audio start ircode detected");
      digitalWrite(STATUS_LED_PIN, HIGH);
      blink(STORED_LED_PIN, 1, 300); // blink recoded led to indicate recording
      storeCode(&results, startAudioCodeType, startAudioCodeLen, startAudioCodeValue);
      digitalWrite(STATUS_LED_PIN, LOW);
  
      //write the to eeprom and report on serial
      EEPROM.update(0, startAudioCodeValue);
      EEPROM.update(1, startAudioCodeValue >> 8);
      EEPROM.update(2, startAudioCodeValue >> 16);
      EEPROM.update(3, startAudioCodeValue >> 24);
      EEPROM.update(4, startAudioCodeLen);
      EEPROM.update(5, startAudioCodeType);
      if(DEBUG_ENABLED) Serial.print("[IRCODE RECORD][1/2] written to EEPROM value: ");
      if(DEBUG_ENABLED) Serial.println(startAudioCodeValue, HEX);
      if(DEBUG_ENABLED) Serial.println("[IRCODE RECORD][1/2] waiting for audio stop code... ");

      irrecv.resume(); //resume recording for the stop code
    } 
    //record stop audio code
    else if( startAudioCodeType != 255 && stopAudioCodeType == 255 ) {
      if(DEBUG_ENABLED) Serial.println("[IRCODE RECORD][2/2] audio stop ircode detected");
      digitalWrite(STATUS_LED_PIN, HIGH);
      blink(STORED_LED_PIN, 1, 300); // blink recoded led to indicate recording
      storeCode(&results, stopAudioCodeType, stopAudioCodeLen, stopAudioCodeValue);
      digitalWrite(STATUS_LED_PIN, LOW);
  
      //write the to eeprom and report on serial
      EEPROM.update(6, stopAudioCodeValue);
      EEPROM.update(7, stopAudioCodeValue >> 8);
      EEPROM.update(8, stopAudioCodeValue >> 16);
      EEPROM.update(9, stopAudioCodeValue >> 24);
      EEPROM.update(10, stopAudioCodeLen);
      EEPROM.update(11, stopAudioCodeType);
      if(DEBUG_ENABLED) Serial.print("[IRCODE RECORD][2/2] written to EEPROM value: ");
      if(DEBUG_ENABLED) Serial.println(stopAudioCodeValue, HEX);

      //trigger transition
      fsm.trigger(TRIGGER_IRCODE_RECORDED);  //trigger transition only after audio stop code was recorded
    }    
  }
}

/**
* announces exit from ircode recording state and turns off recording LED
*/
void on_ircode_record_exit() {
  if(DEBUG_ENABLED) Serial.println("[IRCODE RECORD] exiting state and disabling receiver");
  digitalWrite(RECORD_LED_PIN, LOW);
  digitalWrite(STORED_LED_PIN, HIGH);
}

// AUDIO START STATE

/**
* sends recorded IR code on enter
*/
void on_audio_start_enter() {
  Serial.println("[AUDIO START] entering state, sending IR audio start signal and monitoring system startup");

  //send audio start IR signal
  sendCode(0, startAudioCodeType, startAudioCodeLen, startAudioCodeValue); //send recorded or saved IR code without repeat
}

/**
* waits for the external 12V trigger (typically output from an amplifier
* MK2 - if the external 12V trigger was not detected in AUDIOTRIGGER_TIMEOUT, system enables its own 12V trigger relay
* once 12V trigger was enabled or detected, move to AUDIO_ENABLED state
*/
void on_audio_start_loop() {
  uint8_t audioTriggerState = digitalRead(AUDIOTRIGGER_PIN);  //read the status of the external 12V trigger via optocoupler circuit connected to GND

  //blink 1hz to indicate waiting for audio enabled
  blink(AUDIOTRIGGER_LED_PIN, 1, 500);

  if (audioTriggerState == LOW) {
    if(DEBUG_ENABLED) Serial.println("[AUDIO START] external 12V trigger HIGH detected, audio amplifier is now enabled...");
    blink(STATUS_LED_PIN, 1, 300); // blink status led that the audio is enabled
    digitalWrite(AUDIOTRIGGER_LED_PIN, HIGH); //indicate that the audio is enabled
    fsm.trigger(TRIGGER_AUDIO_ENABLED); //initiate state transition
  }
}

/**
* announce exit from the audio start state
*/
void on_audio_start_exit() {
  if(DEBUG_ENABLED) Serial.println("[AUDIO START] exiting state, audio is on");
}

/**
* timed transition to audio enabled state after AUDIO_START_TIMEOUT (in miliseconds)
*/
void on_audio_start_timed_trans_audio_enabled() {
  if(DEBUG_ENABLED) Serial.println("[AUDIO START] timeout when waiting for audio start, assuming audio was enabled");
}

// AUDIO ENABLED STATE

/**
* announce enter audio enabled state
*/
void on_audio_enabled_enter() {
  if(DEBUG_ENABLED) Serial.println("[AUDIO ENABLED] entering state and monitoring for standby");
}

/**
* listens for the 12V trigger to go LOW (AUDIOTRIGGER_PIN will go up becasue of optocoupler connected to GND and the pullup sense pin)
* when 12V LOW state is detected, it indicates that the amp is in standby. System goes back to AUDIO SENSE state
*/
void on_audio_enabled_loop() {
  uint8_t audioTriggerState = digitalRead(AUDIOTRIGGER_PIN);  //read the status of the external 12V trigger via optocoupler circuit connected to GND 
  
  if (audioTriggerState == HIGH) {
    if(DEBUG_ENABLED) Serial.println("[AUDIO ENABLED] external 12V trigger LOW detected, audio amplifier entered standby...");
    blink(STATUS_LED_PIN, 3, 300); // blink status led that the audio is enabled
    fsm.trigger(TRIGGER_AUDIO_DISABLED); //initiate state transition
  }
}

/**
* announce leaving the audio enabled state
*/
void on_audio_enabled_exit() {
  if(DEBUG_ENABLED) Serial.println("[AUDIO ENABLED] exiting state and sending audio stop IR signal");

  //send audio start IR signal
  sendCode(0, stopAudioCodeType, stopAudioCodeLen, stopAudioCodeValue); //send recorded or saved IR code without repeat
  digitalWrite(AUDIOTRIGGER_LED_PIN, LOW); //indicate that the audio is disabled
}


/** 
* Stores the code for later playback
* Most of this code is just logging 
*/
void storeCode(decode_results *results, uint8_t &codeType, uint8_t &codeLen, unsigned long &codeValue) {
  codeType = results->decode_type;
  uint8_t count = results->rawlen;
  if (codeType == UNKNOWN) {
    if(DEBUG_ENABLED) Serial.println("Received unknown code, saving as raw");
    codeLen = results->rawlen - 1;
    // To store raw codes:
    // Drop first value (gap)
    // Convert from ticks to microseconds
    // Tweak marks shorter, and spaces longer to cancel out IR receiver distortion
    for (int i = 1; i <= codeLen; i++) {
      if (i % 2) {
        // Mark
        rawCodes[i - 1] = results->rawbuf[i]*USECPERTICK - MARK_EXCESS;
        if(DEBUG_ENABLED) Serial.print(" m");
      } 
      else {
        // Space
        rawCodes[i - 1] = results->rawbuf[i]*USECPERTICK + MARK_EXCESS;
        if(DEBUG_ENABLED) Serial.print(" s");
      }
      if(DEBUG_ENABLED) Serial.print(rawCodes[i - 1], DEC);
    }
    if(DEBUG_ENABLED) Serial.println("");
  }
  else {
    if (codeType == NEC) {
      if(DEBUG_ENABLED) Serial.print("Received NEC: ");
      if (results->value == REPEAT) {
        // Don't record a NEC repeat value as that's useless.
        if(DEBUG_ENABLED) Serial.println("repeat; ignoring.");
        return;
      }
    } 
    else if (codeType == SONY) {
      if(DEBUG_ENABLED) Serial.print("Received SONY: ");
    } 
    else if (codeType == RC5) {
      if(DEBUG_ENABLED) Serial.print("Received RC5: ");
    } 
    else if (codeType == RC6) {
      if(DEBUG_ENABLED) Serial.print("Received RC6: ");
    } 
    else {
      if(DEBUG_ENABLED) Serial.print("Unexpected");
      if(DEBUG_ENABLED) Serial.print(codeType, DEC);
      if(DEBUG_ENABLED) Serial.println("");
    }
    if(DEBUG_ENABLED) Serial.println(results->value, HEX);
    codeValue = results->value;
    codeLen = results->bits;
  }
}

/**
* sends the IR code recorded in the global variables: codeType, codeValue, codeLen
* the IR code is sent using PWM pin and IR LED
*/
void sendCode(uint8_t repeat, uint8_t codeType, uint8_t codeLen, unsigned long codeValue) {
  if (codeType == NEC) {
    if (repeat) {
      irsend.sendNEC(REPEAT, codeLen);
      if(DEBUG_ENABLED) Serial.println("Sent NEC repeat");
    } 
    else {
      irsend.sendNEC(codeValue, codeLen);
      if(DEBUG_ENABLED) Serial.print("Sent NEC ");
      if(DEBUG_ENABLED) Serial.println(codeValue, HEX);
    }
  } 
  else if (codeType == SONY) {
    irsend.sendSony(codeValue, codeLen);
    if(DEBUG_ENABLED) Serial.print("Sent Sony ");
    if(DEBUG_ENABLED) Serial.println(codeValue, HEX);
  } 
 
  else if (codeType == RC5 || codeType == RC6) {
    if (!repeat) {
      // Flip the toggle bit for a new button press
      toggle = 1 - toggle;
    }
    // Put the toggle bit into the code to send
    codeValue = codeValue & ~(1 << (codeLen - 1));
    codeValue = codeValue | (toggle << (codeLen - 1));
    if (codeType == RC5) {
      //Serial.print("Sent RC5 ");
      //Serial.println(codeValue, HEX);
      irsend.sendRC5(codeValue, codeLen);
    } 
    else {
      irsend.sendRC6(codeValue, codeLen);
      //Serial.print("Sent RC6 ");
      //Serial.println(codeValue, HEX);
    }
  } 
  else if (codeType == UNKNOWN /* i.e. raw */) {
    // Assume 38 KHz
    irsend.sendRaw(rawCodes, codeLen, 38);
    if(DEBUG_ENABLED) Serial.println("Sent raw");
  }
}

/**
* blinks pin with a led #cycles number of times
*/
static void  blink( const byte pin, const byte cycles, const unsigned int duration){
  uint8_t initialState = digitalRead(pin);
  for(unsigned short i=0; i<cycles * 2; i++){
    digitalWrite(pin, ~(i+initialState) & 1);
    delay(duration / 2);
  }
}
