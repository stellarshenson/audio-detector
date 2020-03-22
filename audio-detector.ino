/*
 * IRrecord: record and play back IR signals as a minimal 
 * An IR detector/demodulator must be connected to the input RECV_PIN.
 * An IR LED must be connected to the output PWM pin 3.
 * A button must be connected to the input BUTTON_PIN and GND; this is the
 * send button.
 * A visible LED can be connected to STATUS_LED_PIN to provide status.
 *
 * The logic is:
 * If the button is pressed, send the IR code.
 * If an IR code is received, record it.
 *
 * Version 0.11 September, 2009
 * Copyright 2009 Ken Shirriff
 * http://arcfn.com
 * 
 * 
 */

#include <IRremote.h>
#include <EEPROM.h>
#include <Fsm.h>

#define RECV_PIN 11;
#define BUTTON_PIN 12;
#define STATUS_LED_PIN 13;
#define STORED_LED_PIN 10;

#define TRIGGER_IRCODE_RECORD 1
#define TRIGGER_IRCODE_RECORDED 2
#define TRIGGER_AUDIO_SENSED 3
#define TRIGGER_AUDIO_ENABLED 4
#define TRIGGER_AUDIO_DISABLED 5

IRrecv irrecv(RECV_PIN);
IRsend irsend;
decode_results results;

// Storage for the recorded code
void storeCode(decode_results *results);
void sendCode(int repeat);

int codeType = -1; // The type of code
unsigned long codeValue; // The code value if not raw
unsigned int rawCodes[RAWBUF]; // The durations if raw
int codeLen; // The length of the code
int toggle = 0; // The RC5/6 toggle state
int lastButtonState;


//set up of the state machine
void on_ircode_record_enter(); 
void on_ircode_record_loop();

State state_audio_sense(NULL, NULL, NULL);
State state_ircode_record(on_ircode_record_enter, on_ircode_record_loop, NULL);
State state_audio_enabled(NULL, NULL, NULL);
State state_audio_start(NULL, NULL, NULL);


void setup()
{
  Serial.begin(9600);
  irrecv.enableIRIn(); // Start the receiver
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(STORED_LED_PIN, OUTPUT);

  //read ircode from EEPROM
  codeValue = ( (unsigned long)EEPROM.read(0)) | ( (unsigned long)EEPROM.read(1)<<8) | ((unsigned long) EEPROM.read(2)<<16) | ( (unsigned long)EEPROM.read(3)<<24);
  codeLen = EEPROM.read(4);
  codeType = EEPROM.read(5);
  if(codeType != -1) digitalWrite(STORED_LED_PIN, HIGH);
  Serial.print("reading stored code.. ");
  Serial.println(codeValue, HEX);
  Serial.println("system initialised");

  //initialise state machine
  fsm.add_transition(&state_audio_sense, &state_ircode_record, TRIGGER_IRCODE_RECORD, NULL);
  fsm.add_transition(&state_ircode_record, &state_audio_sense, TRIGGER_IRCODE_RECORDED, NULL);
  fsm.add_transition(&state_audio_sense, &state_audio_start, TRIGGER_AUDIO_SENSED, NULL);
  fsm.add_transition(&state_audio_start, &state_audio_enabled, TRIGGER_AUDIO_ENABLED, NULL);
  fsm.add_transition(&state_audio_enabled, &state_audio_sense, TRIGGER_AUDIO_DISABLED, NULL);
}

void loop() {
  // If button pressed, send the code.
  int buttonState = digitalRead(BUTTON_PIN);
  if (lastButtonState == LOW && buttonState == HIGH) {
    //Serial.println("Released");
    irrecv.enableIRIn(); // Re-enable receiver
  }

  if (buttonState == LOW) {
    //Serial.println("Pressed, sending");
    digitalWrite(STATUS_LED_PIN, HIGH);
    sendCode(lastButtonState == buttonState);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(50); // Wait a bit between retransmissions
  } 
  else if (irrecv.decode(&results)) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    storeCode(&results);
    irrecv.resume(); // resume receiver
    digitalWrite(STATUS_LED_PIN, LOW);
  }
  lastButtonState = buttonState;
}


//////////////////////////////////////// UTILITIES ///////////////////////////////////////////////

void on_ircode_record_enter() {
  
}


void on_ircode_record_loop() {
  
}


// Stores the code for later playback
// Most of this code is just logging
void storeCode(decode_results *results) {
  codeType = results->decode_type;
  //int count = results->rawlen;
  if (codeType == UNKNOWN) {
    Serial.println("Received unknown code, saving as raw");
    codeLen = results->rawlen - 1;
    // To store raw codes:
    // Drop first value (gap)
    // Convert from ticks to microseconds
    // Tweak marks shorter, and spaces longer to cancel out IR receiver distortion
    for (int i = 1; i <= codeLen; i++) {
      if (i % 2) {
        // Mark
        rawCodes[i - 1] = results->rawbuf[i]*USECPERTICK - MARK_EXCESS;
        Serial.print(" m");
      } 
      else {
        // Space
        rawCodes[i - 1] = results->rawbuf[i]*USECPERTICK + MARK_EXCESS;
        Serial.print(" s");
      }
      Serial.print(rawCodes[i - 1], DEC);
    }
    //Serial.println("");
  }
  else {
    if (codeType == NEC) {
      Serial.print("Received NEC: ");
      if (results->value == REPEAT) {
        // Don't record a NEC repeat value as that's useless.
        Serial.println("repeat; ignoring.");
        return;
      }
    } 
    else if (codeType == SONY) {
      Serial.print("Received SONY: ");
    } 
    else if (codeType == RC5) {
      Serial.print("Received RC5: ");
    } 
    else if (codeType == RC6) {
      Serial.print("Received RC6: ");
    } 
    else {
      Serial.print("Unexpected");
      Serial.print(codeType, DEC);
      Serial.println("");
    }
    Serial.println(results->value, HEX);
    codeValue = results->value;
    codeLen = results->bits;

    //write to eeprom
    EEPROM.update(0, codeValue);
    EEPROM.update(1, codeValue >> 8);
    EEPROM.update(2, codeValue >> 16);
    EEPROM.update(3, codeValue >> 24);
    EEPROM.update(4, codeLen);
    EEPROM.update(5, codeType);
    Serial.print("written to EEPROM value: ");
    Serial.println(codeValue, HEX);
    Serial.print("reading back from EEPROM: ");
    Serial.print(EEPROM.read(3), HEX);
    Serial.print(EEPROM.read(2), HEX);
    Serial.print(EEPROM.read(1), HEX);
    Serial.print(EEPROM.read(0), HEX);
    Serial.println();
    
  }
}

void sendCode(int repeat) {
  if (codeType == NEC) {
    if (repeat) {
      irsend.sendNEC(REPEAT, codeLen);
      Serial.println("Sent NEC repeat");
    } 
    else {
      irsend.sendNEC(codeValue, codeLen);
      Serial.print("Sent NEC ");
      Serial.println(codeValue, HEX);
    }
  } 
  else if (codeType == SONY) {
    irsend.sendSony(codeValue, codeLen);
    Serial.print("Sent Sony ");
    Serial.println(codeValue, HEX);
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
    Serial.println("Sent raw");
  }
}
