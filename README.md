# audio-detector
Audio detector that listens for an audio signal with the SPDIF TOSLINK or COAX receiver and controls audio components with the 12V trigger and IR signal

## Arduino ##
The board used is the Arduino Nano and the libraries:
* State Machine was implemented with *arduino_fsm* state machine library https://github.com/jonblack/arduino-fsm
* IR code recording and sending was implemented with *IRRemote* library https://github.com/z3t0/Arduino-IRremote


## State Machine ##
Diagram was generated by http://asciiflow.com/

	            +---------------------+
	            | state_ircode_record |
	            +---------+--------+--+
	                      ^        |
	TRIGGER_IRCODE_RECORD |        | TRIGGER_IRCODE_RECORDED
	                      |        v
	            +---------+--------+--+
	            | state_audio_sense   +----------------------+
	            +-------------+-------+                      |
	                          ^                              |
	   TRIGGER_AUDIO_DISABLED |         TRIGGER_AUDIO_SENSED |
	                          |                              v
	            +-------------+-------+             +--------+-----------+
	            | state_audio_enabled +<-----+------+ state_audio_start  |
	            +---------------------+      |      +--------------------+
	                                         +
	                                    TRIGGER_AUDIO_ENABLED


### state_audio_sense ###
This state listens for an audio signal on TOSLINK, COAX or (MK2) RCA analog channels and raises TRIGGER_AUDIO_SENSED when signal found
Alternatively, this state also listens for ircode record button press and raises TRIGGER_IRCODE_RECORD when detected

### state_ircode_record ###
This state listens for remote IR controller signal and once it's been captured, it records it to EEPROM and signals TRIGGER_IRCODE_RECORDED

Two codes are recorded consecutively:
* AUDIO START code, and the STORED LED will be flashing once every second
* AUDIO STOP code, and the STORED LED will be turned on permanently

### state_audio_start ###
This state issues the following on enter: 
* AUDIO START IRcode previosly recorded over 3.5mm jack

Next, this state enters listening for audio equipment to enable 12V HIGH state over 3.5mm INPUT jack and signals TRIGGER_AUDIO_ENABLED when detected

### state_audio_enabled ###
This state listens for the 12V LOW state over the 3.5mm INPUT jack and signals TRIGGER_AUDIO_DISABLED when detected
* When TRIGGER_AUDIO_DISABLED was detected, system will send AUDIO STOP ir code


## Circuit and PCB ##
![breadboard](https://github.com/stellarshenson/audio-detector/blob/master/audio-detector_bb.jpg)

