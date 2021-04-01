# audio-detector
Audio detector that listens for an audio signal with the SPDIF TOSLINK or COAX receiver and controls audio components with the 12V trigger and IR signal

All external circuits are galvanically isolated with optoisolators, including the audio digital serial signal.


## Installation and Use ##
System was designed to read the audio signal from any decoder circuit that outputs digital serial audio. Such as WM8805 (popular in the TOSLINK - Analog Audio converters)

* Circuit connects to TOSLINK serial output (i.e. serial output from WM8805 receiver pin 16) via optocoupler on pin D8
* IR phototransistor on pin D2
* IR repeater on pin D3
* Optionally 12V audio trigger on D7

### Initial Setup ###
When powered, system indicates if any pre-recorded IR codes were found. If not, you'd need to press "IRCODE RECORD" button (pin D12). It will initiate 2-stage recording, first AUDIO_START code and AUDIO_STOP code. System gives you the recording feedback with the RECORD LED: blinks twice for the first code and once for the second code. When codes were recorded - STORED LED will be lit. Once codes were recorded and stored - the system should start up every time with STORED LED lit.

### Typical Use ###
System enters audio monitoring state when initiated. When audio was detected, system sends AUDIO START code to turn the audio system on and monitors for 12V audio trigger. If trigger was not detected in 10s, system assumes that audio is enabled.

* Next, if 12V trigger is available, system waits for the line to go LOW and sends AUDIO STOP code and starts monitoring for the audio again back in the initial state
* If 12V trigger is not available, system monitors audio signal and if audio signal is not available for 10min, sends AUDIO STOP and goes back to initial state


## Arduino ##
The board used is the Arduino Nano and the libraries:
* State Machine was implemented with *arduino_fsm* state machine library https://github.com/jonblack/arduino-fsm. This project however uses my fork of the library to allow timed transitions reset: https://github.com/stellarshenson/arduino-fsm. Pull was already requested from the *arduino-fsm* owner, hopefully my fork will be merged soon
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
![schematics](https://github.com/stellarshenson/audio-detector/blob/master/misc/audio-detector_schematics.png)
![top](https://github.com/stellarshenson/audio-detector/blob/master/misc/audio-detector_top_600dpi.png)
![bottom](https://github.com/stellarshenson/audio-detector/blob/master/misc/audio-detector_bottom_600dpi.png)
