# Audio Trigger / Detector
This device listens for an audio signal at the RCA input and in response turns the audio equipment on using IR remote code or a relay. It als turns the audio equipment off when there was no audio signal detected for longer than 10min




## Initial Setup ##

### Installation ###
* Connect the RCA splitter to the RCA socket of the device. 
* Connect one end of the 3.5mm jack cable to the device and the other one to the audio amplifier IR input.
* Connect mini-usb power cable 


### Use ###
When powered, system indicates if any pre-recorded IR codes were found by blinking slowly every 2 seconds. 
If not, the fast blinkng indicates that you need to set-up the device. You need to press and hold the button. 

It will initiate 2-stage recording, first AUDIO START code and AUDIO STOP code. 
System indicates the recording status with a LED: slow blinking for the 1-st stage and fast blinking for the 2-nd. 

Setting up device sensitivity is simple - done by pressing the device button once while playing audio.
The device will listen to the audio signal for a short while and set the sensitivity to the given audio volume.

### Status Leds ###
* **fast blinking after startup** - device was not configured yet, hold button to record IR codes and press button once to record trigger audio volume
* **slow blinking after startup** - everything is ok, device waits for the audio signal
* **led stays on** - audio detected

* **fast blinking after button hold** - device waits for your first infrared remote signal
* **slower blinking after button hold** - device waits for your second infrared remote signal
* **fast blinking after button press** - device learns the volume level for which it should trigger


## Arduino ##
The board used is the Arduino Nano and the libraries:
* **arduino-fsm stellarshenson fork** - State Machine was implemented with arduino_fsm state machine library https://github.com/jonblack/arduino-fsm. This project however uses my fork of the library to allow timed transitions reset: https://github.com/stellarshenson/arduino-fsm. Pull was already requested from the *arduino-fsm* owner, hopefully my fork will be merged soon
* **IRRemote v2.6.1** - IR code recording and sending was implemented with library https://github.com/z3t0/Arduino-IRremote
* **JLed v4.11** - non-blocking led library used for various blinking patterns
* **EnableInterrupt v1.1.0** - simple interrupt driver
* **Smoothed 1.2.0** - moving average for signal detection (detecting averaged value)


## State Machine ##
Diagram was generated by http://asciiflow.com/. System defines 4 states:
* state_audio_sense - the default state, device listens for an audio signal
* state_ircode_record - active after holding the button, records remote control codes
* state_audio_start - active when audio was detected, starts the audio system and moves to state_audio_enabled
* state_audio_enabled - active when audio is enabled

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



## Circuit and PCB ##
Below are the electronics circuit diagram and the board files (600dpi PNG images) for the PCB fabrication

![schematics](https://github.com/stellarshenson/audio-detector/blob/master/misc/audio-detector_schematics.png)
![top](https://github.com/stellarshenson/audio-detector/blob/master/misc/audio-detector_top_600dpi.png)
![bottom](https://github.com/stellarshenson/audio-detector/blob/master/misc/audio-detector_bottom_600dpi.png)
![3d](https://github.com/stellarshenson/audio-detector/blob/master/misc/audio-detector_3d.png)
![assembled](https://github.com/stellarshenson/audio-detector/blob/master/misc/audio-detector.png)
