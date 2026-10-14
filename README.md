# Audio Trigger / Detector
This device listens for an audio signal at the RCA input and in response turns the audio equipment on using an IR remote code and a relay output. It turns the audio equipment off when the amplifier's 12V trigger goes low, or when no audio signal was detected for longer than 10 minutes.

The firmware runs on an Arduino Nano and is a single sketch, `src/audio-detector.ino`, built with PlatformIO. The libraries it uses are bundled in `lib/`. The PCB design files and board images are in `misc/`.


## Initial Setup ##

### Installation ###
* Connect the RCA splitter to the RCA socket of the device
* Connect one end of the 3.5mm jack cable to the device and the other one to the audio amplifier IR input
* Optionally connect the amplifier's 12V trigger output to the trigger connector at the back of the device
* Connect the mini-USB power cable


### Use ###
When powered, the system indicates if any pre-recorded IR codes were found by blinking slowly every 2 seconds.
If not, the fast blinking indicates that you need to set up the device. You need to press and hold the button.

It will initiate 2-stage recording, first the AUDIO START code and then the AUDIO STOP code.
The system indicates the recording status with a LED: fast blinking for the 1st stage and slower blinking for the 2nd.

Setting up device sensitivity is simple - done by pressing the device button once while playing audio.
The device will listen to the audio signal for a short while and set the sensitivity to the given audio volume.
Codes and sensitivity are stored in EEPROM and survive a power cycle.

### 12V Trigger and Standby ###
When audio is detected, the device sends the AUDIO START code and sets the audio-enabled output high (relay). It then waits 3 seconds for the amplifier's 12V trigger:
* **12V trigger detected** - the device follows the trigger and returns to listening when the amplifier goes to standby
* **12V trigger not detected** - the device checks the audio signal every minute and returns to listening after 10 minutes of silence

The AUDIO STOP code is sent on leaving the audio-enabled state only when the auto-standby configuration pin (D6) is connected to GND.

### Status Leds ###
* **fast blinking after startup** - device was not configured yet, hold button to record IR codes and press button once to record trigger audio volume
* **slow blinking after startup** - everything is ok, device waits for the audio signal
* **led stays on** - audio detected

* **fast blinking after button hold** - device waits for your first infrared remote signal
* **slower blinking after button hold** - device waits for your second infrared remote signal
* **long blinks after button press** - device learns the volume level for which it should trigger

### Serial Command Line ###
The device accepts commands over USB serial at 9600 baud:
* `help` - list the commands
* `status` - print the threshold, current audio level, audio state, auto-standby config, 12V trigger state and debug plot state
* `set threshold <n>` - set the audio sense threshold and save it to EEPROM
* `set audio [0|1]` - turn the audio off or on
* `set debug_plot [0|1]` - stream the audio signal to the Arduino Serial Plotter
* `reboot` - restart the device


## Arduino ##
The board used is the Arduino Nano and the libraries:
* **arduino-fsm stellarshenson fork** - State Machine was implemented with arduino_fsm state machine library https://github.com/jonblack/arduino-fsm. This project however uses my fork of the library to allow timed transitions reset: https://github.com/stellarshenson/arduino-fsm. Pull was already requested from the *arduino-fsm* owner, hopefully my fork will be merged soon
* **IRRemote v2.6.1** - IR code recording and sending was implemented with library https://github.com/z3t0/Arduino-IRremote
* **JLed v4.11** - non-blocking led library used for various blinking patterns
* **EnableInterrupt v1.1.0** - simple interrupt driver
* **Smoothed 1.2.0** - moving average for signal detection (detecting averaged value)

### Build ###
The project builds with PlatformIO (`platformio.ini`, environment `nanoatmega328`). In the lab the `firmware-avr` conda environment provides PlatformIO with the matching `atmelavr` platform:
```
conda activate firmware-avr
pio run                                                # build
pio run --target upload --upload-port <serial device>  # build and flash
pio device monitor                                     # serial command line at 9600 baud
```
Boards with the new Nano bootloader need `board = nanoatmega328new` in `platformio.ini`.


## State Machine ##
The system defines 5 states:
* state_audio_sense - the default state, device listens for an audio signal and for the button
* state_ircode_record - active after holding the button, records remote control codes
* state_audio_learn - active after pressing the button once, learns the audio sense threshold
* state_audio_start - active when audio was detected, starts the audio system and waits for the 12V trigger
* state_audio_enabled - active when audio is enabled, waits for standby

| From | To | Trigger |
|---|---|---|
| state_audio_sense | state_ircode_record | TRIGGER_IRCODE_RECORD (button hold) |
| state_ircode_record | state_audio_sense | TRIGGER_IRCODE_RECORDED |
| state_audio_sense | state_audio_learn | TRIGGER_AUDIO_LEARN (button press) |
| state_audio_learn | state_audio_sense | TRIGGER_AUDIO_LEARNED |
| state_audio_sense | state_audio_start | TRIGGER_AUDIO_DETECTED |
| state_audio_start | state_audio_enabled | TRIGGER_AUDIO_ENABLED (12V trigger) or 3s timeout |
| state_audio_enabled | state_audio_sense | TRIGGER_AUDIO_DISABLED (12V trigger low) or 10min timeout |
| state_audio_enabled | state_audio_learn | TRIGGER_AUDIO_LEARN (button press) |
| state_audio_enabled | state_ircode_record | TRIGGER_IRCODE_RECORD (button hold) |


## Circuit and PCB ##
Below are the electronics circuit diagram and the board files (600dpi PNG images) for the PCB fabrication

![schematics](https://github.com/stellarshenson/audio-detector/blob/master/misc/audio-detector_schematics.png)
![top](https://github.com/stellarshenson/audio-detector/blob/master/misc/audio-detector_top_600dpi.png)
![bottom](https://github.com/stellarshenson/audio-detector/blob/master/misc/audio-detector_bottom_600dpi.png)
![3d](https://github.com/stellarshenson/audio-detector/blob/master/misc/audio-detector_3d.png)
![assembled](https://github.com/stellarshenson/audio-detector/blob/master/misc/audio-detector.png)
