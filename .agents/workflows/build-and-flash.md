---
description: Build the firmware, upload it to the board and confirm it answers its console
---

Build and flash audio-detector.

1. `make build`
2. `make upload PORT=$ARGUMENTS` - when no port is given, list the ports and ask the user:
   `ls /dev/ttyUSB* /dev/ttyACM*`
3. `make host-run PORT=$ARGUMENTS` - opening the port resets the board; a JSON line proves the new
   firmware booted and answers its console
4. Report the build size and that JSON line

Stop at the first failure and give the exact error text; do not retry the same command.
