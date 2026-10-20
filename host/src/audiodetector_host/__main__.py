"""Back up and restore the EEPROM of the board over the serial console.

python -m audiodetector_host {backup,restore,decode,verify} ...

The command never prompts and prints one JSON line on success; eeprom_tool holds the commands.
"""

from .eeprom_tool import main

if __name__ == "__main__":
    raise SystemExit(main())
