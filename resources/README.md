# resources

Files the firmware and its documents use, but the build does not compile.

- `graphics/` - diagrams, screenshots and images for the README
- `external/` - vendor files, captured logs and other files fetched from outside
- Project binaries - firmware images and dumps captured from the board
- `eeprom/` - EEPROM images written by `make eeprom-backup` and restored by `make eeprom-restore`
- `envs/` - the conda environment installers; `make create_environment` runs the one for this platform

Keep large binaries out of git when a link to their source is enough.

PCB schematics and board files belong in `references/schematics/`, not here. Arduino libraries
belong in `lib/vendor/` when `arduino_libraries` is `Yes`; other project libraries go under `lib/`
with a row in `lib/README.md`.
