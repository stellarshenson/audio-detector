# references

Reference information the firmware cites, kept beside the code.

- `datasheets/` - vendor datasheets and manuals, one file per part, named `<part>-<document>.pdf`
- `devices/` - one reference per device the project works with, in the format of `devices/README.md`: identity, connection, device protocol and code, host protocol and code
- `notes/` - extracts from them: register maps, timing tables, errata, each naming its datasheet and page
- `schematics/` - the PCB design files (DipTrace `.dch`, `.dip`), the schematic image the README shows, and `archive/` with the earlier board revisions

Cite the file and the page in a code comment or a design decision, so a later reader can check the
value against the source. Do not put build input here: the firmware build reads `src/` and `lib/`
only.
