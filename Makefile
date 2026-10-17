# Every target runs inside the firmware-avr conda environment
ENV = conda run --no-capture-output -n firmware-avr
SHIM = .pio/simharness/libsimharness.so
PORT ?= /dev/ttyUSB0
EEPROM_OUT ?= eeprom
BRIDGE ?= tools/eeprom-bridge
BRIDGE_ELF = $(BRIDGE)/.pio/build/nanoatmega328/firmware.elf

.PHONY: build test eeprom-backup eeprom-restore

build:
	$(ENV) pio run

$(SHIM): test/harness/simharness.c
	$(ENV) sh -c 'S=$$PLATFORMIO_CORE_DIR/packages/tool-simavr && mkdir -p $(@D) && \
		x86_64-conda-linux-gnu-cc -O2 -Wall -shared -fPIC -I$$S/include/simavr -o $@ $< \
		-L$$S/lib -lsimavr -Wl,-rpath,$$S/lib'

$(BRIDGE_ELF): $(BRIDGE)/src/main.cpp $(BRIDGE)/platformio.ini
	$(ENV) pio run -d $(BRIDGE)

test: build $(SHIM) $(BRIDGE_ELF)
	$(ENV) pytest

eeprom-backup:
	$(ENV) python tools/eeprom_tool.py backup --port $(PORT) --out $(EEPROM_OUT) --bridge $(BRIDGE)

eeprom-restore:
	$(ENV) python tools/eeprom_tool.py restore --port $(PORT) --yes $(FILE)
