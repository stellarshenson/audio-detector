.PHONY: help create_environment env build upload monitor test host-test host-run host-app-test sim eeprom-backup eeprom-restore increment_version clean

#################################################################################
# GLOBALS                                                                       #
#################################################################################

.DEFAULT_GOAL := help

PROJECT_DIR := $(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))
ENV = firmware-avr
ENV_RUN = conda run --no-capture-output -n $(ENV)
ENV_SCOPE ?= global
ifeq ($(ENV_SCOPE),local)
export CONDA_ENVS_PATH := $(PROJECT_DIR)/.conda/envs
endif

PORT ?= /dev/ttyUSB0

BAUD ?= 9600
HOST_CC = x86_64-conda-linux-gnu-cc
BUILD_DIR = build

SIMHARNESS = .pio/simharness/libsimharness.so
BRIDGE ?= scripts/eeprom-bridge
BRIDGE_ELF = $(BRIDGE)/.pio/build/nanoatmega328/firmware.elf


MSG_PREFIX = \033[1m\033[36m>>>\033[0m

#################################################################################
# COMMANDS                                                                      #
#################################################################################


## Install the conda environment with the toolchain of this project
create_environment:
	./resources/envs/firmware-avr.py

env: create_environment


## Build the firmware
build:
	@echo "$(MSG_PREFIX) building with PlatformIO"
	$(ENV_RUN) pio run

## Upload the firmware to the board on $(PORT)
upload:
	@echo "$(MSG_PREFIX) uploading to $(PORT)"
	$(ENV_RUN) pio run -t upload --upload-port $(PORT)

## Open the serial console on $(PORT) at $(BAUD) baud
monitor:
	$(ENV_RUN) pio device monitor --port $(PORT) -b $(BAUD)

## Run the host unit tests and the host application tests and the simulator tests
test: host-test host-app-test sim

host-test:
	$(ENV_RUN) pytest test/host -q

## Read the EEPROM settings from the board on $(PORT) with the host application
host-run:
	$(ENV_RUN) env PYTHONPATH=host/src python -m audiodetector_host verify --port $(PORT)

## Run the host application tests
host-app-test:
	$(ENV_RUN) env PYTHONPATH=host/src pytest host/tests -q



$(SIMHARNESS): test/harness/simharness.c
	@mkdir -p $(dir $@)
	$(ENV_RUN) sh -c 'S=$$PLATFORMIO_CORE_DIR/packages/tool-simavr; $(HOST_CC) -O2 -Wall -shared -fPIC -I$$S/include/simavr -o $(SIMHARNESS) test/harness/simharness.c -L$$S/lib -lsimavr -Wl,-rpath,$$S/lib'

$(BRIDGE_ELF): $(BRIDGE)/src/main.cpp $(BRIDGE)/platformio.ini
	$(ENV_RUN) pio run -d $(BRIDGE)

## Run the firmware in the simulator
sim: build $(SIMHARNESS) $(BRIDGE_ELF)
	$(ENV_RUN) pytest test/functional -q




# Back up the board EEPROM under resources/eeprom/ and print one JSON line on success
eeprom-backup:
	$(ENV_RUN) env PYTHONPATH=host/src python -m audiodetector_host backup --port $(PORT) --bridge $(BRIDGE)

# Write the EEPROM image $(FILE) to the board and check the reply to each write
eeprom-restore:
	$(ENV_RUN) env PYTHONPATH=host/src python -m audiodetector_host restore --port $(PORT) --yes $(FILE)


## Remove build output and caches
clean:
	@echo "$(MSG_PREFIX) removing build output and caches"
	@rm -rf .pio $(BRIDGE)/.pio target $(BUILD_DIR) .pytest_cache __pycache__
	@find . -type d -name __pycache__ -prune -exec rm -rf {} +

## Increment the patch version in VERSION
increment_version:
	@current=$$(cat VERSION); \
	major=$$(echo $$current | cut -d. -f1); \
	minor=$$(echo $$current | cut -d. -f2); \
	patch=$$(echo $$current | cut -d. -f3); \
	new_patch=$$((patch + 1)); \
	new_version="$$major.$$minor.$$new_patch"; \
	printf '%s\n' "$$new_version" > VERSION; \
	echo "Version bumped: $$current -> $$new_version"

#################################################################################
# Self Documenting Commands                                                     #
#################################################################################

define PRINT_HELP_PYSCRIPT
import sys; \
lines = sys.stdin.read().splitlines(); \
marker = chr(35) * 2 + ' '; \
pairs = [(lines[i + 1].split(':', 1)[0], lines[i][len(marker):]) for i in range(len(lines) - 1) if lines[i].startswith(marker) and ':' in lines[i + 1]]; \
pairs.sort(key=lambda pair: pair[0]); \
print('\nAvailable rules:\n'); \
print('\n'.join('\033[36m{:25}\033[0m{}'.format(*pair) for pair in pairs)); \
print()
endef
export PRINT_HELP_PYSCRIPT

## Print the list of available commands
help:
	@python3 -c "$${PRINT_HELP_PYSCRIPT}" < $(MAKEFILE_LIST)

# EOF
