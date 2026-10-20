/**
   temporary EEPROM bridge firmware

   Serves only the eeprom dump and eeprom write commands over the 9600 baud
   console and never writes the EEPROM on its own. Flash it when the board
   runs firmware without those commands, take the backup with
   host/src/audiodetector_host/eeprom_tool.py, then flash the saved original
   image back.
*/

#include <Arduino.h>
#include <EEPROM.h>

#define EEPROM_MAX_ADDRESS 1023 //the ATmega328P EEPROM size minus one
#define EEPROM_BLOCK_SIZE 16 //bytes printed or written by one eeprom command

char command[65]; //one console line, at most 64 characters as in the main firmware
uint8_t command_length = 0;

uint8_t hexValue(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return 0xFF;
}

void printHexByte(uint8_t value) {
  const char digits[] = "0123456789ABCDEF";
  Serial.print(digits[value >> 4]);
  Serial.print(digits[value & 0x0F]);
}

/** split the line on spaces in place and return the number of tokens */
uint8_t split(char *text, char **argv, uint8_t max) {
  uint8_t argc = 0;
  while (*text && argc < max) {
    while (*text == ' ') text++;
    if (!*text) break;
    argv[argc++] = text;
    while (*text && *text != ' ') text++;
    if (*text) *text++ = 0;
  }
  return argc;
}

/** parse a decimal EEPROM address; false when the text is not a number from 0 to EEPROM_MAX_ADDRESS */
bool parseAddress(const char *text, long &address) {
  if (!*text || strlen(text) > 4) return false;
  long value = 0;
  for (uint8_t i = 0; text[i]; i++) {
    if (text[i] < '0' || text[i] > '9') return false;
    value = value * 10 + (text[i] - '0');
  }
  if (value > EEPROM_MAX_ADDRESS) return false;
  address = value;
  return true;
}

void handle(char *line) {
  char *argv[4];
  uint8_t argc = split(line, argv, 4);
  if (argc == 0) return;

  if (!strcmp(argv[0], "help")) {
    Serial.println(F("[HELP] eeprom bridge"));
    Serial.println(F("eeprom dump <0-1008> - to print 16 EEPROM bytes as hex"));
    Serial.println(F("eeprom write <0-1023> <hex> - to write 1 to 16 EEPROM bytes"));
    return;
  }

  if (strcmp(argv[0], "eeprom") || argc < 3) return;
  long address;
  if (!strcmp(argv[1], "dump") && parseAddress(argv[2], address) && address <= EEPROM_MAX_ADDRESS - EEPROM_BLOCK_SIZE + 1) {
    Serial.print(F("[EEPROM] "));
    Serial.print(address);
    Serial.print(' ');
    for (uint8_t i = 0; i < EEPROM_BLOCK_SIZE; i++) printHexByte(EEPROM.read(address + i));
    Serial.println();
  }
  if (argc > 3 && !strcmp(argv[1], "write") && parseAddress(argv[2], address)) {
    const char *hex = argv[3];
    uint8_t count = strlen(hex) / 2;
    bool valid = strlen(hex) % 2 == 0 && count >= 1 && count <= EEPROM_BLOCK_SIZE && address + count <= EEPROM_MAX_ADDRESS + 1;
    for (uint8_t i = 0; valid && hex[i]; i++) if (hexValue(hex[i]) == 0xFF) valid = false;
    if (valid) {
      for (uint8_t i = 0; i < count; i++) EEPROM.update(address + i, (hexValue(hex[i * 2]) << 4) | hexValue(hex[i * 2 + 1]));
      Serial.print(F("[EEPROM] wrote "));
      Serial.print(count);
      Serial.print(F(" bytes at "));
      Serial.println(address);
    }
  }
}

void setup() {
  Serial.begin(9600);
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      command[command_length] = 0;
      command_length = 0;
      handle(command);
    } else if (command_length < sizeof(command) - 1) {
      command[command_length++] = c;
    }
  }
}
