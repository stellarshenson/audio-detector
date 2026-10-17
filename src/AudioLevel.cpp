#include "AudioLevel.h"
#include <EEPROM.h>

/**
  sets up the ADC pin and the moving average, restores the threshold from EEPROM
  a stored value above the ADC maximum 1023 (erased EEPROM reads 0xFFFF) keeps the default
  a stored value below AUDIOLEVEL_MIN_THRESHOLD, written by older firmware, is raised to it
*/
void AudioLevel::begin(uint8_t pin) {
  _pin = pin;
  pinMode(_pin, INPUT);
  _average.begin(SMOOTHED_AVERAGE, AUDIOLEVEL_SAMPLES);
  _lastMillis = millis();

  uint16_t _stored = EEPROM.read(AUDIOLEVEL_EEPROM_THRESHOLD) | ( (uint16_t)EEPROM.read(AUDIOLEVEL_EEPROM_THRESHOLD + 1) << 8);
  if (_stored <= 1023) _threshold = _stored > AUDIOLEVEL_MIN_THRESHOLD ? _stored : AUDIOLEVEL_MIN_THRESHOLD;
}

/**
  takes one sample, when more than AUDIOLEVEL_INTERVAL ms have passed since the last one, into the moving average
  and, while learning, into the sum that gives the threshold
*/
bool AudioLevel::poll() {
  if (millis() - _lastMillis <= AUDIOLEVEL_INTERVAL) return false;
  _lastMillis = millis();

  _sample = analogRead(_pin);
  _average.add(_sample);
  _level = _average.get();
  if (_count < AUDIOLEVEL_SAMPLES) _count++;

  if (_learnCount < AUDIOLEVEL_SAMPLES) {
    _learnSum += _sample;

    if (++_learnCount == AUDIOLEVEL_SAMPLES) {
      //half the playing level, so the same music is well above the threshold even when the front end saturates
      uint16_t _half = _learnSum / AUDIOLEVEL_SAMPLES / 2;
      setThreshold(_half > AUDIOLEVEL_MIN_THRESHOLD ? _half : AUDIOLEVEL_MIN_THRESHOLD);
    }
  }
  return true;
}

void AudioLevel::restart() {
  _average.clear();
  _count = 0;
  _level = 0;
}

/**
  true once AUDIOLEVEL_SAMPLES samples were taken since restart() and their average is above the threshold
*/
bool AudioLevel::starts() const {
  return _count >= AUDIOLEVEL_SAMPLES && _level > _threshold;
}

/**
  true while the average is above the off level
*/
bool AudioLevel::holds() const {
  return _level > offLevel();
}

void AudioLevel::learnStart() {
  _learnCount = 0;
  _learnSum = 0;
}

bool AudioLevel::learning() const {
  return _learnCount < AUDIOLEVEL_SAMPLES;
}

void AudioLevel::setThreshold(uint16_t threshold) {
  _threshold = threshold;
  EEPROM.update(AUDIOLEVEL_EEPROM_THRESHOLD, _threshold);
  EEPROM.update(AUDIOLEVEL_EEPROM_THRESHOLD + 1, _threshold >> 8);
}
