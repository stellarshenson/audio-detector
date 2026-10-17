/*
  Audio level driver for the RCA input on an ADC pin
  samples the level, smooths it with a moving average, learns the threshold from the playing level,
  and decides with hysteresis:
  - starts(): the level is above the threshold, used to turn audio on
  - holds(): the level is above the off level, a quarter of the threshold, used to keep audio on
  quiet reads 4 to 35 on the Mk10 front end and loud music saturates near 580, so the off level sits
  near quiet and quiet music keeps audio on (notebooks/audio-level-simulation.ipynb)
  the threshold is stored in EEPROM bytes 12-13
*/
#ifndef AUDIOLEVEL_H
#define AUDIOLEVEL_H

#include <Arduino.h>
#include <Smoothed.h>         //version 1.2.0

#define AUDIOLEVEL_SAMPLES 50 //number of samples for averaging and for learning
#define AUDIOLEVEL_INTERVAL 50 //a sample is taken when more than this many ms have passed, so samples are at least 51 ms apart
#define AUDIOLEVEL_INIT_THRESHOLD 300 //initial threshold if no config available
#define AUDIOLEVEL_MIN_THRESHOLD 160 //learning, set threshold and the EEPROM restore keep the threshold at or above this, so the off level stays above quiet
#define AUDIOLEVEL_EEPROM_THRESHOLD 12 //bytes 12-13

class AudioLevel {
  public:
    void begin(uint8_t pin); //restores the threshold from EEPROM
    bool poll(); //run on every loop; takes one sample when more than AUDIOLEVEL_INTERVAL ms have passed since the last one and returns true when it did
    void restart(); //drops the samples, so starts() waits for AUDIOLEVEL_SAMPLES new ones
    bool starts() const;
    bool holds() const;
    void learnStart(); //the next AUDIOLEVEL_SAMPLES samples set the threshold
    bool learning() const;
    void setThreshold(uint16_t threshold); //saved to EEPROM
    uint16_t sample() const { return _sample; } //last ADC reading
    uint16_t level() const { return _level; } //moving average
    uint16_t threshold() const { return _threshold; }
    uint16_t offLevel() const { return _threshold / 4; }

  private:
    uint8_t _pin;
    Smoothed <float> _average;
    uint8_t _count = 0; //samples since restart, up to AUDIOLEVEL_SAMPLES
    uint32_t _lastMillis = 0;
    uint16_t _sample = 0;
    uint16_t _level = 0;
    uint16_t _threshold = AUDIOLEVEL_INIT_THRESHOLD;
    uint8_t _learnCount = AUDIOLEVEL_SAMPLES; //AUDIOLEVEL_SAMPLES when not learning
    uint32_t _learnSum = 0;
};

#endif
