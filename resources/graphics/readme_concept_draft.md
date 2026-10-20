# Concept Draft - README header banner

One wide header banner at the top of `README.md` for the audio detector firmware. The banner tells the device's job in one picture: music arrives at the RCA input, its level crosses the threshold, and the device sends the amplifier's IR code. Theme: Stellars-Tech swatch (`theme_swatch_0_stellars-tech.svg`) - cyan primary, orange accent, mint for "on"; dual light/dark CSS.

Open questions for review:
- The subtitle wording is taken from the brief; the README's first paragraph says the same in a longer form
- Headless mode: built without approval, from this draft

## 01-header - Audio Detector

```svg-infographics
canvas:  1200x300 header banner (Pattern: header, from header_banner_verdant_grounds /
         header_cover_banner_halcyon_labs). Stellars-Tech palette, light and dark CSS.

left text block (x 40..470):
  kicker:    "ARDUINO NANO  ·  FIRMWARE"   (small caps, letter-spaced, cyan)
  title:     "Audio Detector"              (large, bold)
  subtitle:  "Turns your amplifier on when the music starts"
  meta line: "RCA level sense  ·  IR remote codes  ·  relay  ·  12V trigger"

right graphic band (x 500..1160), one continuous signal from left to right:
  1. audio waveform (cyan): a smooth spline - flat near zero at the left (silence),
     then music rising into a dense oscillation with growing amplitude
     (spline through waypoints, never hand-written bezier)
  2. threshold line (orange, dashed) horizontally across the waveform, label "threshold"
     at its left end; the point where the waveform envelope first crosses it carries a
     small marker dot
  3. after the crossing, the waveform turns into an IR pulse train (mint): square
     marks and spaces of an NEC frame - one long header mark (9 ms) and short space,
     then bit marks of two widths - drawn as a rectangular step line, label "IR START"
  4. at the right end a small amplifier glyph or power icon (Lucide "power" or
     "speaker") lit in mint

state strip (bottom of the right band, small, 4 pills joined by thin arrows):
  idle  →  audio start  →  audio on  →  standby
  idle and standby in muted grey, audio start in orange, audio on in mint

background: subtle - faint grid dots or very light horizontal lines in the right band;
no photograph, no gradient fill over text.
```

Data sources: state names from `src/audio-detector.ino` (`state_audio_sense`, `state_audio_start`, `state_audio_enabled`, standby by trigger or 10 min silence); NEC timings 9000/4500 us header, 560 us bit mark, 560/1690 us spaces from `lib/vendor/IRremote/src/ir_NEC.cpp`.
