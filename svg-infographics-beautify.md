# svg-infographics-beautify directive (project-local)

## Standing directive overrides (optional; defaults in this skill)

(none - shipped defaults apply)

## Creative brief (rewritten each run, free-text from user)

"make the svg a little bit cooler" - README header banner of the audio detector firmware. Soft glow on the cyan waveform, the mint IR pulse train and the power icon; colour variation along the waveform (cyan to teal as it rises); a faint abstract sound-wave or concentric-arc motif behind the title block; keep text, layout, light/dark CSS and readability unchanged.

## Resolved pattern (rewritten by this skill each run)

2026-10-17 - target `.images_readme/01-header.svg`, output suffix `+` (`01-header+.svg`, print variant `01-header+_print.svg`)
- intensity: medium
- colour variation: moderate (per-element hue variants in family: waveform cyan to blue-teal, IR train mint variants, pills keep their state colours)
- shapes: moderate (concentric arc fragments behind the title block, a few particles near the waveform, a soft plate behind the signal band)
- icons: per-header, Lucide multi-stroke (small glyphs beside "threshold" and "IR START"; the existing power icon kept)
- embroidery: sparse, style tech (corner tick marks on banner edges, a few dash accents)
- abstract graphics: moderate (background arcs, ring fragments, particles)
- background texture: circuit (faint PCB traces with node dots in the signal band)
- background opacity cap: 0.08
- glow: focal-only, dual palette (cool cyan-mint on waveform, IR train, power icon; warm on the threshold marker dot)
- shader: subtle, light-leak (one corner); print compat strip-on-export (default, not asked); animation off (default, not asked)
- validation: strict-errors-only

## History (append-only)

# 2026-10-17 12:00 - .images_readme/01-header.svg - medium - OK
