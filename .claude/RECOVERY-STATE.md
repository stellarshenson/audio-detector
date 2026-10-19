# Recovery state

## BRACE 2026-10-19 22:20

- HORIZON: LIMIT - `brace-horizon` printed `LIMIT 99% five_hour resets 12:00am (102m)`; the machine stays up, the same session returns after midnight
- Running workflows: none (all completed); no detached compute
- Working tree: 271 staged entries (template adoption phases 1 and 2, review round 5 fixes 1-12) plus an unstaged `docs/acc-crit-audio-detector.md` (pm-tools closes); nothing of it is committed, by design - commit needs the owner's approval

### Valid on disk

- Phase 1 and 2 of the copier-embedded-project adoption: verified (`tmp/workflows/template-adopt-p1/verify-2.json`, `tmp/workflows/template-adopt-p2/verify-1.json`); `make test` 101 passed; `firmware.hex` byte-identical to 8ff8d53 (`tmp/workflows/template-adopt-p1/baseline-firmware.hex`)
- Trackers: 76 of 84 criteria closed, 0 open defects, `pm-tools check docs/` 0 errors 0 warnings (item codes migrated)
- Review round 5 discovery (`wf_69781366-aeb`): PLAN of 3 text changes; plan, deferrals and state in `tmp/review/round5/plan-r1.json`
- Applied fixes 1-12, one patch each: `tmp/review/round5/fix-*.diff`; reports `tmp/workflows/apply-r5{,b,c}/applied.json`

### FIRST ACTION

Launch the pinned confirming round of review round 5:
`Workflow({scriptPath: "tmp/workflows/review-r5/workflow.js", args: <contents of tmp/workflows/review-r5/args-confirm-r2.json>})` (args hold the round 1 state and the 12 appliedFixes; rebuild with `python3 tmp/workflows/review-r5/build_confirm_args.py`)

### Then

- On PLAN: apply through a workflow agent, one patch per change, re-invoke with the new state; on SHIP: ask the owner accept or one confirmation round
- After SHIP: close ACC-LAYOUT-81 and 82 with evidence; journal entry via the journal plugin (phases 1-2, codes, review round 5)
- Waiting on the owner: the template commit with the renames (then `.copier-answers.yml`, ACC-LAYOUT-66, 67, 68); the board on USB (ACC-EEPROM-63, 64, 65, and the avrdude EEPROM read that decides the deferred bridge finding); simulations component yes or no; commit approval
- Template defects to report to the template session: `platformio.ini` lacks `lib_extra_dirs` for lib/vendor, lib/README.md claim, RELEASING.md targets, STM32 Makefile variable, `make clean` deletes logs/ and tmp/, `make help` gaps, toolchain skill blank lines
