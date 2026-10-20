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

## RESUME 2026-10-20 00:22

- Nothing was running; tree unchanged (271 staged entries plus the unstaged acc-crit closes)
- FIRST ACTION done: confirming round launched, run wf_001f369e-01c, launcher tmp/workflows/review-r5/confirm-r2.js (embeds args-confirm-r2.json, calls review-r5/workflow.js)
- The template working copy now has scripts/ in place of tools/ (still uncommitted, HEAD 114f979)
- Review round 5 confirming round wf_001f369e-01c: SHIP, 4 MINOR non-material open; result tmp/review/round5/confirm-r2-result.json; the owner accepted SHIP on 2026-10-20
- Closing ACC-LAYOUT-81 and 82: run wf_50f47539-77a, script tmp/workflows/close-81-82/workflow.js (README beside it); next: the journal entry through the journal plugin
- ACC-LAYOUT-81 and 82 closed (run wf_50f47539-77a: 101 passed, firmware.hex identical, 0 src/ warnings, pm-tools check 0/0); its SCOPE check failed only because the prompt range was 69-84 while the extra line is the 2026-10-19 phase 2 log on ACC-LAYOUT-67
- Journal entry 12 written through the journal plugin, journal-tools check OK
- Remaining: template commit (then .copier-answers.yml, ACC-LAYOUT-66, 67, 68), board on USB (ACC-EEPROM-63, 64, 65), simulations component decision, commit approval
- The template committed b2b32b9 (v1.0.13-1-gb2b32b9) with the renames; phase 3 compare launched: run wf_97a52571-e39, script tmp/workflows/template-adopt-p3/compare.js (read-only, checkpoint compare.json); next: review compare.json, apply, copier update test in a temporary git copy, close ACC-LAYOUT-66, 67, 68, then a review round on the phase 3 changes
- Template released v1.0.16 (c0cee19, pushed; TURMO example removed). Owner: _src_path GitHub URL, scripts/README.md with eeprom-bridge exception, references/devices/ as rendered, ltspice unselected
- Phase 3 apply launched: run wf_556a4e37-005, script tmp/workflows/template-adopt-p3/apply.js (README beside it); next: a review round on the phase 3 patches (tmp/workflows/template-adopt-p3/patches/)
- Phase 3 apply wf_556a4e37-005: four patches staged (tmp/workflows/template-adopt-p3/patches/), gates 1-4 passed; copier update test re-adds 11 removed starter files (template _skip_if_exists names whole folders; copier _main.py:1463)
- ACC-LAYOUT-66 and 67 closed (wf_1cc81a25-23e, check all_pass); ACC-LAYOUT-68 open with a log line; the owner chose a template fix: message sent to session copier-embedded-project (msg ed9f3b38), it replies to "ds audio-detector" with the release tag; then rerun the update test and close 68
- Review round 6 (phase 3 patches) launched: run wf_1194201f-4d7, launcher tmp/workflows/review-r6/launch.js (README beside it)
- The template session acknowledged: it files the defect, fixes post_gen to remove update-written files the old-version render also had, and replies with the release tag after the owner approves the release
- Review round 6 discovery wf_1194201f-4d7: PLAN, 12 MINOR, 3 text changes (tmp/review/round6/plan-r1-result.json); applying via wf_aa130819-944 (tmp/workflows/apply-r6/); next: confirming round with state plus appliedFixes
- Round 6 deferrals to keep: ACC-LAYOUT-68 closes only on a copier update that moves _commit to a newer tag (log it through pm-tools when 68 is next touched); template report additions: RELEASING.md says copier update reads the project's tags (it reads the template's), scripts/README.md renders the STM32Cube freertos.py paragraph for every platform
- Round 6 fixes applied (wf_aa130819-944, check all_pass); confirming round run wf_7b87c095-341, launcher tmp/workflows/review-r6/confirm-r2.js
- Review round 6 confirming round wf_7b87c095-341: SHIP, 0 findings; result tmp/review/round6/confirm-r2-result.json; waiting on the owner: accept or one confirmation round
- Owner on round 6: moved to the template check without asking for a confirmation round (round 6 SHIP stands)
- Template v1.0.17 released (b79ca9f, pushed): update write-back fixed. Update test and ACC-LAYOUT-68 close: run wf_18958f43-c57, script tmp/workflows/template-adopt-p3/update-v1.0.17.js; next: a pinned confirming round on patches/fix-5-update-v1.0.17.diff with the round 6 state
- ACC-LAYOUT-68 closed (wf_18958f43-c57, check all_pass): update v1.0.16 to v1.0.17 clean, only _commit changed (patches/fix-5-update-v1.0.17.diff). Pinned confirm on fix-5: run wf_018a3369-a5d, launcher tmp/workflows/review-r6/confirm-r3.js
- Pinned confirm wf_018a3369-a5d on fix-5: SHIP, 3 MINOR non-material open (AGENTS.md:99 dated v1.0.16 still true; ACC-LAYOUT-68 test line cannot fail on the write-back; ACC-LAYOUT-66 evidence cites v1.0.16); result tmp/review/round6/confirm-r3-result.json; waiting on the owner
- Owner accepted the final SHIP; tracker edits done (wf_c85c9b5a-24b, all_pass): ACC-LAYOUT-68 test line now asserts a newer tag and a clean git status, ACC-LAYOUT-66 log records _commit v1.0.17. Remaining: ACC-EEPROM-63, 64, 65 (board on USB), commit approval, sending the two deferred template text issues (owner to decide), journal entry 13
