---
name: audio-detector-toolchain
description: Manage the conda environment firmware-avr of audio-detector, choose its global or local scope when it is installed, and run the tools it carries - builds and the simavr simulator; use when a toolchain program is missing, the environment must be installed, checked or repaired, or a simulator run must be started.
---
# audio-detector toolchain skill

The project builds, uploads and simulates inside one conda environment. Run every command below from
the project root. The Makefile enters the environment with `conda run`; never activate it by hand.
Board facts and upload commands are in the `audio-detector-device` skill.

## First run, in order

1. `make help` - list the targets. Stop and report if `make` or the expected target is missing.
2. Choose the environment scope. Ask the user whether the environment should be global or local:
   - global is shared by every project that uses the same environment name and is one large copy on
     the workstation
   - local sits under `.conda/envs/` in this project, is easy to use with this project, and takes the
     same large space inside the project folder
   The project default is `global`. Ask before running. Install the environment with
   the command for the chosen scope:
   - global: `make create_environment`
   - local: `make create_environment ENV_SCOPE=local`
   The installer ends with a tool table; every tool must show a version.
3. Use the same scope on every later make command, for example `make build ENV_SCOPE=<scope>` and
   `make test ENV_SCOPE=<scope>`. A command without it uses the default `global`;
   conda then looks in the other location and reports the environment as missing, or uses a global
   environment with the same name.
4. `make build ENV_SCOPE=<scope>` - build the firmware. On failure read the first error line, not the last.
5. `make test ENV_SCOPE=<scope>` - run the host tests, the host application tests and the simulator tests. All must pass.
6. `make sim ENV_SCOPE=<scope>` - run the simulator tests. A skipped test is not a pass; read the printed reason.
7. Only after the earlier steps pass: `make upload PORT=<port> ENV_SCOPE=<scope>`, then `make host-run PORT=<port> ENV_SCOPE=<scope>`.

## Environment

- Name: `firmware-avr`
- Installer: `resources/envs/firmware-avr.py`
- Default scope: `global`
- Global: shared by every project that uses the same environment name; one large copy on the workstation
- Local: under `.conda/envs/firmware-avr` in this project; easy to use with this project, and it takes the same large space inside the project folder. A conda command run by hand in local scope needs `CONDA_ENVS_PATH=$PWD/.conda/envs`
- Form: a `uv` script with its own shebang; run it as a program, never with `python`
- Needs: `uv`, x86-64 Linux, internet, and conda at `/opt/conda/bin/conda` or in `CONDA_CMD`
- Idempotent: every finished stage leaves a marker in `<prefix>/.firmware-env/`; a rerun installs only the missing stages, and a listed environment without its conda stage marker is recreated
- Full output: `~/.local/firmware-avr.out`

Read `references/toolchains.md` when the installer fails or a tool reports an unexpected version.

## Commands

| Task | Command |
|---|---|
| Install or repair | global: `make create_environment`; local: `make create_environment ENV_SCOPE=local` |
| Verify the environment | `make build ENV_SCOPE=<scope>`, then `make test ENV_SCOPE=<scope>` |
| Confirm one tool | global: `conda run -n firmware-avr <tool> --version`; local: `CONDA_ENVS_PATH=$PWD/.conda/envs conda run -n firmware-avr <tool> --version` |
| Show the environment prefix | global: `conda run -n firmware-avr python -c "import sys; print(sys.prefix)"`; local: `CONDA_ENVS_PATH=$PWD/.conda/envs conda run -n firmware-avr python -c "import sys; print(sys.prefix)"` |
| Read the installer log | `tail -50 ~/.local/firmware-avr.out` |
| Remove the environment | global: `conda env remove -n firmware-avr`; local: `conda env remove -p .conda/envs/firmware-avr` |

## Rules

- Ask the user before running an installer or removing an environment. Both change the workstation.
- Run one scope per project. Before switching, remove the environment of the old scope; two
  environments with one name make conda choose by search order.
- Do not install a missing tool with `apt`, `pip`, `cargo install` or `rustup`; the installer owns
  every tool, and a second copy on the PATH is the usual cause of a version mismatch.
- Do not edit the installer in this project. The master lives in `envs/` of the template repository;
  report a faulty stage there.
- Build, upload and test through the make targets. They enter the environment for you.
- A stage that fails leaves its marker unwritten. Rerun the installer before you investigate.

## Simulator

1. `make sim ENV_SCOPE=<scope>`.
2. All tests must pass. A skipped test prints its reason; read it before you report the result.
3. The test tier table in `AGENTS.md` names the emulator for this board. A board marked
   `HIL only` has no emulator; run the test on hardware.
4. Confirm the emulator program; in local scope prefix a hand-run `conda run` with `CONDA_ENVS_PATH=$PWD/.conda/envs`: simavr is the PlatformIO package `tool-simavr`, used by `test/harness`.

## Failure modes

| Symptom | Cause | Action |
|---|---|---|
| `conda: command not found` | conda is not at `/opt/conda/bin/conda` | set `CONDA_CMD` to the conda binary and rerun |
| The installer stops mid-stage | network or a failed download | rerun; completed stages are skipped, the log names the failed command |
| The environment exists but a tool is missing | an interrupted stage | rerun the installer; the conda stage recreates an incomplete environment |
| `pio`, `cargo` or `avrdude` not found in a shell | the shell is not inside the environment, or it uses the other scope | use `make ENV_SCOPE=<scope>`, or `conda run -n firmware-avr` with `CONDA_ENVS_PATH=$PWD/.conda/envs` in local scope |
| `make sim` skips | the emulator is absent or the board has none | read the reason; rerun the installer when it names a missing program |
