"""Shared stage runner for the firmware environment installers in this folder.

An installer names a conda environment and lists its stages. A stage that finished leaves a marker
file in <prefix>/.firmware-env/, so a later run installs only the stages that are missing. The
full output of every command goes to ~/.local/<environment>.out.

This file is not executable, so the lab-utils menu does not list it.
"""

import importlib.util
import sys

if importlib.util.find_spec("rich") is None:  # silent under uv; reached by a plain `python script.py`
    print("Missing Python package: rich\nInstall it with: pip install rich", file=sys.stderr)
    sys.exit(1)

import collections
import functools
import hashlib
import json
import os
import re
import select
import shutil
import subprocess
import tempfile
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Callable

os.environ.setdefault("COLORTERM", "truecolor")  # WSL leaves it unset and Rich then downsamples the palette

from rich.console import Console, Group
from rich.live import Live
from rich.progress import BarColumn, Progress, TextColumn, TimeElapsedColumn
from rich.table import Table
from rich.text import Text
from rich.theme import Theme

VERSION = "1.0.0"
CONDA = os.environ.get("CONDA_CMD") or "/opt/conda/bin/conda"  # image ENV wins when set
MARKERS = ".firmware-env"

# conda sets these when the environment is activated; {prefix} is the environment's folder.
# rustup's programs find the toolchain through RUSTUP_HOME from any place on the PATH, so a
# second Rust installation in ~/.cargo/bin does not replace the environment's compiler.
VARIABLES = {
    "RUSTUP_HOME": "{prefix}/rustup",
    "CARGO_HOME": "{prefix}/cargo",
    "CARGO_INSTALL_ROOT": "{prefix}",  # `cargo install` puts programs into <prefix>/bin
    "CARGO_TARGET_X86_64_UNKNOWN_LINUX_GNU_LINKER": "x86_64-conda-linux-gnu-cc",
}
HOST_COMPILER = "gcc_impl_linux-64"  # links the build scripts and procedural macros of a firmware project

# Stellars-Tech palette, the roles the installers use
PALETTES = {
    "dark": {
        "track": "#303841", "text": "#c3c3c3", "muted": "#a5a5a5",
        "cyan": "#21a8e4", "cyan-bright": "#46bcf0", "orange": "#da8230",
        "mint": "#3fb950", "rose": "#ef4444",
    },
    "light": {
        "track": "#d4dde3", "text": "#1a3540", "muted": "#46606c",
        "cyan": "#06709c", "cyan-bright": "#05607f", "orange": "#b5641b",
        "mint": "#1a7f37", "rose": "#d94851",
    },
}


def _osc11_is_dark(reply: str) -> bool | None:
    """True for a dark background, False for light, None when the reply is not an OSC 11 colour."""
    m = re.search(r"rgba?:([0-9a-fA-F]{1,4})/([0-9a-fA-F]{1,4})/([0-9a-fA-F]{1,4})", reply)
    if not m:
        return None
    r, g, b = (int(h, 16) / (16 ** len(h) - 1) for h in m.groups())
    return 0.2126 * r + 0.7152 * g + 0.0722 * b < 0.5


def _ask_terminal(timeout: float = 0.2) -> bool | None:
    """Send the OSC 11 background query to the controlling terminal. None when there is no
    terminal or it does not answer within the timeout."""
    try:
        import termios
        import tty

        fd = os.open("/dev/tty", os.O_RDWR | os.O_NOCTTY)
    except (ImportError, OSError):
        return None
    saved = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        os.write(fd, b"\033]11;?\033\\")
        reply = b""
        while select.select([fd], [], [], timeout)[0]:
            reply += os.read(fd, 64)
            if reply.endswith((b"\033\\", b"\a")):
                break
        return _osc11_is_dark(reply.decode(errors="replace"))
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)
        os.close(fd)


def _palette() -> dict[str, str]:
    """Order: TEXTUAL_THEME naming a brand theme, the terminal's answer to OSC 11, COLORFGBG, dark."""
    wanted = os.environ.get("TEXTUAL_THEME", "").removeprefix("stellars-tech-")
    if wanted in PALETTES:
        return PALETTES[wanted]
    dark = _ask_terminal() if sys.stdin.isatty() and sys.stdout.isatty() else None
    if dark is None:
        bg = os.environ.get("COLORFGBG", "").split(";")[-1]
        dark = int(bg) in (0, 1, 2, 3, 4, 5, 6, 8) if bg.isdigit() else None
    return PALETTES["dark" if dark is not False else "light"]


def _theme(p: dict[str, str]) -> Theme:
    return Theme({
        "bar.back": p["track"], "bar.complete": p["cyan"], "bar.finished": p["mint"],
        "bar.pulse": p["cyan"], "progress.elapsed": p["muted"],
        "st.title": f"bold {p['cyan-bright']}", "st.text": p["text"], "st.muted": p["muted"],
        "st.source": p["orange"], "st.target": p["cyan"],
        "st.ok": p["mint"], "st.warn": p["orange"], "st.error": p["rose"],
    })


def find_prefix(name: str) -> Path | None:
    """The folder of the conda environment with this name, None when conda does not list one."""
    envs = json.loads(subprocess.check_output([CONDA, "env", "list", "--json"]))["envs"]
    return next((Path(p) for p in envs if Path(p).name == name), None)


@dataclass
class Stage:
    title: str  # shown in the stage list; also names the marker file
    source: str  # the server the files come from
    run: Callable[["Env"], None]


class _Bar(BarColumn):
    """Rich pulses the bar of every task that has not started; a waiting stage shows an empty bar."""

    def render(self, task):
        bar = super().render(task)
        bar.pulse = task.started and task.total is None
        return bar


class Ui:
    """The live stage list: one row per stage, and below it the last output line of the running command."""

    def __init__(self, console: Console, stages: list[Stage]):
        wide = console.width >= 110  # narrower: no source column and a shorter bar, the rows fit 80 columns
        self.progress = Progress(
            TextColumn("[st.muted]{task.fields[index]}"),
            TextColumn("[st.text]{task.description}"),
            *([TextColumn("[st.source]{task.fields[source]}")] if wide else []),
            _Bar(bar_width=20 if wide else 14),
            TextColumn("{task.fields[state]}"),
            TimeElapsedColumn(),
            console=console,
        )
        self.tasks = [
            self.progress.add_task(
                s.title, start=False, total=1, index=f"{i}/{len(stages)}", source=s.source,
                state="[st.muted]waiting",
            )
            for i, s in enumerate(stages, 1)
        ]
        self.line = Text(style="st.muted", no_wrap=True, overflow="ellipsis")
        self.live = Live(Group(self.progress, self.line), console=console)
        self.current = self.tasks[0]

    def earlier(self, index: int) -> None:
        self.progress.update(self.tasks[index], completed=1, state="[st.ok]present")

    def begin(self, index: int) -> None:
        self.current = self.tasks[index]
        self.progress.start_task(self.current)
        self.working()

    def working(self) -> None:
        self.progress.update(self.current, total=None, state="[st.target]installing")

    def downloaded(self, done: int, total: int) -> None:
        state = f"[st.target]{done / 1e6:.1f} / {total / 1e6:.1f} MB"
        self.progress.update(self.current, total=total, completed=done, state=state)

    def tail(self, text: str) -> None:
        self.line.plain = text

    def end(self, ok: bool) -> None:
        state = "[st.ok]complete" if ok else "[st.error]failed"
        self.progress.update(self.current, total=1, completed=int(ok), state=state)
        self.progress.stop_task(self.current)
        self.tail("")


class Env:
    """What a stage function works with: the environment's folder, a command runner and a downloader."""

    def __init__(self, name: str, variables: dict[str, str], ui: Ui):
        self.name, self.variables, self.ui = name, VARIABLES | variables, ui
        self.prefix = find_prefix(name)
        self.tmp = Path()  # a new temporary folder for every stage
        self.log_path = Path.home() / ".local" / f"{name}.out"
        self.recent = collections.deque(maxlen=15)

    @functools.cached_property
    def log(self):
        """Opened by the first stage that runs, so a run that installs nothing keeps the last log."""
        return open(self.log_path, "w")

    def marker(self, stage: Stage) -> Path:
        return self.prefix / MARKERS / re.sub(r"[^a-z0-9.]+", "-", stage.title.lower())

    def environ(self) -> dict[str, str]:
        """The process environment of the activated environment: its bin folder first, its variables set."""
        if self.prefix is None:
            return dict(os.environ)
        env = dict(os.environ, PATH=f"{self.prefix}/bin:{os.environ['PATH']}")
        env.update({k: v.format(prefix=self.prefix) for k, v in self.variables.items()})
        return env

    def run(self, *cmd, cwd: Path | None = None, env: dict[str, str] | None = None) -> None:
        """Run a command. Its output goes to the log file, and its last line shows under the stage list.

        env - the process environment; the activated environment by default
        """
        cmd = [str(c) for c in cmd]
        self.ui.working()
        self.log.write(f"\n$ {' '.join(cmd)}\n")
        with subprocess.Popen(
            cmd, cwd=cwd, env=env or self.environ(), text=True, errors="replace",
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        ) as process:
            for line in process.stdout:
                self.log.write(line)
                self.log.flush()
                if line.strip():
                    self.recent.append(line.rstrip())
                    self.ui.tail(line.strip())
        if process.returncode:
            raise RuntimeError(f"{Path(cmd[0]).name} exited with status {process.returncode}")

    def fetch(self, *urls: str) -> list[Path]:
        """Download files into the stage's temporary folder, with one byte count over all of them."""
        sizes = [
            int(urllib.request.urlopen(urllib.request.Request(url, method="HEAD")).headers["Content-Length"])
            for url in urls
        ]
        done, files = 0, []
        for url in urls:
            files.append(self.tmp / url.rsplit("/", 1)[-1])
            self.ui.tail(url)
            self.log.write(f"\ndownload {url}\n")
            with urllib.request.urlopen(url) as response, open(files[-1], "wb") as out:
                while chunk := response.read(1 << 16):
                    out.write(chunk)
                    done += len(chunk)
                    self.ui.downloaded(done, sum(sizes))
        self.ui.working()
        return files

    def create(self, *packages: str) -> None:
        """The first stage of every installer: the conda environment with the host C compiler, the
        packages the installer names, and the activation variables."""
        if self.prefix:  # listed by conda but without this stage's marker: an interrupted install
            self.run(CONDA, "env", "remove", "--name", self.name, "--yes")
            self.prefix = None
        self.run(
            CONDA, "create", "--name", self.name, "--yes",
            "--override-channels", "--channel", "conda-forge", HOST_COMPILER, *packages,
        )
        self.prefix = find_prefix(self.name)
        values = (f"{k}={v.format(prefix=self.prefix)}" for k, v in self.variables.items())
        self.run(CONDA, "env", "config", "vars", "set", "--name", self.name, *values)

    def rustup(self, toolchain: str, *options: str) -> None:
        """Install a Rust toolchain into the environment's folder and link its programs into bin."""
        (rustup_init,) = self.fetch("https://static.rust-lang.org/rustup/dist/x86_64-unknown-linux-gnu/rustup-init")
        rustup_init.chmod(0o755)
        self.run(
            rustup_init, "-y", "--no-modify-path", "--profile", "minimal",
            "--default-toolchain", toolchain, *options,
        )
        self.link(self.prefix / "cargo/bin")

    def platformio(self, version: str, platform: str, board: str, *tools: str, framework: str = "arduino") -> None:
        """Install PlatformIO into the environment's Python, then the platform with the toolchain and
        the framework for the board into PLATFORMIO_CORE_DIR. `pio pkg install` resolves the
        packages a build needs only inside a project, so a temporary project names the board.

        framework - the framework the temporary project names, arduino by default
        tools     - optional packages of the platform that a build does not install, such as an
                    uploader; each is installed at the version the platform requires
        """
        self.run(self.prefix / "bin/python", "-m", "pip", "install", f"platformio=={version}")
        project = self.tmp / "project"
        project.mkdir(exist_ok=True)
        (project / "platformio.ini").write_text(
            f"[env:{board}]\nplatform = {platform}\nboard = {board}\nframework = {framework}\n"
        )
        self.run("pio", "pkg", "install", cwd=project)
        core = Path(self.environ()["PLATFORMIO_CORE_DIR"])
        manifest = core / "platforms" / platform.split("@")[0] / "platform.json"
        packages = json.loads(manifest.read_text())["packages"]
        for tool in tools:
            spec = f"{packages[tool]['owner']}/{tool}@{packages[tool]['version']}"
            self.run("pio", "pkg", "install", "--tool", spec, cwd=project)

    def link(self, bin_dir: Path) -> None:
        """Symlink every program of a tool's own bin folder into the environment's bin folder."""
        for program in bin_dir.iterdir():
            link = self.prefix / "bin" / program.name
            link.unlink(missing_ok=True)
            link.symlink_to(os.path.relpath(program, link.parent))

    def circuit_simulator(self) -> None:
        """ngspice, the batch simulator that runs the ltspice component's circuit netlist."""
        self.run(
            CONDA, "install", "--yes", "--override-channels", "--channel", "conda-forge",
            "--prefix", self.prefix, "ngspice=41",
        )

    def qemu_system(self, version: str, url: str, sha256: str, target: str, machine: str) -> None:
        """Build and install a minimal QEMU system emulator from pinned sources.

        version - the upstream release, e.g. 11.1.2
        url     - the source tarball; its sha256 must equal the pinned digest
        target  - the QEMU target list, e.g. arm-softmmu
        machine - the machine the simulator tier needs; the stage fails when the build lacks it

        The build dependencies stay in the environment: removing them after the build is the
        fragile step, and they are small next to the compiler the environment already carries.
        The rpath points at the environment's lib folder, so the binary runs from the environment
        on any machine without LD_LIBRARY_PATH.
        """
        self.run(
            CONDA, "install", "--yes", "--override-channels", "--channel", "conda-forge",
            "--prefix", self.prefix, "glib", "pixman", "zlib", "meson", "ninja", "pkg-config",
        )
        (archive,) = self.fetch(url)
        digest = hashlib.sha256(archive.read_bytes()).hexdigest()
        if digest != sha256:
            raise RuntimeError(f"{url}: sha256 {digest} does not match the pinned {sha256}")
        shutil.unpack_archive(archive, self.tmp, filter="fully_trusted")
        source = self.tmp / f"qemu-{version}"
        environment = dict(self.environ(), PKG_CONFIG_PATH=f"{self.prefix}/lib/pkgconfig")
        compiler = self.prefix / "bin" / "x86_64-conda-linux-gnu-cc"
        self.run(
            source / "configure",
            f"--prefix={self.prefix}", f"--target-list={target}",
            "--without-default-features", "--disable-docs", "--disable-tools",
            "--disable-guest-agent", "--disable-slirp", "--disable-gcrypt",
            "--disable-nettle", "--disable-libssh", "--disable-curl",
            f"--cc={compiler}", f"--extra-ldflags=-Wl,-rpath,{self.prefix}/lib",
            cwd=source, env=environment,
        )
        jobs = str(os.cpu_count() or 1)
        self.run("ninja", "-C", "build", f"-j{jobs}", cwd=source, env=environment)
        self.run("ninja", "-C", "build", "install", cwd=source, env=environment)
        binary = self.prefix / "bin" / f"qemu-system-{target.split('-')[0]}"
        machines = subprocess.run(
            [binary, "-machine", "help"], capture_output=True, text=True, env=self.environ()
        ).stdout
        if machine not in machines:
            raise RuntimeError(f"{binary.name} has no {machine} machine")


def install(
    name: str, summary: str, stages: list[Stage], tools: list[tuple[str, list[str] | str]],
    usage: list[str], variables: dict[str, str] | None = None,
) -> None:
    """Run the stages that have no marker yet, then print the installed tools and the usage lines.

    tools     - rows of the closing table: a label with a command whose first output line is
                shown, or a label with a fixed text
    variables - activation variables this environment needs beyond VARIABLES, in the same form
    """
    console = Console(theme=_theme(_palette()), highlight=False)
    header = Table.grid(expand=True)
    header.add_column()
    header.add_column(justify="right")
    header.add_row(
        Text.assemble((name, "st.title"), ("  " + summary, "st.text")),
        Text(f"v{VERSION}", style="st.muted"),
    )
    console.print(header)
    console.print()

    ui = Ui(console, stages)
    env = Env(name, variables or {}, ui)
    failure = None
    try:
        with ui.live:
            for index, stage in enumerate(stages):
                if env.prefix and env.marker(stage).exists():
                    ui.earlier(index)
                    continue
                ui.begin(index)
                env.recent.clear()
                try:
                    with tempfile.TemporaryDirectory() as tmp:
                        env.tmp = Path(tmp)
                        stage.run(env)
                    env.marker(stage).parent.mkdir(exist_ok=True)
                    env.marker(stage).touch()
                except Exception as error:
                    ui.end(ok=False)
                    failure = (stage, error)
                    break
                ui.end(ok=True)
    except KeyboardInterrupt:
        console.print("cancelled", style="st.warn")
        sys.exit(130)  # same exit code as the built-in installers

    if failure:
        stage, error = failure
        console.print(Text(f'ERROR: stage "{stage.title}" failed: {error}', style="st.error"))
        for line in env.recent:
            console.print(Text("  " + line, style="st.muted"))
        console.print(Text.assemble(("Full output: ", "st.muted"), (str(env.log_path), "st.text")))
        sys.exit(1)

    table = Table.grid(padding=(0, 2))
    table.add_row(Text("environment", style="st.muted"), Text(str(env.prefix), style="st.target"))
    for label, value in tools:
        if isinstance(value, list):
            result = subprocess.run(value, env=env.environ(), capture_output=True, text=True)
            value = next(line for line in (result.stdout + result.stderr).splitlines() if line.strip())
        table.add_row(Text(label, style="st.muted"), Text(value.strip(), style="st.text"))
    console.print(table)
    console.print()
    console.print(Text("Typical usage", style="st.title"))
    for line in usage:
        console.print(Text("  " + line, style="st.text"))
