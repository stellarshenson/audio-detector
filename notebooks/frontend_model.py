"""Behavioural model of the Audio Detector Mk10 front end and of the firmware's audio level logic

The circuit follows references/schematics/audio-detector_schematics.png:

    SIG_IN -> C_AMP1 220n / R_VGND3 100k to VGND (47k/10k divider from 5 V)
           -> U1.1 LM324A, non-inverting, gain 1 + R_AMP1/(R_AMP2 + 1/sC_AMP2)
           -> C_AMP3 220n / R4 1M to GND
           -> U1.2 precision rectifier (D_PEAK inside the feedback loop)
           -> R_PEAK1 10k -> C_PEAK 1u || R_PEAK2 1M -> A1 (ADC, AVCC 5 V)

LM324 behaviour taken from the datasheet: output swings from about 5 mV to VCC - 1.5 V, input bias
current flows out of the input pins (PNP input stage), and an input driven below ground is clamped
near -0.6 V by the input transistor's collector-base junction.
"""

from dataclasses import dataclass, replace

import numpy as np
from scipy import signal

FS = 16_000  # audio sample rate of the model, Hz
VCC = 5.0  # supply and ADC reference (AVCC), V
LINE_0DBFS = 2.0  # V RMS of a full-scale sine at the RCA output (CD line level)
ADC_PERIOD = 0.051  # firmware samples when millis() - last > 50, so every 51 ms
ADC_STEP = round(ADC_PERIOD * FS)  # audio samples between two ADC samples


@dataclass(frozen=True)
class FrontEnd:
    """component values and LM324 limits; every field is a schematic value or a datasheet figure"""
    r_vgnd1: float = 47e3  # VGND divider, top
    r_vgnd2: float = 10e3  # VGND divider, bottom
    c_amp1: float = 220e-9  # input coupling
    r_vgnd3: float = 100e3  # input bias to VGND
    r_amp1: float = 100e3  # feedback
    r_amp2: float = 1e3  # gain leg
    c_amp2: float = 1e-6  # gain leg, sets the 159 Hz corner
    gbw: float = 1.2e6  # LM324 gain-bandwidth product, Hz
    vol: float = 0.005  # LM324 output low limit, V
    voh: float = VCC - 1.5  # LM324 output high limit, V
    c_amp3: float = 220e-9  # coupling to the rectifier
    r4: float = 1e6  # rectifier input bias to GND
    ib: float = 20e-9  # LM324A input bias current, typical 20 nA, maximum 100 nA
    v_clamp: float = -0.6  # input clamp below ground
    v_diode: float = 0.6  # D_PEAK forward drop, compensated by the loop but limits the top
    r_peak1: float = 10e3  # peak detector charge resistor
    c_peak: float = 1e-6  # peak detector capacitor
    r_peak2: float = 1e6  # peak detector discharge resistor

    @property
    def vgnd(self):
        return VCC * self.r_vgnd2 / (self.r_vgnd1 + self.r_vgnd2)

    @property
    def gain(self):
        return 1 + self.r_amp1 / self.r_amp2


def stage1_filter(fe):
    """digital (b, a) of the stage 1 small-signal response: input high-pass, gain leg, GBW pole"""
    hp = ([fe.r_vgnd3 * fe.c_amp1, 0], [fe.r_vgnd3 * fe.c_amp1, 1])
    leg = ([(fe.r_amp1 + fe.r_amp2) * fe.c_amp2, 1], [fe.r_amp2 * fe.c_amp2, 1])
    pole = ([1], [1 / (2 * np.pi * fe.gbw / fe.gain), 1])
    num = np.polymul(np.polymul(hp[0], leg[0]), pole[0])
    den = np.polymul(np.polymul(hp[1], leg[1]), pole[1])
    return signal.bilinear(num, den, FS)


class Circuit:
    """streams audio through the front end chunk by chunk and keeps every state between chunks"""

    def __init__(self, fe=FrontEnd()):
        self.fe = fe
        self.b, self.a = stage1_filter(fe)
        self.zi = np.zeros(max(len(self.a), len(self.b)) - 1)
        self.vc = fe.vgnd - fe.ib * fe.r4  # C_AMP3 voltage at rest: VGND on one side, Ib*R4 on the other
        self.vcap = fe.ib * fe.r4 * fe.r_peak2 / (fe.r_peak1 + fe.r_peak2)
        self.phase = 0  # audio samples since the last ADC sample

    def feed(self, vin):
        """vin: RCA voltage per audio sample; returns the A1 voltage at each ADC sample in this chunk"""
        fe = self.fe
        y, self.zi = signal.lfilter(self.b, self.a, vin, zi=self.zi)
        v1 = np.clip(fe.vgnd + y, fe.vol, fe.voh)
        dt = 1 / FS
        k3, kc, kd = dt / fe.c_amp3, dt / (fe.r_peak1 * fe.c_peak), dt / (fe.r_peak2 * fe.c_peak)
        inv_r4, ib, clamp, top = 1 / fe.r4, fe.ib, fe.v_clamp, fe.voh - fe.v_diode
        vc, vcap, phase = self.vc, self.vcap, self.phase
        out = []
        for x in v1.tolist():
            node = x - vc
            if node < clamp:  # input junction conducts and recharges C_AMP3
                node = clamp
                vc = x - clamp
            vc += k3 * (node * inv_r4 - ib)
            drive = node if node < top else top
            if drive > vcap:
                vcap += kc * (drive - vcap)
            vcap -= kd * vcap
            phase += 1
            if phase == ADC_STEP:
                out.append(vcap)
                phase = 0
        self.vc, self.vcap, self.phase = vc, vcap, phase
        return np.array(out)


def adc(volts):
    """10-bit ADC reading against AVCC"""
    return np.clip(np.floor(np.asarray(volts) / VCC * 1024), 0, 1023).astype(int)


# --- source signals -------------------------------------------------------------------------

PINK_B = [0.049922035, -0.095993537, 0.050612699, -0.004408786]  # Kellet's pink noise filter
PINK_A = [1, -2.494956002, 2.017265875, -0.522189400]


class Source:
    """an RCA source: music as band-limited pink noise under a loudness envelope, plus the
    source's own noise floor and mains hum that are present whether or not music plays"""

    def __init__(self, rng, noise_uv=10.0, hum_mv=0.3):
        self.rng = rng
        self.noise = noise_uv * 1e-6  # V RMS, DAC noise floor plus LM324 input noise
        self.hum = hum_mv * 1e-3  # V RMS of mains hum at 50 Hz with 100 and 150 Hz harmonics
        self.band = signal.butter(2, [40, 7000], "bandpass", fs=FS, output="sos")
        self.zi_pink = np.zeros(3)
        self.zi_band = np.zeros((self.band.shape[0], 2))
        self.t = 0.0
        self.pink_rms = self._calibrate()

    def _calibrate(self):
        w = np.random.default_rng(0).standard_normal(FS * 20)
        p = signal.sosfilt(self.band, signal.lfilter(PINK_B, PINK_A, w))
        return float(np.sqrt(np.mean(p[FS:] ** 2)))

    def chunk(self, seconds, level_dbfs=None, spread_db=0.0):
        """seconds of audio; level_dbfs None is silence, otherwise music whose short-term level
        wanders around level_dbfs with a standard deviation of spread_db"""
        n = int(round(seconds * FS))
        t = self.t + np.arange(n) / FS
        self.t += n / FS
        hum = self.hum * np.sqrt(2) * (np.sin(2 * np.pi * 50 * t) + 0.5 * np.sin(2 * np.pi * 100 * t)
                                       + 0.3 * np.sin(2 * np.pi * 150 * t)) / np.sqrt(1 + 0.25 + 0.09)
        v = self.noise * self.rng.standard_normal(n) + hum
        w = self.rng.standard_normal(n)
        p, self.zi_pink = signal.lfilter(PINK_B, PINK_A, w, zi=self.zi_pink)
        p, self.zi_band = signal.sosfilt(self.band, p, zi=self.zi_band)
        if level_dbfs is not None:
            # loudness wanders on a 0.25 s scale, like phrases and beats
            knots = self.rng.standard_normal(int(seconds * 4) + 2) * spread_db
            env_db = np.interp(np.arange(n) / FS, np.arange(len(knots)) / 4, knots) + level_dbfs
            v = v + LINE_0DBFS * 10 ** (env_db / 20) * p / self.pink_rms
        return v


def click(amplitude, seconds, decay):
    """a transient: amplitude in V, decaying with the time constant decay, lasting seconds"""
    t = np.arange(int(seconds * FS)) / FS
    return amplitude * np.exp(-t / decay)


# --- firmware -------------------------------------------------------------------------------

SAMPLES = 50  # AUDIOLEVEL_SAMPLES
MIN_LEVEL = 10  # AUDIOLEVEL_MIN_LEVEL
STABILISE = 3.0  # STARTUP_STABILISE_DURATION, s
START_TIMEOUT = 3.0  # AUDIO_START_TIMEOUT, s
STANDBY = 600.0  # AUDIO_STANDBY_TIMEOUT, s
CHECK = 60.0  # AUDIO_ENABLED_SENSE_INTERVAL, s


def off_level(threshold, sigma):
    """AudioLevel::offLevel()"""
    if threshold <= MIN_LEVEL:
        return threshold
    if threshold - MIN_LEVEL <= sigma:
        return MIN_LEVEL
    return threshold - sigma


def learn(counts):
    """AudioLevel learning over 50 samples: (threshold, sigma) from integer sums"""
    c = np.asarray(counts[:SAMPLES], dtype=np.int64)
    s, q = int(c.sum()), int((c * c).sum())
    mean = s // SAMPLES
    return max(mean, MIN_LEVEL) if mean > MIN_LEVEL else MIN_LEVEL, int(np.sqrt(SAMPLES * q - s * s) / SAMPLES)


def run_firmware(counts, threshold, off):
    """replays the firmware's audio decisions in timeout mode (no 12V trigger) on ADC readings
    taken every ADC_PERIOD; returns the moving average and the list of (time, event)"""
    window, average, events = [], np.zeros(len(counts)), []
    state, entered, check_at = "idle", 0.0, 0.0
    for k, c in enumerate(counts):
        t = (k + 1) * ADC_PERIOD
        window = (window + [int(c)])[-SAMPLES:]
        level = int(sum(window) / len(window))
        average[k] = level
        if state == "idle":
            if len(window) >= SAMPLES and level > threshold and t >= STABILISE:
                state, entered = "start", t
                events.append((t, "START"))
        elif state == "start":
            if t - entered >= START_TIMEOUT:
                state, entered, check_at = "on", t, t
        elif state == "on":
            if level > off and t - check_at >= CHECK:
                entered, check_at = t, t  # timeout restarted
            if t - entered >= STANDBY:
                state, window = "idle", []
                events.append((t, "OFF"))
    return average, events
