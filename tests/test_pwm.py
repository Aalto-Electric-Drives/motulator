"""Tests for the converter models and the PWM of the control system."""

from functools import partial

import numpy as np
import pytest

from motulator.common.control import PWM, InverterNonlinearity
from motulator.common.control._pwm import Method
from motulator.common.model import (
    ZOH,
    CarrierComparison,
    TwoLevelModulator,
    VoltageSourceConverter,
)
from motulator.common.model._pwm import GateStates, SwitchingTimes
from motulator.common.utils import abc2complex, complex2abc


@pytest.mark.parametrize("pwm_cls", [ZOH, CarrierComparison], ids=["zoh", "carrier"])
@pytest.mark.parametrize("t_d", [0, 2e-6], ids=["ideal", "dead_time"])
def test_average_voltage(pwm_cls: type[ZOH | CarrierComparison], t_d: float) -> None:
    """The average switching state over a carrier period equals the duty ratios minus
    the dead-time error, which depends on the current direction."""
    T_s, d_abc, i_abc = 1e-4, [0.3, 0.55, 0.9], np.array([4, -1, -3])
    converter = VoltageSourceConverter(u_dc=540)
    converter.inp.i_c_ab = abc2complex(i_abc)
    pwm = pwm_cls(t_d=t_d, delay=0)
    q_avg = 0
    for _ in range(2):  # Rising and falling edges of the carrier
        t_steps, gates = pwm(T_s, d_abc)
        assert np.sum(t_steps) == pytest.approx(T_s)
        for t_step, g in zip(t_steps, gates, strict=True):
            converter.set_gates(g)
            q_avg += t_step * converter.inp.q_c_ab / (2 * T_s)
    q_ref = abc2complex(np.array(d_abc) - np.sign(i_abc) * t_d / (2 * T_s))
    assert q_avg == pytest.approx(q_ref, abs=1e-3)


@pytest.mark.parametrize(
    "d_abc", [[0.005, 0.5, 0.995], [0, 0.995, 1]], ids=["short_pulses", "clamped"]
)
def test_carrier_average_near_limits(d_abc: list[float]) -> None:
    """Near the duty-ratio limits, the carrier comparison agrees on average with the
    ZOH and with the duty-ratio error used in the control system."""
    T_s, t_d, i_abc = 1e-4, 2e-6, np.array([4, -1, -3])
    carrier = CarrierComparison(N=10000, t_d=t_d, delay=0)
    for _ in range(2):  # Reach the steady state
        carrier(T_s, d_abc)
    q_avg, b_avg = np.zeros(3), np.zeros(3)
    for _ in range(2):
        t_steps, gates = carrier(T_s, d_abc)
        q_avg += t_steps @ gates[:, 0] / (2 * T_s)
        b_avg += t_steps @ gates[:, 1] / (2 * T_s)
    _, gates_zoh = ZOH(t_d=t_d, delay=0)(T_s, d_abc)
    assert q_avg == pytest.approx(gates_zoh[0, 0])
    assert b_avg == pytest.approx(gates_zoh[0, 1])
    d_real = q_avg + 0.5 * b_avg * (1 - np.sign(i_abc))
    d_err = InverterNonlinearity(t_d)(i_abc, np.array(d_abc), 540, T_s)
    assert np.array(d_abc) - d_err == pytest.approx(d_real)


def test_blanking_across_samples() -> None:
    """Blanking continues to the next sampling period, even if its length changes."""
    pwm = CarrierComparison(N=10000, t_d=2e-6, delay=0)
    pwm(1e-4, [0.5, 0.5, 0.5])
    pwm(1e-4, [0.995, 0.5, 0.5])  # Switching at 0.5 us before the end
    t_steps, gates = pwm(2e-5, [0.5, 0.5, 0.5])
    assert t_steps[0] == pytest.approx(1.5e-6)
    assert (gates[0, 0, 0], gates[0, 1, 0], gates[1, 1, 0]) == (0, 1, 0)
    assert np.sum(t_steps) == pytest.approx(2e-5)


@pytest.mark.parametrize("pwm", [False, True], ids=["zoh", "carrier"])
@pytest.mark.parametrize("delay", [1, 2], ids=["half_period", "full_period"])
def test_single_update(pwm: bool, delay: int) -> None:
    """With single update, the sampling period consists of two half carrier periods,
    whose duty ratios are delayed by the given number of half carrier periods, and
    the mean gate signal equals the mean of their duty ratios."""
    T_s, t_d = 2e-4, 2e-6
    d_abc = [[0.25, 0.5, 0.75], [0.5, 0.25, 0.875]]
    cls = CarrierComparison if pwm else ZOH
    modulator = cls(t_d=t_d, delay=delay, single_update=True)
    modulator(T_s, d_abc[0])
    t_steps, gates = modulator(T_s, d_abc[1])
    # The same modulator called with the half carrier period and the delayed duty
    # ratios, which differ between the halves only with the delay of a half period
    d_halves = [d_abc[0], d_abc[2 - delay]]
    ref = cls(t_d=t_d, delay=0)
    expected = [ref(T_s / 2, d) for d in d_halves]
    assert t_steps == pytest.approx(np.concatenate([e[0] for e in expected]))
    assert np.array_equal(gates, np.concatenate([e[1] for e in expected]))
    # The blanking splits evenly between the switches of a leg
    q_mean = t_steps @ (gates[:, 0] + gates[:, 1] / 2) / T_s
    assert q_mean == pytest.approx(np.mean(d_halves, axis=0))


def test_blocked_gates() -> None:
    """While the gates are blocked, the gate states are zero for the whole sampling
    period, but the duty ratios pass through the computational delay."""
    converter = VoltageSourceConverter(u_dc=540, modulator=ZOH(delay=1))
    converter.set_enable(False)
    t_steps, gates = converter.modulate(1e-4, [0.2, 0.5, 0.8])
    assert t_steps == pytest.approx([1e-4])
    assert gates.shape == (1, 2, 3) and not np.any(gates)
    converter.set_enable(True)
    _, gates = converter.modulate(1e-4, [0.5, 0.5, 0.5])
    assert gates[0, 0] == pytest.approx([0.2, 0.5, 0.8])


def test_modulator_with_other_command() -> None:
    """A modulator subclass defines its command type, the initial command of the
    computational delay, and the gate states, and it drives the converter."""

    class EdgeModulator(TwoLevelModulator[list[list[float]]]):
        """Switching instants of each phase as fractions of the half carrier period."""

        def initial_command(self, command: list[list[float]]) -> list[list[float]]:
            return [[] for _ in command]  # No switching

        def schedule(
            self, T_half: float, command: list[list[float]]
        ) -> tuple[SwitchingTimes, GateStates]:
            # Each phase switches at most once, turning its upper switch on
            t_sw = np.array([T_half * t[0] if t else np.inf for t in command])
            t_n = np.unique(np.concatenate(([0.0], t_sw[t_sw < T_half])))
            q_abc = (t_n[:, np.newaxis] >= t_sw).astype(int)
            gates = np.stack([q_abc, np.zeros_like(q_abc)], axis=1)
            return np.diff(t_n, append=T_half), gates

    converter = VoltageSourceConverter(u_dc=540, modulator=EdgeModulator(delay=1))
    t_steps, gates = converter.modulate(1e-4, [[0.5], [], [0.25]])
    assert t_steps == pytest.approx([1e-4]) and not np.any(gates)  # Initial command
    t_steps, gates = converter.modulate(1e-4, [[], [], []])
    assert t_steps == pytest.approx([0.25e-4, 0.25e-4, 0.5e-4])
    assert np.array_equal(gates[:, 0], [[0, 0, 0], [0, 0, 1], [1, 0, 1]])
    converter.set_gates(gates[-1])
    assert converter.inp.q_c_ab == abc2complex([1, 0, 1])
    # The default initial command is for array commands only
    with pytest.raises(TypeError, match="initial_command"):
        ZOH()(1e-4, [[0.5], [], [0.25]])  # type: ignore[arg-type]


@pytest.mark.parametrize("method", ["SVPWM", "DPWM1"])
@pytest.mark.parametrize("u_refs", [(100, 100), (100, 356.4)], ids=["linear", "limit"])
@pytest.mark.parametrize("feedforward", [True, False], ids=["ff", "no_ff"])
@pytest.mark.parametrize("average", [True, False], ids=["average", "ongoing"])
def test_realized_voltage(
    method: Method, u_refs: tuple[float, float], feedforward: bool, average: bool
) -> None:
    """The realized voltage of the control system equals that of the system model,
    also near the duty-ratio limits, where the error depends on the duty ratios. In
    DPWM, the feedforward compensates for the error of the clamped leg."""
    T_s, t_d, u_th, u_dc, i_c_ab = 1e-4, 2e-6, 1.5, 540, 4 + 0j
    d_err = InverterNonlinearity(t_d, u_th)
    pwm = PWM(method=method, d_err=d_err, feedforward=feedforward)
    converter = VoltageSourceConverter(u_dc, u_th=u_th)
    converter.inp.i_c_ab = i_c_ab
    realized, d_abc = [], []
    for u_ref in u_refs:
        pwm.get_realized_voltage(i_c_ab, u_dc)
        d_abc = pwm(T_s, u_ref, u_dc, w=0)
        _, gates = ZOH(t_d=t_d, delay=0)(T_s, d_abc)
        converter.set_gates(gates[0])
        converter.set_outputs(0)
        realized.append(converter.out.u_c_ab)
    expected = np.mean(realized) if average else realized[-1]
    u_c_ab = pwm.get_realized_voltage(i_c_ab, u_dc, average=average)
    assert u_c_ab == pytest.approx(expected)
    if feedforward and u_refs[0] == u_refs[1]:
        assert expected == pytest.approx(u_refs[0])
        assert d_abc[0] == 1 if method == "DPWM1" else d_abc[0] < 1


@pytest.mark.parametrize("average", [True, False], ids=["average", "ongoing"])
def test_realized_voltage_follows_dc_bus(average: bool) -> None:
    """The realized voltage uses the measured DC-bus voltage, also if it has changed
    after the duty ratios were computed."""
    T_s, i_c_ab = 1e-4, 4 + 0j
    pwm = PWM()
    converter = VoltageSourceConverter(600)
    converter.inp.i_c_ab = i_c_ab
    realized = []
    for u_ref in (100, 50 + 80j):
        pwm.get_realized_voltage(i_c_ab, 300)
        d_abc = pwm(T_s, u_ref, 300, w=0)
        _, gates = ZOH(delay=0)(T_s, d_abc)
        converter.set_gates(gates[0])
        realized.append(600 * converter.inp.q_c_ab)
    expected = np.mean(realized) if average else realized[-1]
    u_c_ab = pwm.get_realized_voltage(i_c_ab, 600, average=average)
    assert u_c_ab == pytest.approx(expected)
    assert realized[-1] == pytest.approx(2 * (50 + 80j))


def test_feedforward_current_prediction() -> None:
    """The currents of the feedforward are predicted to the middle of the period."""
    T_s, w, i_c_ab = 1e-4, 2 * np.pi * 50, 4 + 1j
    currents = []

    def d_err(
        i_abc: np.ndarray, d_abc: np.ndarray, u_dc: float, T_half: float
    ) -> np.ndarray:
        currents.append(i_abc)
        return np.zeros(3)

    pwm = PWM(d_err=d_err)
    pwm.get_realized_voltage(i_c_ab, 540)
    pwm(T_s, 100, 540, w)
    expected = complex2abc(np.exp(1.5j * w * T_s) * i_c_ab)
    assert currents[-1] == pytest.approx(expected)


@pytest.mark.parametrize("method", ["SVPWM", "DPWM1"])
@pytest.mark.parametrize("t_d", [0, 2e-6], ids=["no_d_err", "dead_time"])
def test_min_pulse(method: Method, t_d: float) -> None:
    """The duty ratios of the switching legs are in [d_min, 1 - d_min], those of the
    clamped legs are 0 or 1, the limited voltage reference includes the minimum
    pulses, and the realized voltage equals that of the system model, also in
    overmodulation."""
    T_s, u_dc, d_min = 1e-4, 540, 0.04
    d_err = None if t_d == 0 else InverterNonlinearity(t_d)
    pwm = PWM(method=method, d_err=d_err, d_min=d_min)
    converter = VoltageSourceConverter(u_dc)
    rng = np.random.default_rng(0)
    limited = 0
    for _ in range(500):
        # Up to the overmodulation range (the linear range ends at u_dc/sqrt(3))
        u_ref = rng.uniform(0, 0.65 * u_dc) * np.exp(2j * np.pi * rng.uniform())
        i_c_ab = 5 * np.exp(2j * np.pi * rng.uniform())
        converter.inp.i_c_ab = i_c_ab
        pwm.get_realized_voltage(i_c_ab, u_dc)
        d_ref = np.array(pwm.duty_ratios(u_ref, u_dc))
        d = np.array(pwm(T_s, u_ref, u_dc, w=0))
        switching = (d > 0) & (d < 1)
        assert np.all(d[switching] >= d_min - 1e-12)
        assert np.all(d[switching] <= 1 - d_min + 1e-12)
        limited += np.sum(~switching | (d == d_min) | (d == 1 - d_min))
        if t_d == 0:  # Rounded to the nearest
            assert np.all(np.abs(d - d_ref) <= d_min / 2 + 1e-12)
            assert pwm.limited_voltage == pytest.approx(u_dc * abc2complex(d))
        _, gates = ZOH(t_d=t_d, delay=0)(T_s, list(d))
        converter.set_gates(gates[0])
        u_c_ab = pwm.get_realized_voltage(i_c_ab, u_dc, average=False)
        assert u_c_ab == pytest.approx(u_dc * converter.inp.q_c_ab)
    assert limited > 50  # The limits were reached


@pytest.mark.parametrize("method", ["SVPWM", "DPWM2"])
def test_min_pulse_threshold_voltage(method: Method) -> None:
    """With the compensation of the threshold voltages, the limited voltage reference
    equals the realized voltage, also in DPWM, in which the compensation shifts the
    realized duty ratios, and the minimum pulse is selected by the realized voltage."""
    T_s, u_dc, u_th, d_min = 1e-4, 540, 1.5, 0.04
    pwm = PWM(method=method, d_err=InverterNonlinearity(u_th=u_th), d_min=d_min)
    converter = VoltageSourceConverter(u_dc, u_th=u_th)
    converter.inp.i_c_ab = i_c_ab = 10 * np.exp(1j * np.pi / 4)
    # In DPWM2, phase a is clamped to the positive rail and phase b is near it
    u_ref = u_dc * abc2complex([1, 0.978, 0.45])
    pwm.get_realized_voltage(i_c_ab, u_dc)
    d_abc = pwm(T_s, u_ref, u_dc, w=0)
    _, gates = ZOH(delay=0)(T_s, d_abc)
    converter.set_gates(gates[0])
    converter.set_outputs(0)
    assert pwm.limited_voltage == pytest.approx(converter.out.u_c_ab)
    if method == "DPWM2":
        assert d_abc[:2] == [1, 1 - d_min]  # Nearer to the reference than 1


def test_min_pulse_dead_time() -> None:
    """With the compensation of the dead time, the duty ratio of the minimum pulse is
    selected by the realized duty ratio, not by the compensated duty ratio."""
    T_s, t_d, u_dc, d_min = 1e-4, 2e-6, 540, 0.04  # Error of t_d/(2*T_s) = 0.01
    nonlin = InverterNonlinearity(t_d)

    def d_err(i: np.ndarray, d: np.ndarray) -> np.ndarray:
        return nonlin(i, d, u_dc, T_s)

    pwm = PWM(d_err=nonlin, d_min=d_min)
    i_abc = np.array([4.0, -4.0, 4.0])
    d_ref = np.array([0.012, 0.012, 0.985])
    d_comp = np.clip(d_ref + d_err(i_abc, d_ref), 0, 1)  # [0.022, 0.002, 0.995]
    # Rounding the compensated duty ratio would give d_min = 0.04 in phase a, whose
    # realized duty ratio 0.03 is farther from the reference 0.012 than 0
    d_lim, _ = pwm.limit_pulses(list(d_comp), d_ref, partial(d_err, i_abc))
    assert d_lim == [0, 0, 1]
    # Without the compensation, the duty ratios are rounded to the nearest
    d = [0.022, 0.002, 0.995]
    assert pwm.limit_pulses(d, np.array(d))[0] == [d_min, 0, 1]
    # Exhaustively, the realized duty ratio is the nearest among the allowed ones
    rng = np.random.default_rng(1)
    for _ in range(200):
        d_ref = rng.uniform(0, 1, 3) ** 4 * rng.choice([1, -1], 3) % 1
        i_abc = rng.uniform(-5, 5, 3)
        d_comp = np.clip(d_ref + d_err(i_abc, d_ref), 0, 1)
        d_lim, d_realized = pwm.limit_pulses(list(d_comp), d_ref, partial(d_err, i_abc))
        d = np.array(d_lim)
        realized = d - d_err(i_abc, d)
        assert d_realized == pytest.approx(np.where(d != d_comp, realized, d_ref))
        for k in np.flatnonzero((0 < d_comp) & (d_comp < 1) & (d != d_comp)):
            for d_k in (0.0, d_min, 1 - d_min, 1.0):
                if abs(d_k - d_comp[k]) <= d_min:
                    other = d.copy()
                    other[k] = d_k
                    r = d_k - d_err(i_abc, other)[k]
                    assert abs(realized[k] - d_ref[k]) <= abs(r - d_ref[k]) + 1e-12


def test_invalid_configuration() -> None:
    """The minimum duty ratio must be in [0, 0.5), and the methods valid."""
    with pytest.raises(ValueError, match="d_min"):
        PWM(d_min=0.5)
    with pytest.raises(ValueError, match="overmodulation"):
        PWM(overmodulation="mpe")  # type: ignore[arg-type]
    with pytest.raises(ValueError, match="PWM method"):
        PWM(method="DPWM")  # type: ignore[arg-type]


@pytest.mark.parametrize(
    ("method", "segments"),
    [
        ("DPWM0", [(-60, 0)]),
        ("DPWM1", [(-30, 30)]),
        ("DPWM2", [(0, 60)]),
        ("DPWM3", [(-60, -30), (30, 60)]),
    ],
)
@pytest.mark.parametrize(
    "u_ref", [4, 200, 400], ids=["low", "linear", "overmodulation"]
)
def test_discontinuous_pwm(
    method: Method, segments: list[tuple[float, float]], u_ref: float
) -> None:
    """In DPWM, the voltage equals that of space-vector PWM, and in the linear range,
    one leg at a time is clamped: to the positive rail in the segments around the
    positive peak of its voltage reference and to the negative rail in the segments
    around the negative peak. The feedforward compensation keeps the leg clamped, also
    if the duty-ratio error changes the order of the duty ratios."""
    T_s, u_dc = 1e-4, 540
    svpwm, dpwm = PWM(), PWM(method=method)
    ff = PWM(method=method, d_err=InverterNonlinearity(t_d=2e-6, u_th=1.5))
    for theta in np.arange(0.5, 360, 1):
        u_c_ref_ab = u_ref * np.exp(1j * np.deg2rad(theta))
        d_sv = np.array(svpwm.duty_ratios(u_c_ref_ab, u_dc))
        d = np.array(dpwm.duty_ratios(u_c_ref_ab, u_dc))
        assert abc2complex(d) == pytest.approx(abc2complex(d_sv))
        if u_ref < u_dc / np.sqrt(3):
            # Angles (deg) from the positive and negative peaks of phase a
            angles = ((theta + 180) % 360 - 180, theta - 180)
            for rail, angle in zip((1, 0), angles, strict=True):
                assert (d[0] == rail) == any(a < angle < b for a, b in segments)
            clamped = (d == 0) | (d == 1)
            assert np.sum(clamped) == 1
            # Current lagging the voltage by 30°
            ff.get_realized_voltage(10 * np.exp(1j * np.deg2rad(theta - 30)), u_dc)
            d_ff = np.array(ff(T_s, u_c_ref_ab, u_dc, w=0))
            assert np.array_equal(d_ff[clamped], d[clamped])
