"""Tests for the converter models and the PWM of the control system."""

import numpy as np
import pytest

from motulator.common.control import PWM
from motulator.common.model._converter import VoltageSourceConverter
from motulator.common.model._pwm import ZOH, CarrierComparison
from motulator.common.utils import abc2complex, complex2abc, dead_time_error


@pytest.mark.parametrize("pwm_cls", [ZOH, CarrierComparison], ids=["zoh", "carrier"])
@pytest.mark.parametrize("t_d", [0, 2e-6], ids=["ideal", "dead_time"])
def test_average_voltage(pwm_cls: type[ZOH | CarrierComparison], t_d: float) -> None:
    """The average switching state over a carrier period equals the duty ratios minus
    the dead-time error, which depends on the current direction."""
    T_s, d_abc, i_abc = 1e-4, [0.3, 0.55, 0.9], np.array([4, -1, -3])
    converter = VoltageSourceConverter(u_dc=540, t_d=t_d)
    converter.inp.i_c_ab = abc2complex(i_abc)
    pwm = pwm_cls(t_d=t_d)
    q_avg = 0
    for _ in range(2):  # Rising and falling edges of the carrier
        t_steps, q_abc, b_abc = pwm(T_s, d_abc)
        assert np.sum(t_steps) == pytest.approx(T_s)
        for t_step, q, b in zip(t_steps, q_abc, b_abc, strict=True):
            converter.set_gate_signals(q, b)
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
    carrier = CarrierComparison(N=10000, t_d=t_d)
    for _ in range(2):  # Reach the steady state
        carrier(T_s, d_abc)
    q_avg, b_avg = np.zeros(3), np.zeros(3)
    for _ in range(2):
        t_steps, q_abc, b_abc = carrier(T_s, d_abc)
        q_avg += t_steps @ q_abc / (2 * T_s)
        b_avg += t_steps @ b_abc / (2 * T_s)
    _, q_zoh, b_zoh = ZOH(t_d=t_d)(T_s, d_abc)
    assert q_avg == pytest.approx(q_zoh[0])
    assert b_avg == pytest.approx(b_zoh[0])
    d_real = q_avg + 0.5 * b_avg * (1 - np.sign(i_abc))
    d_err = dead_time_error(i_abc, np.array(d_abc), t_d, T_s)
    assert np.array(d_abc) - d_err == pytest.approx(d_real)


def test_blanking_across_samples() -> None:
    """Blanking continues to the next sampling period, even if its length changes."""
    pwm = CarrierComparison(N=10000, t_d=2e-6)
    pwm(1e-4, [0.5, 0.5, 0.5])
    pwm(1e-4, [0.995, 0.5, 0.5])  # Switching at 0.5 us before the end
    t_steps, q_abc, b_abc = pwm(2e-5, [0.5, 0.5, 0.5])
    assert t_steps[0] == pytest.approx(1.5e-6)
    assert (q_abc[0, 0], b_abc[0, 0], b_abc[1, 0]) == (0, 1, 0)
    assert np.sum(t_steps) == pytest.approx(2e-5)


@pytest.mark.parametrize("u_refs", [(100, 100), (100, 356.4)], ids=["linear", "limit"])
@pytest.mark.parametrize("feedforward", [True, False], ids=["ff", "no_ff"])
@pytest.mark.parametrize("average", [True, False], ids=["average", "ongoing"])
def test_realized_voltage(
    u_refs: tuple[float, float], feedforward: bool, average: bool
) -> None:
    """The realized voltage of the control system equals that of the system model,
    also near the duty-ratio limits, where the error depends on the duty ratios."""
    T_s, t_d, u_dc, i_c_ab = 1e-4, 2e-6, 540, 4 + 0j
    pwm = PWM(
        d_err=lambda i, d: dead_time_error(i, d, t_d, T_s), feedforward=feedforward
    )
    converter = VoltageSourceConverter(u_dc, t_d=t_d)
    converter.inp.i_c_ab = i_c_ab
    realized = []
    for u_ref in u_refs:
        pwm.get_realized_voltage(i_c_ab, u_dc)
        d_abc = pwm(T_s, u_ref, u_dc, w=0)
        _, q_abc, b_abc = ZOH(t_d=t_d)(T_s, d_abc)
        converter.set_gate_signals(q_abc[0], b_abc[0])
        realized.append(u_dc * converter.inp.q_c_ab)
    expected = np.mean(realized) if average else realized[-1]
    u_c_ab = pwm.get_realized_voltage(i_c_ab, u_dc, average=average)
    assert u_c_ab == pytest.approx(expected)
    if feedforward and u_refs[0] == u_refs[1]:
        assert expected == pytest.approx(u_refs[0])


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
        _, q_abc, b_abc = ZOH()(T_s, d_abc)
        converter.set_gate_signals(q_abc[0], b_abc[0])
        realized.append(600 * converter.inp.q_c_ab)
    expected = np.mean(realized) if average else realized[-1]
    u_c_ab = pwm.get_realized_voltage(i_c_ab, 600, average=average)
    assert u_c_ab == pytest.approx(expected)
    assert realized[-1] == pytest.approx(2 * (50 + 80j))


@pytest.mark.parametrize("k_comp", [0, 1.5])
def test_feedforward_current_prediction(k_comp: float) -> None:
    """The currents of the feedforward are predicted independently of k_comp."""
    T_s, w, i_c_ab = 1e-4, 2 * np.pi * 50, 4 + 1j
    currents = []

    def d_err(i_abc: np.ndarray, d_abc: np.ndarray) -> np.ndarray:
        currents.append(i_abc)
        return np.zeros(3)

    pwm = PWM(k_comp=k_comp, d_err=d_err)
    pwm.get_realized_voltage(i_c_ab, 540)
    pwm(T_s, 100, 540, w)
    expected = complex2abc(np.exp(1.5j * w * T_s) * i_c_ab)
    assert currents[-1] == pytest.approx(expected)


@pytest.mark.parametrize("t_d", [0, 2e-6], ids=["no_d_err", "dead_time"])
def test_min_pulse(t_d: float) -> None:
    """The duty ratios of the switching legs are in [d_min, 1 - d_min], those of the
    clamped legs are 0 or 1, and the realized voltage equals that of the system model,
    also in overmodulation."""
    T_s, u_dc, d_min = 1e-4, 540, 0.04
    d_err = None if t_d == 0 else lambda i, d: dead_time_error(i, d, t_d, T_s)
    pwm = PWM(d_err=d_err, d_min=d_min)
    converter = VoltageSourceConverter(u_dc, t_d=t_d)
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
        _, q_abc, b_abc = ZOH(t_d=t_d)(T_s, list(d))
        converter.set_gate_signals(q_abc[0], b_abc[0])
        u_c_ab = pwm.get_realized_voltage(i_c_ab, u_dc, average=False)
        assert u_c_ab == pytest.approx(u_dc * converter.inp.q_c_ab)
    assert limited > 50  # The limits were reached


def test_min_pulse_dead_time() -> None:
    """With the compensation of the dead time, the duty ratio of the minimum pulse is
    selected by the realized duty ratio, not by the compensated duty ratio."""
    T_s, t_d, d_min = 1e-4, 2e-6, 0.04  # Error of t_d/(2*T_s) = 0.01

    def d_err(i: np.ndarray, d: np.ndarray) -> np.ndarray:
        return dead_time_error(i, d, t_d, T_s)

    pwm = PWM(d_err=d_err, d_min=d_min)
    i_abc = np.array([4.0, -4.0, 4.0])
    d_ref = np.array([0.012, 0.012, 0.985])
    d_comp = np.clip(d_ref + d_err(i_abc, d_ref), 0, 1)  # [0.022, 0.002, 0.995]
    # Rounding the compensated duty ratio would give d_min = 0.04 in phase a, whose
    # realized duty ratio 0.03 is farther from the reference 0.012 than 0
    assert pwm.limit_pulses(list(d_comp), d_ref, i_abc) == [0, 0, 1]
    # Without the compensation, the duty ratios are rounded to the nearest
    d = [0.022, 0.002, 0.995]
    assert pwm.limit_pulses(d, np.array(d)) == [d_min, 0, 1]
    # Exhaustively, the realized duty ratio is the nearest among the allowed ones
    rng = np.random.default_rng(1)
    for _ in range(200):
        d_ref = rng.uniform(0, 1, 3) ** 4 * rng.choice([1, -1], 3) % 1
        i_abc = rng.uniform(-5, 5, 3)
        d_comp = np.clip(d_ref + d_err(i_abc, d_ref), 0, 1)
        d = np.array(pwm.limit_pulses(list(d_comp), d_ref, i_abc))
        realized = d - d_err(i_abc, d)
        for k in np.flatnonzero((0 < d_comp) & (d_comp < 1) & (d != d_comp)):
            for d_k in (0.0, d_min, 1 - d_min, 1.0):
                if abs(d_k - d_comp[k]) <= d_min:
                    other = d.copy()
                    other[k] = d_k
                    r = d_k - d_err(i_abc, other)[k]
                    assert abs(realized[k] - d_ref[k]) <= abs(r - d_ref[k]) + 1e-12


def test_min_pulse_range() -> None:
    """The minimum duty ratio must be in [0, 0.5)."""
    with pytest.raises(ValueError, match="d_min"):
        PWM(d_min=0.5)
