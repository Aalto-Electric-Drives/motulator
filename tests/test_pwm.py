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
def test_realized_voltage(u_refs: tuple[float, float], feedforward: bool) -> None:
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
    assert pwm.get_realized_voltage(i_c_ab, u_dc) == pytest.approx(np.mean(realized))
    if feedforward and u_refs[0] == u_refs[1]:
        assert np.mean(realized) == pytest.approx(u_refs[0])


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
