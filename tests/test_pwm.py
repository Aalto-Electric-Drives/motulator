"""Tests for the converter models."""

import numpy as np
import pytest

from motulator.common.model._pwm import ZOH, CarrierComparison
from motulator.common.utils import abc2complex


@pytest.mark.parametrize("pwm", [ZOH(), CarrierComparison()], ids=["zoh", "carrier"])
def test_average_voltage(pwm: ZOH | CarrierComparison) -> None:
    """The average switching state over a sampling period equals the duty ratios."""
    T_s, d_abc = 1e-4, [0.3, 0.55, 0.9]
    for _ in range(2):  # Rising and falling edges of the carrier
        t_steps, sw_states = pwm(T_s, d_abc)
        assert np.sum(t_steps) == pytest.approx(T_s)
        q_avg = np.sum(t_steps * sw_states) / T_s
        assert q_avg == pytest.approx(abc2complex(d_abc), abs=1e-3)
