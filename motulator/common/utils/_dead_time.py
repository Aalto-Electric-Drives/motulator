"""Switching-cycle averaged model of the dead time."""

from collections.abc import Sequence

import numpy as np


def averaged_gate_signals(
    d_abc: Sequence[float] | np.ndarray, t_d: float, T_half: float
) -> tuple[np.ndarray, np.ndarray]:
    """
    Compute the on-time fraction of the upper switch and the blanking fraction.

    The dead time `t_d` reduces the on-time fractions of both switches of a leg by
    `t_d/(2*T_half)` per half carrier period `T_half`, unless the leg does not switch.
    Short pulses are suppressed, and the blanking fraction is the remaining fraction
    in which neither switch conducts.

    """
    d = np.asarray(d_abc, dtype=float)
    delta = np.where((d > 0) & (d < 1), t_d / (2 * T_half), 0.0)
    q_hi = np.maximum(d - delta, 0)
    q_lo = np.maximum(1 - d - delta, 0)
    return q_hi, 1 - q_hi - q_lo
