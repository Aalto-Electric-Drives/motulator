"""Switching-cycle averaged model of the dead time."""

from typing import Callable, Sequence

import numpy as np


def averaged_gate_signals(
    d_abc: Sequence[float] | np.ndarray, t_d: float, T_s: float
) -> tuple[np.ndarray, np.ndarray]:
    """Compute the on-time fraction of the upper switch and the blanking fraction."""
    d = np.asarray(d_abc, dtype=float)
    delta = np.where((d > 0) & (d < 1), t_d / (2 * T_s), 0.0)
    q_hi = np.maximum(d - delta, 0)
    q_lo = np.maximum(1 - d - delta, 0)
    return q_hi, 1 - q_hi - q_lo


def dead_time_error(
    i_abc: np.ndarray,
    d_abc: np.ndarray,
    t_d: float,
    T_s: float,
    sign: Callable[[np.ndarray], np.ndarray] = np.sign,
) -> np.ndarray:
    """
    Compute the switching-cycle averaged duty-ratio error due to dead time.

    The realized duty ratios are `d_abc - dead_time_error(...)`. Away from the duty
    limits, the error is `t_d/(2*T_s)*sign(i_abc)`. Short pulses are suppressed if
    their duration is less than the dead time. Clamped legs have no error.

    Parameters
    ----------
    i_abc : ndarray, shape (3,)
        Phase currents (A).
    d_abc : ndarray, shape (3,)
        Duty ratios in the range [0, 1].
    t_d : float
        Dead time (s).
    T_s : float
        Sampling period (s), equal to the half carrier period. In the control system,
        this must equal the sampling period of the controller.
    sign : Callable[[np.ndarray], np.ndarray], optional
        Current-direction function, defaults to `np.sign`. A smooth approximation
        may be used, matching the converter model.

    Returns
    -------
    ndarray, shape (3,)
        Duty-ratio errors, including the effect of short pulses.

    Notes
    -----
    This averaged model assumes constant duty ratios and currents over a carrier
    period. It does not include transients due to changes in the switching commands.

    """
    q_abc, b_abc = averaged_gate_signals(d_abc, t_d, T_s)
    return d_abc - q_abc - 0.5 * b_abc * (1 - sign(i_abc))
