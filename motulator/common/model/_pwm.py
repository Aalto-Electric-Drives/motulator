"""
Pulse-width modulation (PWM) implementations.

The PWM models produce the gate signals of the converter legs. The gate signal `q` of a
leg is the on-state of its upper switch. The blanking signal `b` is one when neither of
the switches conducts due to the dead time, and the leg state is then resolved from the
current direction by the converter model. In the averaged model, `q` and `b` are the
fractions of the sampling period.

"""

from typing import Protocol, Sequence

import numpy as np

from motulator.common.utils._dead_time import averaged_gate_signals

# %%
SwitchingTimes = np.ndarray
GateSignals = np.ndarray
BlankingSignals = np.ndarray


class PWM(Protocol):
    """Protocol for PWM implementations."""

    def __call__(
        self, T_s: float, d_abc: Sequence[float]
    ) -> tuple[SwitchingTimes, GateSignals, BlankingSignals]:
        """
        Convert duty ratios to gate signals and their durations.

        Parameters
        ----------
        T_s : float
            Half carrier period (s), corresponds to sampling period.
        d_abc : Sequence[float]
            Duty ratios in range [0, 1].

        Returns
        -------
        t_steps : SwitchingTimes, shape (N,)
            Durations of the intervals.
        q_abc : GateSignals, shape (N, 3)
            Gate signals of the upper switches in each interval.
        b_abc : BlankingSignals, shape (N, 3)
            Blanking signals in each interval.

        """
        ...


# %%
class ZOH(PWM):
    """
    Replace PWM with zero-order hold.

    The dead time reduces the gate signal duty ratios of both switches of a leg by
    `t_d/(2*T_s)`, leaving the leg blanked for the fraction `t_d/T_s` of the sampling
    period, unless the leg does not switch. Short pulses are suppressed, and the
    blanking fraction is computed from the remaining on-times of both switches.
    This is a switching-cycle averaged model, not a model of switching transients.

    Parameters
    ----------
    t_d : float, optional
        Dead time (s), defaults to 0.

    """

    def __init__(self, t_d: float = 0.0) -> None:
        self.t_d = t_d

    def __call__(
        self, T_s: float, d_abc: Sequence[float]
    ) -> tuple[SwitchingTimes, GateSignals, BlankingSignals]:
        q, b = averaged_gate_signals(d_abc, self.t_d, T_s)
        return np.array([T_s]), q[np.newaxis], b[np.newaxis]


# %%
class CarrierComparison(PWM):
    """
    Carrier comparison.

    This computes the gate signals and their durations based on the duty ratios. Instead
    of searching for zero crossings, the switching instants are explicitly computed in
    the beginning of each sampling period, allowing faster simulations. The dead time
    delays the turn-on of the switches, i.e., each leg is blanked for `t_d` after its
    switching instant.

    Parameters
    ----------
    N : int, optional
        Amount of the counter quantization levels, defaults to 2**12.
    t_d : float, optional
        Dead time (s), defaults to 0.

    Examples
    --------
    >>> from motulator.common.model import CarrierComparison
    >>> carrier_cmp = CarrierComparison()
    >>> # First call gives rising edges
    >>> t_steps, q_abc, _ = carrier_cmp(1e-3, [.4, .2, .8])
    >>> # Durations of the switching states
    >>> t_steps
    array([0.00019995, 0.00040015, 0.00019995, 0.00019995])
    >>> # Switching states
    >>> q_abc
    array([[0, 0, 0],
           [0, 0, 1],
           [1, 0, 1],
           [1, 1, 1]])
    >>> # Second call gives falling edges
    >>> t_steps, q_abc, _ = carrier_cmp(1e-3, [.4, .2, .8])
    >>> t_steps
    array([0.00019995, 0.00019995, 0.00040015, 0.00019995])
    >>> q_abc
    array([[1, 1, 1],
           [1, 0, 1],
           [0, 0, 1],
           [0, 0, 0]])
    >>> # Sum of the step times equals T_s
    >>> float(np.sum(t_steps))
    0.001
    >>> # Dead time blanks each leg after its switching instant
    >>> carrier_cmp = CarrierComparison(t_d=1e-4)
    >>> t_steps, q_abc, b_abc = carrier_cmp(1e-3, [.5, .5, .5])
    >>> np.round(t_steps / 1e-3, 3)  # In ms
    array([0.5, 0.1, 0.4])
    >>> q_abc
    array([[0, 0, 0],
           [0, 0, 0],
           [1, 1, 1]])
    >>> b_abc
    array([[0, 0, 0],
           [1, 1, 1],
           [0, 0, 0]])

    """

    def __init__(self, N: int = 2**12, t_d: float = 0.0) -> None:
        self.N = N
        self.t_d = t_d
        self._rising_edge = True  # Stores the carrier direction
        self._command: np.ndarray | None = None
        self._remaining_dead_time: float | np.ndarray = 0.0

    def __call__(
        self, T_s: float, d_abc: Sequence[float]
    ) -> tuple[SwitchingTimes, GateSignals, BlankingSignals]:
        """
        Compute the gate signals and their durations.

        Parameters
        ----------
        T_s : float
            Half carrier period (s).
        d_abc : Sequence[float], shape (3,)
            Duty ratios in the range [0, 1].

        Returns
        -------
        t_steps : SwitchingTimes, shape (N,)
            Durations of the intervals (s).
        q_abc : GateSignals, shape (N, 3)
            Gate signals of the upper switches in each interval.
        b_abc : BlankingSignals, shape (N, 3)
            Blanking signals in each interval.

        Notes
        -----
        Simultaneous switching instants are merged, so the number of intervals varies.
        Blanking continues across sampling periods, including changes in `T_s`.
        A reversed command cancels a pending turn-on and starts a new blanking
        interval. On the first call, the initial commanded state is assumed to
        already conduct.

        """
        d = np.round(self.N * np.asarray(d_abc)) / self.N
        command = (d == 1 if self._rising_edge else d > 0).astype(int)
        t_sw = T_s * (1 - d if self._rising_edge else d)
        t_sw = np.where((d > 0) & (d < 1), t_sw, np.inf)
        old_command = command if self._command is None else self._command
        t_on = np.where(command != old_command, self.t_d, self._remaining_dead_time)

        # A command reversal cancels an earlier pending turn-on.
        events = np.concatenate(([0], t_sw, np.minimum(t_on, t_sw), t_sw + self.t_d))
        t_n = np.unique(events[events < T_s])
        t_steps = np.diff(t_n, append=T_s)
        t = t_n[:, np.newaxis]
        switched = t >= t_sw
        b_abc = (t < np.where(switched, t_sw + self.t_d, t_on)).astype(int)
        q_abc = np.where(switched, 1 - command, command) * (1 - b_abc)

        self._command = np.where(t_sw < T_s, 1 - command, command)
        self._remaining_dead_time = np.maximum(
            np.where(t_sw < T_s, t_sw + self.t_d, t_on) - T_s, 0
        )
        self._rising_edge = not self._rising_edge

        return t_steps, q_abc, b_abc
