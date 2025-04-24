"""Pulse-width modulation (PWM) implementations."""

from typing import Protocol, Sequence

import numpy as np

from motulator.common.utils import abc2complex

# %%
SwitchingTimes = np.ndarray
ComplexSwitchingStates = np.ndarray
PhaseSwitchingStates = Sequence[Sequence[int]]
SwitchingStates = ComplexSwitchingStates | PhaseSwitchingStates


class PWM(Protocol):
    """Protocol for PWM implementations."""

    def __call__(
        self, T_s: float, d_abc: Sequence[float]
    ) -> tuple[SwitchingTimes, SwitchingStates]:
        """
        Convert duty ratios to switching states and their durations.

        Parameters
        ----------
        T_s : float
            Half carrier period (s), corresponds to sampling period.
        d_abc : Sequence[float], shape (3,)
            Duty ratios in range [0, 1].

        Returns
        -------
        t_steps : SwitchingTimes
            Switching state durations.
        SwitchingStates
            Switching states in the complex space vector form.

        """
        ...


# %%
class ZOH(PWM):
    """Replace PWM with zero-order hold."""

    def __call__(
        self, T_s: float, d_abc: Sequence[float]
    ) -> tuple[SwitchingTimes, ComplexSwitchingStates]:
        # Shape the output arrays to be compatible with the solver
        t_steps = np.array([T_s])
        return t_steps, np.array([abc2complex(d_abc)])


# %%
class CarrierComparison(PWM):
    """
    Carrier comparison.

    This computes the the switching states and their durations based on the duty ratios.
    Instead of searching for zero crossings, the switching instants are explicitly
    computed in the beginning of each sampling period, allowing faster simulations.

    Parameters
    ----------
    N : int, optional
        Amount of the counter quantization levels, defaults to 2**12.
    return_complex : bool, optional
        Complex switching state space vectors are returned if True. Otherwise phase
        switching states are returned, defaults to True.

    Examples
    --------
    >>> from motulator.common.model import CarrierComparison
    >>> carrier_cmp = CarrierComparison(return_complex=False)
    >>> # First call gives rising edges
    >>> t_steps, q_abc = carrier_cmp(1e-3, [.4, .2, .8])
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
    >>> t_steps, q_abc = carrier_cmp(.001, [.4, .2, .8])
    >>> t_steps
    array([0.00019995, 0.00019995, 0.00040015, 0.00019995])
    >>> q_abc
    array([[1, 1, 1],
           [1, 0, 1],
           [0, 0, 1],
           [0, 0, 0]])
    >>> # Sum of the step times equals T_s
    >>> np.sum(t_steps)
    0.001
    >>> # 50% duty ratios in all phases
    >>> t_steps, q_abc = carrier_cmp(1e-3, [.5, .5, .5])
    >>> t_steps
    array([0.0005, 0.    , 0.    , 0.0005])
    >>> q_abc
    array([[0, 0, 0],
           [0, 0, 0],
           [0, 0, 0],
           [1, 1, 1]])

    """

    def __init__(self, N: int = 2**12, return_complex: bool = True) -> None:
        self.N = N
        self.return_complex = return_complex
        self._rising_edge = True  # Stores the carrier direction

    def __call__(
        self, T_s: float, d_abc: Sequence[float]
    ) -> tuple[SwitchingTimes, SwitchingStates]:
        """
        Compute switching state durations and vectors.

        Parameters
        ----------
        T_s : float
            Half carrier period (s).
        d_abc : Sequence[float], shape (3,)
            Duty ratios in the range [0, 1].

        Returns
        -------
        t_steps : SwitchingTimes
            Switching state durations (s), `[t0, t1, t2, t3]`.
        SwitchingStates
            Switching state vectors, `[q0, q1, q2, q3]`, where `q1` and `q2` are active
            vectors.

        Notes
        -----
        No switching (e.g. `d_a == 0` or `d_a == 1`) or simultaneous switching (e.g.
        `d_a == d_b`) lead to zeroes in `t_steps`.

        """
        # Quantize the duty ratios to N levels
        d_abc_arr = np.round(self.N * np.array(d_abc)) / self.N

        # Assume falling edge and compute normalized switching instants
        t_n = np.append(0, np.sort(d_abc_arr))
        # Compute the corresponding switching states:
        q_abc = (t_n[:, np.newaxis] < d_abc_arr).astype(int)

        # Durations of switching states
        t_steps = T_s * np.diff(t_n, append=1)

        # Flip the sequence if rising edge
        if self._rising_edge:
            t_steps = np.flip(t_steps)
            q_abc = np.flipud(q_abc)

        # Change the carrier direction for the next call
        self._rising_edge = not self._rising_edge

        return (
            (t_steps, abc2complex(q_abc.T)) if self.return_complex else (t_steps, q_abc)
        )
