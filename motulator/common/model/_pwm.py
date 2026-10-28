"""
Modulators of the converter models.

A modulator converts the command of the control system into the gate states of the
converter and their durations within the sampling period, including the computational
delay. The command and the gate states are specific to the converter type. For a
voltage-source converter, the command consists of the duty ratios of the legs. The
gate state of a leg is the gate signal `q` of its upper switch and the blanking signal
`b`, which is one when neither of the switches conducts due to the dead time. The
converter then resolves the leg state from the current direction. In the averaged
model, `q` and `b` are the fractions of the half carrier period.

"""

from collections.abc import Callable, Sequence
from typing import Any

import numpy as np

from motulator.common.utils._dead_time import averaged_gate_signals

# %%
SwitchingTimes = np.ndarray
GateStates = np.ndarray


class Delay:
    """
    Computational delay modeled as a ring buffer.

    Parameters
    ----------
    length : int, optional
        Length of the buffer, i.e., the delay in calls, defaults to 1.
    zeros : Callable[[Any], Any], optional
        Function giving the initial content of the buffer from the first input,
        defaults to `np.zeros_like`.

    """

    def __init__(
        self, length: int = 1, zeros: Callable[[Any], Any] = np.zeros_like
    ) -> None:
        self.length = length
        self.zeros = zeros
        self.data: list[Any] = []  # Filled at the first call

    def __call__(self, u: Any) -> Any:
        """
        Delay the input.

        Parameters
        ----------
        u : Any
            Input.

        Returns
        -------
        Any
            Input `length` calls earlier, or zeros before that.

        """
        if not self.data:
            self.data = [self.zeros(u) for _ in range(self.length)]
        # Add the latest value to the end of the list
        self.data.append(u)
        # Pop the first element and return it
        return self.data.pop(0)


# %%
class Modulator[Cmd]:
    """
    Base class for modulators.

    The sampling period consists of one half carrier period with double update and of
    two with single update. The command passes through the computational delay once
    per half carrier period, and the gate states of each half carrier period are
    computed by `schedule`, which the subclasses implement. The type parameter `Cmd`
    is the type of the command, e.g., the duty ratios of a voltage-source converter.

    Parameters
    ----------
    delay : int, optional
        Computational delay in half carrier periods from the sampling instant to the
        loading of the command. Defaults to None, i.e., one sampling period, which the
        control systems of this package compensate for: 1 with double update and 2
        with single update. With single update, `delay=1` loads the command at the
        carrier extremum between the sampling instants.
    single_update : bool, optional
        Sample and update the command once per carrier period, i.e., the sampling
        period of the control system equals the carrier period, defaults to False
        (double update, the sampling period is half the carrier period).

    """

    def __init__(self, delay: int | None = None, single_update: bool = False) -> None:
        if delay is None:
            delay = 2 if single_update else 1
        self.delay = Delay(delay, self.initial_command)
        self.single_update = single_update

    def initial_command(self, command: Cmd) -> Any:
        """
        Command assumed before the first command is loaded.

        The computational delay starts with this command, zeros of the shape of the
        command by default. A subclass whose command is not an array overrides this
        method.

        Parameters
        ----------
        command : Cmd
            First command given to the modulator.

        Returns
        -------
        Any
            Initial command.

        """
        try:
            return np.zeros_like(np.asarray(command, dtype=float))
        except (TypeError, ValueError) as err:
            raise TypeError(
                f"{type(self).__name__} must override initial_command for its command"
            ) from err

    def __call__(self, T_s: float, command: Cmd) -> tuple[SwitchingTimes, GateStates]:
        """
        Delay the command and convert it to gate states.

        Parameters
        ----------
        T_s : float
            Sampling period (s).
        command : Cmd
            Command computed at this sampling instant.

        Returns
        -------
        t_steps : SwitchingTimes, shape (N,)
            Durations of the intervals (s), summing to `T_s`.
        gates : GateStates, shape (N, ...)
            Gate states in each interval.

        """
        n = 2 if self.single_update else 1  # Half carrier periods per sampling period
        parts = [self.schedule(T_s / n, self.delay(command)) for _ in range(n)]
        if n == 1:
            return parts[0]
        t_steps = np.concatenate([p[0] for p in parts])
        gates = np.concatenate([p[1] for p in parts])
        return t_steps, gates

    def schedule(
        self, T_half: float, command: Cmd
    ) -> tuple[SwitchingTimes, GateStates]:
        """
        Convert the command to the gate states of a half carrier period.

        Parameters
        ----------
        T_half : float
            Half carrier period (s).
        command : Cmd
            Command, delayed by the computational delay.

        Returns
        -------
        t_steps : SwitchingTimes, shape (N,)
            Durations of the intervals (s), summing to `T_half`.
        gates : GateStates, shape (N, ...)
            Gate states in each interval.

        """
        raise NotImplementedError


# %%
class TwoLevelModulator[Cmd](Modulator[Cmd]):
    """
    Base class for the modulators of two-level voltage-source converters.

    The gate state of a leg consists of the gate signal of its upper switch and the
    blanking signal, see `VoltageSourceConverter`. Hence, the gate states of the N
    intervals form an array of shape (N, 2, n_legs), the gate signals in the first
    row and the blanking signals in the second row of each interval. In the averaged
    model, they are the on-time fractions of the upper switches and the blanking
    fractions.

    """


# %%
class ZOH(TwoLevelModulator[Sequence[float]]):
    """
    Zero-order hold of the duty ratios of a voltage-source converter.

    The dead time reduces the gate signal duty ratios of both switches of a leg by
    `t_d/(2*T_half)`, leaving the leg blanked for the fraction `t_d/T_half` of the
    half carrier period `T_half`, unless the leg does not switch. Short pulses are
    suppressed, and the blanking fraction is computed from the remaining on-times of
    both switches. This is a switching-cycle averaged model, not a model of switching
    transients.

    Parameters
    ----------
    t_d : float, optional
        Dead time (s), defaults to 0.
    delay : int, optional
        Computational delay in half carrier periods, defaults to one sampling period,
        see `Modulator`.
    single_update : bool, optional
        Sample and update the duty ratios once per carrier period, defaults to False,
        see `Modulator`.

    """

    def __init__(
        self, t_d: float = 0.0, delay: int | None = None, single_update: bool = False
    ) -> None:
        super().__init__(delay, single_update)
        self.t_d = t_d

    def schedule(
        self, T_half: float, command: Sequence[float]
    ) -> tuple[SwitchingTimes, GateStates]:
        """
        Compute the averaged gate states.

        Parameters
        ----------
        T_half : float
            Half carrier period (s).
        command : Sequence[float], shape (n_legs,)
            Duty ratios in the range [0, 1].

        Returns
        -------
        t_steps : SwitchingTimes, shape (1,)
            Half carrier period (s).
        gates : GateStates, shape (1, 2, n_legs)
            On-time fractions of the upper switches and blanking fractions.

        """
        q, b = averaged_gate_signals(command, self.t_d, T_half)
        return np.array([T_half]), np.stack([q, b])[np.newaxis]


# %%
class CarrierComparison(TwoLevelModulator[Sequence[float]]):
    """
    Carrier comparison of a voltage-source converter.

    This computes the gate signals and their durations based on the duty ratios. Instead
    of searching for zero crossings, the switching instants are explicitly computed in
    the beginning of each half carrier period, allowing faster simulations. The dead
    time delays the turn-on of the switches, i.e., each leg is blanked for `t_d` after
    its switching instant.

    Parameters
    ----------
    t_d : float, optional
        Dead time (s), defaults to 0.
    delay : int, optional
        Computational delay in half carrier periods, defaults to one sampling period,
        see `Modulator`.
    single_update : bool, optional
        Sample and update the duty ratios once per carrier period, defaults to False,
        see `Modulator`.
    N : int, optional
        Amount of the counter quantization levels, defaults to 2**12.

    Examples
    --------
    >>> from motulator.common.model import CarrierComparison
    >>> carrier_cmp = CarrierComparison(delay=0)
    >>> # First call gives rising edges
    >>> t_steps, gates = carrier_cmp(1e-3, [.4, .2, .8])
    >>> # Durations of the switching states
    >>> t_steps
    array([0.00019995, 0.00040015, 0.00019995, 0.00019995])
    >>> # Gate signals of the upper switches
    >>> gates[:, 0]
    array([[0, 0, 0],
           [0, 0, 1],
           [1, 0, 1],
           [1, 1, 1]])
    >>> # Second call gives falling edges
    >>> t_steps, gates = carrier_cmp(1e-3, [.4, .2, .8])
    >>> t_steps
    array([0.00019995, 0.00019995, 0.00040015, 0.00019995])
    >>> gates[:, 0]
    array([[1, 1, 1],
           [1, 0, 1],
           [0, 0, 1],
           [0, 0, 0]])
    >>> # Sum of the step times equals T_s
    >>> float(np.sum(t_steps))
    0.001
    >>> # Dead time blanks each leg after its switching instant
    >>> carrier_cmp = CarrierComparison(t_d=1e-4, delay=0)
    >>> t_steps, gates = carrier_cmp(1e-3, [.5, .5, .5])
    >>> np.round(t_steps / 1e-3, 3)  # In ms
    array([0.5, 0.1, 0.4])
    >>> gates[:, 0]  # Gate signals
    array([[0, 0, 0],
           [0, 0, 0],
           [1, 1, 1]])
    >>> gates[:, 1]  # Blanking signals
    array([[0, 0, 0],
           [1, 1, 1],
           [0, 0, 0]])

    """

    def __init__(
        self,
        t_d: float = 0.0,
        delay: int | None = None,
        single_update: bool = False,
        N: int = 2**12,
    ) -> None:
        super().__init__(delay, single_update)
        self.t_d = t_d
        self.N = N
        self._rising_edge = True  # Stores the carrier direction
        self._gate_cmd: np.ndarray | None = None  # Latched gate commands
        self._remaining_dead_time: float | np.ndarray = 0.0

    def schedule(
        self, T_half: float, command: Sequence[float]
    ) -> tuple[SwitchingTimes, GateStates]:
        """
        Compute the gate signals and their durations.

        Parameters
        ----------
        T_half : float
            Half carrier period (s).
        command : Sequence[float], shape (n_legs,)
            Duty ratios in the range [0, 1].

        Returns
        -------
        t_steps : SwitchingTimes, shape (N,)
            Durations of the intervals (s).
        gates : GateStates, shape (N, 2, n_legs)
            Gate signals of the upper switches and blanking signals in each interval.

        Notes
        -----
        Simultaneous switching instants are merged, so the number of intervals varies.
        Blanking continues across half carrier periods, including changes in their
        length. A reversed command cancels a pending turn-on and starts a new blanking
        interval. On the first call, the initial commanded state is assumed to
        already conduct.

        """
        d = np.round(self.N * np.asarray(command)) / self.N
        gate_cmd = (d == 1 if self._rising_edge else d > 0).astype(int)
        t_sw = T_half * (1 - d if self._rising_edge else d)
        t_sw = np.where((d > 0) & (d < 1), t_sw, np.inf)
        old_cmd = gate_cmd if self._gate_cmd is None else self._gate_cmd
        t_on = np.where(gate_cmd != old_cmd, self.t_d, self._remaining_dead_time)

        # A command reversal cancels an earlier pending turn-on.
        events = np.concatenate(([0], t_sw, np.minimum(t_on, t_sw), t_sw + self.t_d))
        t_n = np.unique(events[events < T_half])
        t_steps = np.diff(t_n, append=T_half)
        t = t_n[:, np.newaxis]
        switched = t >= t_sw
        b_abc = (t < np.where(switched, t_sw + self.t_d, t_on)).astype(int)
        q_abc = np.where(switched, 1 - gate_cmd, gate_cmd) * (1 - b_abc)

        self._gate_cmd = np.where(t_sw < T_half, 1 - gate_cmd, gate_cmd)
        self._remaining_dead_time = np.maximum(
            np.where(t_sw < T_half, t_sw + self.t_d, t_on) - T_half, 0
        )
        self._rising_edge = not self._rising_edge

        return t_steps, np.stack([q_abc, b_abc], axis=1)
