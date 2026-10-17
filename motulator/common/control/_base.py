"""Base classes for controls."""

from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import Any, Protocol

import numpy as np

from motulator.common.utils._utils import get_value


# %%
class References(Protocol):
    """Protocol defining the interface for reference signals."""

    T_s: float  # Sampling period for next control cycle
    d_abc: Sequence[float]  # Duty ratios for the next control cycle


@dataclass
class TimeSeries:
    """Container for control system's discrete-time data."""

    t: np.ndarray = field(default_factory=lambda: np.array([]))
    # Populated in post_process
    fbk: Any = field(default_factory=dict)
    ref: Any = field(default_factory=dict)


class ControlSystem[Mdl, Meas, Ref: References, Fbk](Protocol):
    """
    Base class for control systems.

    This class defines the interface for control systems. It is a generic class that can
    be used with different models, measurements, feedback signals, and reference
    signals. The class provides methods for saving, post-processing, and clearing data.

    The gates of the converter can be blocked with `set_enable`. While the gates are
    blocked, the control system runs its control loop with `compute_standby_output`
    in place of `compute_output`, so that its observers keep tracking the system,
    and the model holds the switches open.

    """

    t: float
    enabled: bool  # Gate enable at the latest sampling instant
    _enable: bool | Callable[[float], bool]
    # Time and signal history
    _t: list[float]
    _history: dict[str, dict[str, list]]

    def __init__(self) -> None:
        self.t: float = 0.0  # Controller time
        self.enabled: bool = True
        self._enable: bool | Callable[[float], bool] = True
        # Initialize the data buffer
        self._t: list[float] = []
        self._history: dict[str, dict[str, list]] = {}

    def set_enable(self, enable: bool | Callable[[float], bool]) -> None:
        """
        Set the gate enable.

        Parameters
        ----------
        enable : bool | Callable[[float], bool]
            Gate enable, constant or a function of time, evaluated at the sampling
            instants. While the gates are blocked (False), the control system
            computes its standby output, see `compute_standby_output`.

        """
        self._enable = enable

    def get_enable(self) -> bool:
        """Evaluate the gate enable at the controller time."""
        return bool(get_value(self._enable, self.t))

    def get_measurement(self, mdl: Mdl) -> Meas:
        """Get measurements from the model."""
        ...

    def get_feedback(self, meas: Meas) -> Fbk:
        """Get feedback signals from the model."""
        ...

    def compute_output(self, fbk: Fbk) -> Ref:
        """Compute controller output based on feedback."""
        ...

    def compute_standby_output(self, fbk: Fbk) -> Ref:
        """Compute controller output while the gates are blocked."""
        raise NotImplementedError(
            f"{type(self).__name__} does not support gate blocking"
        )

    def update(self, ref: Ref, fbk: Fbk) -> None:
        """Update controller internal states."""
        self.t = (self.t + ref.T_s) % 1e9  # Avoid overflow

    def save(self, t: float, **signal_groups: Any) -> None:
        """Save a single timestep of data."""
        self._t.append(t)
        # Save all signals from each group
        for group_name, signals in signal_groups.items():
            if group_name not in self._history:
                self._history[group_name] = {}
            for key, value in vars(signals).items():
                self._history[group_name].setdefault(key, []).append(value)

    def run_control_loop(self, mdl: Mdl) -> tuple[float, Sequence[float]]:
        """Run the default control loop, can be overridden."""
        self.enabled = self.get_enable()
        meas = self.get_measurement(mdl)
        fbk = self.get_feedback(meas)
        if self.enabled:
            ref = self.compute_output(fbk)
        else:
            ref = self.compute_standby_output(fbk)
        self.save(self.t, ref=ref, fbk=fbk)
        self.update(ref, fbk)
        return self.get_duty_ratios(ref)

    def get_duty_ratios(self, ref: References) -> tuple[float, Sequence[float]]:
        """Extract duty ratios from the reference signals."""
        return ref.T_s, ref.d_abc

    def __call__(self, mdl: Mdl) -> tuple[float, Sequence[float]]:
        """Make the control system callable."""
        return self.run_control_loop(mdl)

    def post_process(self) -> TimeSeries:
        """Convert stored lists to numpy arrays."""
        ts = TimeSeries()
        ts.t = np.array(self._t)
        # Convert each signal group to a namespace
        for group_name, signals in self._history.items():
            group_data = {k: np.array(v) for k, v in signals.items()}
            setattr(ts, group_name, SimpleNamespace(**group_data))
        return ts

    def clear_data(self) -> None:
        """Clear all stored data."""
        self._t.clear()
        self._history.clear()
