"""Base classes for controls."""

from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import Any, Protocol, Sequence

import numpy as np


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

    """

    t: float
    # Time and signal history
    _t: list[float]
    _history: dict[str, dict[str, list]]

    def __init__(self) -> None:
        self.t: float = 0.0  # Controller time
        # Initialize the data buffer
        self._t: list[float] = []
        self._history: dict[str, dict[str, list]] = {}

    def get_measurement(self, mdl: Mdl) -> Meas:
        """Get measurements from the model."""
        ...

    def get_feedback(self, meas: Meas) -> Fbk:
        """Get feedback signals from the model."""
        ...

    def compute_output(self, fbk: Fbk) -> Ref:
        """Compute controller output based on feedback."""
        ...

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
        meas = self.get_measurement(mdl)
        fbk = self.get_feedback(meas)
        ref = self.compute_output(fbk)
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
