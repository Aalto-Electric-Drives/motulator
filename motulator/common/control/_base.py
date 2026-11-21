"""Base classes for controls."""

from cmath import exp
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
from types import SimpleNamespace
from typing import Any, Protocol

import numpy as np

from motulator.common.control._pwm import PWM
from motulator.common.utils._utils import get_value, wrap


# %%
class References(Protocol):
    """Protocol defining the interface for reference signals."""

    T_s: float  # Sampling period for next control cycle


class ConverterReferences(References, Protocol):
    """Protocol defining the reference signals of converter control systems."""

    d_abc: Sequence[float]  # Duty ratios


class ConverterFeedbacks(Protocol):
    """Protocol defining the feedback signals used in the modulation."""

    w_c: float  # Angular speed of the coordinate system
    theta_c: float  # Angular position of the coordinate system
    u_dc: float  # DC-bus voltage


@dataclass
class TimeSeries:
    """
    Container for control system's discrete-time data.

    Each signal group saved by `ControlSystem.save` becomes an attribute in
    `ControlSystem.post_process`, e.g., `fbk` and `ref`.

    """

    t: np.ndarray = field(default_factory=lambda: np.array([]))
    # Populated in post_process
    fbk: Any = field(default_factory=dict)
    ref: Any = field(default_factory=dict)

    def __getattr__(self, name: str) -> SimpleNamespace:
        """Declare the other signal groups for type checkers."""
        # Only called if the attribute is not found, i.e., the group was not saved
        raise AttributeError(
            f"{type(self).__name__!r} object has no attribute {name!r}"
        )


def rotate_coordinates(ts: TimeSeries, theta: np.ndarray) -> None:
    """
    Transform the controller time series to rotated coordinates.

    All complex signals of the feedback and reference groups are transformed from the
    controller coordinates to the coordinates rotated by `theta`, and the angle
    `fbk.theta_c` is updated to the angle of the new coordinates. Hence, the stationary
    signals are still obtained as, e.g., `exp(1j*fbk.theta_c)*fbk.i_c`. The angular
    speed `fbk.w_c` is not changed.

    Parameters
    ----------
    ts : TimeSeries
        Controller time series.
    theta : ndarray
        Angle of the new coordinates in the controller coordinates (rad).

    """
    T = np.exp(-1j * theta)
    for group in (ts.fbk, ts.ref):
        for name, x in list(vars(group).items()):
            if np.iscomplexobj(x):
                setattr(group, name, T * x)
    ts.fbk.theta_c = wrap(ts.fbk.theta_c + theta)


class ControlSystem[Mdl, Meas, Ref: References, Fbk]:
    """
    Base class for control systems.

    The type parameters are the types of the model, the measurements, the reference
    signals, and the feedback signals. The control loop, see `run_control_loop`, returns
    the sampling period and the command to the converter, see `get_command`. The signals
    of each step are saved and converted into time series by `post_process`.

    The gates of the converter can be blocked with `set_enable`. While the gates are
    blocked, the control system runs its control loop with `compute_standby_output` in
    place of `compute_output`, so that its observers keep running, and the model holds
    the switches open.

    """

    def __init__(self) -> None:
        self.t: float = 0.0  # Controller time
        self.enabled: bool = True  # Gate enable at the latest sampling instant
        self._enable: bool | Callable[[float], bool] = True
        # Time and signal history
        self._t: list[float] = []
        self._history: dict[str, dict[str, list]] = {}

    def set_enable(self, enable: bool | Callable[[float], bool]) -> None:
        """
        Set the gate enable.

        Parameters
        ----------
        enable : bool | Callable[[float], bool]
            Gate enable, constant or a function of time, evaluated at the sampling
            instants.

        """
        self._enable = enable

    def get_enable(self) -> bool:
        """Evaluate the gate enable at the controller time."""
        return bool(get_value(self._enable, self.t))

    def get_measurement(self, mdl: Mdl) -> Meas:
        """Get measurements from the model."""
        raise NotImplementedError

    def get_feedback(self, meas: Meas) -> Fbk:
        """Get the feedback signals from the measurements."""
        raise NotImplementedError

    def compute_output(self, fbk: Fbk) -> Ref:
        """Compute controller output based on feedback."""
        raise NotImplementedError

    def compute_standby_output(self, fbk: Fbk) -> Ref:
        """Compute controller output while the gates are blocked."""
        raise TypeError(f"{type(self).__name__} does not support gate blocking")

    def update(self, ref: Ref, fbk: Fbk) -> None:
        """Advance the controller time by the sampling period."""
        self.t += ref.T_s

    def save(self, t: float, **signal_groups: Any) -> None:
        """Save a single timestep of data."""
        self._t.append(t)
        for name, signals in signal_groups.items():
            group = self._history.setdefault(name, {})
            for key, value in vars(signals).items():
                group.setdefault(key, []).append(value)

    def run_control_loop(self, mdl: Mdl) -> tuple[float, Any]:
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
        return self.get_command(ref)

    def get_command(self, ref: Ref) -> tuple[float, Any]:
        """
        Extract the sampling period and the converter command from the references.

        The modulator of the converter converts the command into the gate states, see
        `motulator.common.model.ControlledConverter`. The command of a voltage-source
        converter consists of the duty ratios.

        """
        raise NotImplementedError

    def __call__(self, mdl: Mdl) -> tuple[float, Any]:
        """Make the control system callable."""
        return self.run_control_loop(mdl)

    def post_process(self) -> TimeSeries:
        """Convert stored lists to numpy arrays."""
        ts = TimeSeries(t=np.array(self._t))
        for name, signals in self._history.items():
            group = {k: np.array(v) for k, v in signals.items()}
            setattr(ts, name, SimpleNamespace(**group))
        return ts

    def clear_data(self) -> None:
        """Clear all stored data."""
        self._t.clear()
        self._history.clear()


# %%
class InnerController(Protocol):
    """Protocol defining the interface shared by the inner controllers."""

    k_comp: float  # Angle-advance factor, see ConverterControlSystem

    def update(self, ref: Any, fbk: Any) -> None:
        """Update the controller states."""
        ...

    def post_process(self, ts: TimeSeries) -> None:
        """
        Post-process the controller time series.

        If the signals are transformed to other coordinates, `fbk.theta_c` is updated to
        the angle of these coordinates. The angular speed `fbk.w_c` is not updated,
        i.e., it remains that of the controller coordinates, which equals that of the
        new coordinates in steady state.

        """
        ...


class ConverterControlSystem[C: InnerController](ControlSystem):
    """
    Base class for control systems of voltage-source converters.

    The control system runs an inner controller (e.g., current-vector control of a
    machine or grid-forming control), which it updates at every sampling instant and
    whose post-processing it applies to the time series. The observer of the inner
    controller gets the realized converter voltage of the ongoing sampling period, while
    the feedback voltage for the control laws is the realized voltage averaged over the
    previous and the ongoing sampling periods. Before the modulation, the angle of the
    voltage reference is advanced by `k_comp*T_s*w_c`, where `k_comp` is given by the
    inner controller. The value 1.5 compensates for the computational delay of one
    sampling period and the delay of half a sampling period caused by the zero-order
    hold of the PWM [#Bae2003]_. The value 0 means that the angle is not advanced, which
    is used if the controller takes the delays into account itself, e.g., in a direct
    discrete-time design.

    Parameters
    ----------
    inner_ctrl : InnerController
        Inner controller.
    pwm : PWM
        Pulse-width modulator.

    References
    ----------
    .. [#Bae2003] Bae, Sul, "A compensation method for time delay of full-digital
       synchronous frame current regulator of PWM AC drives," IEEE Trans. Ind. Appl.,
       2003, https://doi.org/10.1109/TIA.2003.810660

    """

    def __init__(self, inner_ctrl: C, pwm: PWM) -> None:
        super().__init__()
        self.inner_ctrl = inner_ctrl
        self.pwm = pwm

    def _get_ref(
        self, ref: float | Callable[[float], float] | None, quantity: str
    ) -> float:
        """Evaluate the external reference of `quantity` at the controller time."""
        if ref is None:
            raise ValueError(f"{quantity} reference must be set")
        return get_value(ref, self.t)

    def modulate(
        self, T_s: float, u_ref: complex, fbk: ConverterFeedbacks
    ) -> Sequence[float]:
        """
        Compute the duty ratios from the voltage reference.

        The reference is in the controller coordinates. Its angle is advanced by
        `k_comp*T_s*w_c`. Subclasses can override this method, e.g., to control an
        output filter.

        """
        u_ab_ref = exp(1j * fbk.theta_c) * u_ref
        u_ab_ref *= exp(1j * self.inner_ctrl.k_comp * T_s * fbk.w_c)
        return self.pwm(T_s, u_ab_ref, fbk.u_dc, fbk.w_c)

    def update(self, ref: Any, fbk: Any) -> None:
        """Update the controller states."""
        super().update(ref, fbk)
        self.inner_ctrl.update(ref, fbk)

    def get_command(self, ref: ConverterReferences) -> tuple[float, Sequence[float]]:
        """Extract the sampling period and the duty ratios from the references."""
        return ref.T_s, ref.d_abc

    def post_process(self) -> TimeSeries:
        """Convert the stored signals to time series, post-processed by the inner
        controller."""
        ts = super().post_process()
        if self._t:
            self.inner_ctrl.post_process(ts)
        return ts
