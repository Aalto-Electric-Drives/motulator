"""Base control systems for machine drives."""

from dataclasses import dataclass
from math import inf
from typing import Callable, Protocol, Sequence

from motulator.common.control._base import ControlSystem, TimeSeries
from motulator.common.control._controllers import PIController, RateLimiter
from motulator.common.utils._utils import abc2complex, wrap
from motulator.drive.control._controllers import SpeedController
from motulator.drive.model._drive import Drive


# %%
class Feedbacks(Protocol):
    """Protocol defining the required fields for feedback signals."""

    w_M: float


class References(Protocol):
    """Protocol defining the required fields for reference signals."""

    T_s: float
    d_abc: Sequence[float]
    tau_M: float | None
    w_M: float | None


@dataclass
class ExternalReferences:
    """External reference signals."""

    w_M: Callable[[float], float] | None = None
    tau_M: Callable[[float], float] | None = None


@dataclass
class Measurements:
    """Measured signals."""

    i_c_ab: complex  # Converter current (A) in stationary coordinates
    u_dc: float
    w_M: float | None = None
    theta_M: float | None = None


# %%
class VectorController[Ref, Fbk](Protocol):
    """Protocol defining the interface for vector controllers."""

    sensorless: bool

    def get_feedback(self, meas: Measurements) -> Fbk:
        """Get feedback signals from measurements."""
        ...

    def compute_output(self, tau_M_ref: float, fbk: Fbk) -> Ref:
        """Compute control output from feedback signals."""
        ...

    def update(self, ref: Ref, fbk: Fbk) -> None:
        """Update controller states."""
        ...


class VectorControlSystem(ControlSystem):
    """
    Vector control system.

    This class defines the interface for drive control systems. It is a generic class
    that can be used with different inner controllers (such as current-vector control
    and flux-vector control).

    Parameters
    ----------
    vector_ctrl : VectorController
        Vector controller whose input is the torque reference.
    speed_ctrl : SpeedController | PIController | None
        Speed controller. If not given or None, torque-control mode is used.

    """

    def __init__(
        self,
        vector_ctrl: VectorController,
        speed_ctrl: SpeedController | PIController | None = None,
    ) -> None:
        super().__init__()
        self.vector_ctrl = vector_ctrl
        self.speed_ctrl = speed_ctrl
        self.ext_ref = ExternalReferences()

    def set_torque_ref(self, ref_fcn: Callable[[float], float]) -> None:
        """
        Set the external torque reference for torque-control mode.

        Parameters
        ----------
        ref_fcn : Callable[[float], float]
            Torque reference (Nm) as a function of time.

        """
        self.ext_ref.tau_M = ref_fcn

    def set_speed_ref(self, ref_fcn: Callable[[float], float]) -> None:
        """
        Set the external speed reference for speed-control mode.

        Parameters
        ----------
        ref_fcn : Callable[[float], float]
            Speed reference (mechanical rad/s) as a function of time.

        """
        self.ext_ref.w_M = ref_fcn

    def get_measurement(self, mdl: Drive) -> Measurements:
        """Get measurements from sensors."""
        u_dc = mdl.converter.meas_dc_voltage()
        if mdl.lc_filter is not None:
            i_c_ab = abc2complex(mdl.lc_filter.meas_currents())
        else:
            i_c_ab = abc2complex(mdl.machine.meas_currents())

        if self.vector_ctrl.sensorless:
            w_M = None
            theta_M = None
        else:
            w_M = mdl.mechanics.meas_speed()
            theta_M = wrap(mdl.mechanics.meas_position())
        return Measurements(i_c_ab, u_dc, w_M, theta_M)

    def get_feedback(self, meas: Measurements) -> Feedbacks:
        """Get feedback signals."""
        return self.vector_ctrl.get_feedback(meas)

    def compute_output(self, fbk: Feedbacks) -> References:
        """Compute controller output based on feedback."""
        if self.speed_ctrl and self.ext_ref.w_M is not None:  # Speed-control mode
            w_M_ref = self.ext_ref.w_M(self.t)
            tau_M_ref = self.speed_ctrl.compute_output(w_M_ref, fbk.w_M)
            ref = self.vector_ctrl.compute_output(tau_M_ref, fbk)
            ref.w_M = w_M_ref
            return ref
        if self.ext_ref.tau_M is not None:  # Torque-control mode
            tau_M_ref = self.ext_ref.tau_M(self.t)
            ref = self.vector_ctrl.compute_output(tau_M_ref, fbk)
            ref.w_M = None
            return ref
        raise ValueError

    def update(self, ref: References, fbk: Feedbacks) -> None:
        """Update controller states."""
        super().update(ref, fbk)
        self.vector_ctrl.update(ref, fbk)
        if self.speed_ctrl and ref.tau_M is not None:
            self.speed_ctrl.update(ref.T_s, ref.tau_M)

    def post_process(self) -> TimeSeries:
        """Extend the post-process method."""
        ts = super().post_process()
        return ts


# %%
class VHzController[Ref, Fbk](Protocol):
    """Protocol defining the interface for V/Hz controllers."""

    T_s: float

    def get_feedback(self, w_M_ref: float, meas: Measurements) -> Fbk:
        """Get feedback signals from measurements."""
        ...

    def compute_output(self, fbk: Fbk) -> Ref:
        """Compute control output from feedback signals."""
        ...

    def update(self, ref: Ref, fbk: Fbk) -> None:
        """Update controller states."""
        ...


class VHzControlSystem(ControlSystem):
    """
    V/Hz control system.

    Parameters
    ----------
    vhz_ctrl : VHzController
        V/Hz controller to be used in the drive control system.
    slew_rate : float, optional
        Slew rate (mechanical rad/s**2) for the speed reference, defaults to `inf`.

    """

    def __init__(self, vhz_ctrl: VHzController, slew_rate: float = inf) -> None:
        super().__init__()
        self.vhz_ctrl = vhz_ctrl
        self.rate_limiter = RateLimiter(slew_rate)
        self.ext_ref = ExternalReferences()
        self._w_M_ref: float = 0  # For storing ramp-limited speed reference

    def set_speed_ref(self, ref_fcn: Callable[[float], float]) -> None:
        """
        Set the external speed reference.

        Parameters
        ----------
        ref_fcn : Callable[[float], float]
            Speed reference (mechanical rad/s) as a function of time.

        """
        self.ext_ref.w_M = ref_fcn

    def get_measurement(self, mdl: Drive) -> Measurements:
        """Get measurements."""
        u_dc = mdl.converter.meas_dc_voltage()
        i_s_ab = abc2complex(mdl.machine.meas_currents())
        return Measurements(i_s_ab, u_dc)

    def get_feedback(self, meas: Measurements) -> Feedbacks:
        """Get feedback signals."""
        if self.ext_ref.w_M is not None:
            w_M_ref = self.ext_ref.w_M(self.t)
            self._w_M_ref = self.rate_limiter(self.vhz_ctrl.T_s, w_M_ref)
            return self.vhz_ctrl.get_feedback(self._w_M_ref, meas)
        raise ValueError

    def compute_output(self, fbk: Feedbacks) -> References:
        """Compute controller output based on feedback."""
        ref = self.vhz_ctrl.compute_output(fbk)
        ref.w_M = self._w_M_ref
        return ref

    def update(self, ref: References, fbk: Feedbacks) -> None:
        """Update controller states."""
        super().update(ref, fbk)
        self.vhz_ctrl.update(ref, fbk)

    def post_process(self) -> TimeSeries:
        """Extend the post-process method."""
        ts = super().post_process()
        return ts
