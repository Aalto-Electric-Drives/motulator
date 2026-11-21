"""Base control systems for machine drives."""

from cmath import exp
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from math import inf
from typing import Any, Protocol

from motulator.common.control._base import (
    ConverterControlSystem,
    ConverterFeedbacks,
    InnerController,
)
from motulator.common.control._controllers import PIController, RateLimiter
from motulator.common.control._pwm import PWM
from motulator.common.utils._utils import abc2complex, get_value, wrap
from motulator.drive.control._common import References, SpeedController
from motulator.drive.model._drive import Drive


# %%
class Feedbacks(ConverterFeedbacks, Protocol):
    """Protocol defining the required fields for feedback signals."""

    w_M: float  # Mechanical angular speed
    u_s: complex  # Stator voltage (controller coordinates) for the control laws
    u_s_zoh: complex  # Stator voltage (controller coordinates) for the observers


@dataclass
class ExternalReferences:
    """External reference signals."""

    w_M: float | Callable[[float], float] | None = None
    tau_M: float | Callable[[float], float] | None = None


@dataclass
class Measurements:
    """Measured signals."""

    i_c_ab: complex  # Converter current in stationary coordinates
    u_dc: float
    w_M: float | None = None
    theta_M: float | None = None  # Mechanical angular position


# %%
class VectorController(InnerController, Protocol):
    """
    Protocol defining the interface for vector controllers.

    The attribute `sensorless` is True if the rotor speed and angle are estimated
    instead of measured.

    """

    sensorless: bool

    def get_feedback(
        self,
        u_s_ab: complex | None,
        i_s_ab: complex,
        w_M_meas: float | None,
        theta_M_meas: float | None,
    ) -> Any:
        """Get feedback signals from measurements, `u_s_ab` None if unknown."""
        ...

    def compute_output(self, tau_M_ref: float, fbk: Any) -> Any:
        """Compute control output from feedback signals."""
        ...


class VHzController(InnerController, Protocol):
    """
    Protocol defining the interface for V/Hz controllers.

    The attribute `T_s` is the sampling period (s), which the rate limiter of the speed
    reference uses.

    """

    T_s: float

    def get_feedback(
        self, u_s_ab: complex | None, i_s_ab: complex, w_M_ref: float
    ) -> Any:
        """Get feedback signals from measurements, `u_s_ab` None if unknown."""
        ...

    def compute_output(self, fbk: Any) -> Any:
        """Compute control output from feedback signals."""
        ...

    def compute_standby_output(self, fbk: Any) -> Any:
        """Compute control output while the gates are blocked."""
        ...


# %%
class DriveControlSystem[C: InnerController](ConverterControlSystem[C]):
    """
    Base class for drive control systems.

    This class extends `ConverterControlSystem` with the parts common to the drive
    control systems: the measurements, the feedback signals, the speed reference, and
    the standby. Subclasses implement the hooks `_get_inner_feedback`,
    `_compute_inner_output`, and `_compute_inner_standby_output`.

    While the gates are blocked (see `set_enable`), the control system is in standby.
    Since the converter voltage is unknown, the inner controller gets no voltage, and
    its observer, if any, runs on its open-circuit model.

    Parameters
    ----------
    inner_ctrl : InnerController
        Inner controller.
    pwm : PWM
        Pulse-width modulator.

    """

    def __init__(self, inner_ctrl: C, pwm: PWM) -> None:
        super().__init__(inner_ctrl, pwm)
        self.ext_ref = ExternalReferences()

    def set_speed_ref(self, ref_fcn: float | Callable[[float], float]) -> None:
        """
        Set the external speed reference.

        Parameters
        ----------
        ref_fcn : float | Callable[[float], float]
            Speed reference (mechanical rad/s), constant or a function of time.

        """
        self.ext_ref.w_M = ref_fcn

    def get_measurement(self, mdl: Drive) -> Measurements:
        """Sample the phase-current and DC-bus voltage sensors."""
        i_c_ab = abc2complex(mdl.sensors.i_c.sample())
        return Measurements(i_c_ab, mdl.sensors.u_dc.sample())

    def _get_inner_feedback(
        self, u_s_ab: complex | None, meas: Measurements
    ) -> Feedbacks:
        """Get the feedback signals from the inner controller."""
        raise NotImplementedError

    def _compute_inner_output(self, fbk: Feedbacks) -> References:
        """Compute the references, including the speed reference `w_M`."""
        raise NotImplementedError

    def _compute_inner_standby_output(self, fbk: Feedbacks) -> References:
        """Compute the references while the gates are blocked."""
        raise NotImplementedError

    def _standby(self) -> bool:
        """Standby of the control system, while the gates are blocked."""
        return not self.enabled

    def get_feedback(self, meas: Measurements) -> Feedbacks:
        """Get feedback signals."""
        u_c_ab_zoh, u_c_ab = self.pwm.get_realized_voltages(meas.i_c_ab, meas.u_dc)
        if self._standby():
            # The voltage is unknown with blocked gates, so the observers run on their
            # open-circuit model, whose voltage is used for the control laws too
            fbk = self._get_inner_feedback(None, meas)
            fbk.u_s = fbk.u_s_zoh
        else:
            fbk = self._get_inner_feedback(u_c_ab_zoh, meas)
            fbk.u_s = exp(-1j * fbk.theta_c) * u_c_ab
        fbk.u_dc = meas.u_dc
        return fbk

    def compute_output(self, fbk: Feedbacks) -> References:
        """Compute controller output based on feedback."""
        return self._add_duty_ratios(self._compute_inner_output(fbk), fbk)

    def compute_standby_output(self, fbk: Feedbacks) -> References:
        """Compute controller output while the gates are blocked."""
        return self._add_duty_ratios(self._compute_inner_standby_output(fbk), fbk)

    def _add_duty_ratios(self, ref: References, fbk: Feedbacks) -> References:
        """Compute the duty ratios and the limited voltage reference."""
        ref.d_abc = self.modulate(ref.T_s, ref.u_s, fbk)
        # The limited voltage in the controller coordinates, without the angle advance
        advance = exp(1j * self.inner_ctrl.k_comp * ref.T_s * fbk.w_c)
        ref.u_s_lim = (
            exp(-1j * fbk.theta_c) * advance.conjugate() * self.pwm.limited_voltage
        )
        ref.enabled = self.enabled
        return ref


# %%
class VectorControlSystem(DriveControlSystem[VectorController]):
    """
    Vector control system.

    The inner controller (e.g., current-vector or flux-vector control) gets the torque
    reference, either directly or from the speed controller.

    In standby, the inner controller computes its output for zero torque, and the speed
    controller is not updated. A sensorless drive stays in standby with the zero voltage
    vector also at the first sampling instant after the release of the gates. Due to the
    computational delay, the first two sampling periods are then zero-voltage periods,
    and the observer of a synchronous machine catches the rotating machine from the
    current response to the first one (a flying start).

    Parameters
    ----------
    inner_ctrl : VectorController
        Vector controller whose input is the torque reference.
    speed_ctrl : SpeedController | PIController | None, optional
        Speed controller. If None (default), torque-control mode is used.
    pwm : PWM, optional
        Pulse-width modulator, defaults to `PWM()`.

    """

    def __init__(
        self,
        inner_ctrl: VectorController,
        speed_ctrl: SpeedController | PIController | None = None,
        pwm: PWM | None = None,
    ) -> None:
        super().__init__(inner_ctrl, pwm if pwm is not None else PWM())
        self.speed_ctrl = speed_ctrl
        self._release_step: bool = False  # First sampling instant after the release

    def set_speed_ref(self, ref_fcn: float | Callable[[float], float]) -> None:
        """
        Set the external speed reference for speed-control mode.

        Parameters
        ----------
        ref_fcn : float | Callable[[float], float]
            Speed reference (mechanical rad/s), constant or a function of time.

        """
        if self.speed_ctrl is None:
            raise ValueError("Speed-control mode requires speed_ctrl")
        super().set_speed_ref(ref_fcn)

    def set_torque_ref(self, ref_fcn: float | Callable[[float], float]) -> None:
        """
        Set the external torque reference for torque-control mode.

        Parameters
        ----------
        ref_fcn : float | Callable[[float], float]
            Torque reference (Nm), constant or a function of time.

        """
        if self.speed_ctrl is not None:
            raise ValueError("The speed controller gives the torque reference")
        self.ext_ref.tau_M = ref_fcn

    def get_measurement(self, mdl: Drive) -> Measurements:
        """Sample the sensors, the speed and position sensors if the drive has them."""
        meas = super().get_measurement(mdl)
        sensors = mdl.sensors
        if sensors.w_M is not None:
            meas.w_M = sensors.w_M.sample()
        if sensors.theta_M is not None:
            meas.theta_M = wrap(sensors.theta_M.sample())
        return meas

    def _standby(self) -> bool:
        return not self.enabled or self._release_step

    def _get_inner_feedback(
        self, u_s_ab: complex | None, meas: Measurements
    ) -> Feedbacks:
        return self.inner_ctrl.get_feedback(u_s_ab, meas.i_c_ab, meas.w_M, meas.theta_M)

    def _compute_inner_standby_output(self, fbk: Feedbacks) -> References:
        # Zero torque reference: the inner controller computes the voltage of the
        # zero-torque operating point (with field weakening if needed), from which the
        # control continues when the gates are released
        ref = self.inner_ctrl.compute_output(0.0, fbk)
        if self.speed_ctrl is not None and self.ext_ref.w_M is not None:
            ref.w_M = get_value(self.ext_ref.w_M, self.t)
        return ref

    def _compute_inner_output(self, fbk: Feedbacks) -> References:
        if self.speed_ctrl is not None:  # Speed-control mode
            w_M_ref = self._get_ref(self.ext_ref.w_M, "Speed")
            tau_M_ref = self.speed_ctrl.compute_output(w_M_ref, fbk.w_M)
        else:  # Torque-control mode
            w_M_ref = None
            tau_M_ref = self._get_ref(self.ext_ref.tau_M, "Torque")
        ref = self.inner_ctrl.compute_output(tau_M_ref, fbk)
        ref.w_M = w_M_ref
        return ref

    def compute_output(self, fbk: Feedbacks) -> References:
        """Compute controller output based on feedback."""
        if self._standby():  # The first sampling instant after the release
            return self.compute_standby_output(fbk)
        return super().compute_output(fbk)

    def modulate(
        self, T_s: float, u_ref: complex, fbk: ConverterFeedbacks
    ) -> Sequence[float]:
        """Compute the duty ratios, the zero vector in standby if sensorless."""
        if self.inner_ctrl.sensorless and self._standby():
            u_ref = 0j
        return super().modulate(T_s, u_ref, fbk)

    def update(self, ref: References, fbk: Feedbacks) -> None:
        """Update controller states."""
        super().update(ref, fbk)
        if self.speed_ctrl is not None and not self._standby():
            self.speed_ctrl.update(ref.T_s, ref.tau_M)
        self._release_step = self.inner_ctrl.sensorless and not self.enabled


# %%
class VHzControlSystem(DriveControlSystem[VHzController]):
    """
    V/Hz control system.

    Parameters
    ----------
    inner_ctrl : VHzController
        V/Hz controller.
    slew_rate : float, optional
        Slew rate (mechanical rad/s**2) for the speed reference, defaults to `inf`.
    pwm : PWM, optional
        Pulse-width modulator, defaults to `PWM(overmodulation="MME")`.

    """

    def __init__(
        self, inner_ctrl: VHzController, slew_rate: float = inf, pwm: PWM | None = None
    ) -> None:
        super().__init__(
            inner_ctrl, pwm if pwm is not None else PWM(overmodulation="MME")
        )
        self.rate_limiter = RateLimiter(slew_rate)
        self._w_M_ref: float = 0  # For storing ramp-limited speed reference

    def _get_inner_feedback(
        self, u_s_ab: complex | None, meas: Measurements
    ) -> Feedbacks:
        if self.ext_ref.w_M is None:
            if self.enabled:
                raise ValueError("Speed reference must be set")
            w_M_ref = 0.0  # No speed reference is needed while the gates are blocked
        else:
            w_M_ref = get_value(self.ext_ref.w_M, self.t)
        self._w_M_ref = self.rate_limiter(self.inner_ctrl.T_s, w_M_ref)
        return self.inner_ctrl.get_feedback(u_s_ab, meas.i_c_ab, self._w_M_ref)

    def _compute_inner_output(self, fbk: Feedbacks) -> References:
        ref = self.inner_ctrl.compute_output(fbk)
        ref.w_M = self._w_M_ref
        return ref

    def _compute_inner_standby_output(self, fbk: Feedbacks) -> References:
        ref = self.inner_ctrl.compute_standby_output(fbk)
        ref.w_M = self._w_M_ref
        return ref
