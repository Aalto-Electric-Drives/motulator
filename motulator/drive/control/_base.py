"""Base control systems for machine drives."""

from cmath import exp
from collections.abc import Callable, Sequence
from dataclasses import dataclass
from math import inf
from typing import Protocol

from motulator.common.control._base import ControlSystem, TimeSeries
from motulator.common.control._controllers import PIController, RateLimiter
from motulator.common.control._pwm import PWM
from motulator.common.utils._utils import abc2complex, get_value, line2complex, wrap
from motulator.drive.control._common import References, SpeedController
from motulator.drive.model._drive import Drive


# %%
class Feedbacks(Protocol):
    """Protocol defining the required fields for feedback signals."""

    w_M: float  # Mechanical angular speed
    w_c: float  # Angular speed of the coordinate system
    theta_c: float  # Angular position of the coordinate system
    u_s: complex  # Stator voltage (controller coordinates) for the control laws
    u_dc: float  # DC-bus voltage


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
    u_c_ab: complex | None = None  # Converter voltage, measured with blocked gates


# %%
class VectorController[Ref, Fbk](Protocol):
    """Protocol defining the interface for vector controllers."""

    sensorless: bool
    k_comp: float  # Angle-advance factor, see DriveControlSystem

    def get_feedback(
        self,
        u_s_ab: complex,
        i_s_ab: complex,
        w_M_meas: float | None,
        theta_M_meas: float | None,
    ) -> Fbk:
        """Get feedback signals from measurements."""
        ...

    def compute_output(self, tau_M_ref: float, fbk: Fbk) -> Ref:
        """Compute control output from feedback signals."""
        ...

    def compute_standby_output(self, fbk: Fbk) -> Ref:
        """Compute control output while the gates are blocked."""
        ...

    def update(self, ref: Ref, fbk: Fbk) -> None:
        """Update controller states."""
        ...

    def post_process(self, ts: TimeSeries) -> None:
        """Post-process controller outputs."""
        ...


class VHzController[Ref, Fbk](Protocol):
    """Protocol defining the interface for V/Hz controllers."""

    T_s: float
    k_comp: float  # Angle-advance factor, see DriveControlSystem

    def get_feedback(self, u_s_ab: complex, i_s_ab: complex, w_M_ref: float) -> Fbk:
        """Get feedback signals from measurements."""
        ...

    def compute_output(self, fbk: Fbk) -> Ref:
        """Compute control output from feedback signals."""
        ...

    def compute_standby_output(self, fbk: Fbk) -> Ref:
        """Compute control output while the gates are blocked."""
        ...

    def update(self, ref: Ref, fbk: Fbk) -> None:
        """Update controller states."""
        ...

    def post_process(self, ts: TimeSeries) -> None:
        """Post-process controller outputs."""
        ...


# %%
class DriveControlSystem(ControlSystem):
    """
    Base class for drive control systems.

    This class implements the parts common to the drive control systems: the feedback
    voltages, the modulation, and the speed reference. Subclasses implement
    `get_measurement`, `update`, `post_process`, and the hooks `_get_inner_feedback`,
    `_compute_inner_output`, and `_compute_inner_standby_output`.

    The inner controller gets the realized voltage of the ongoing sampling period for
    its observer, while the feedback voltage `u_s` for the control laws is the realized
    voltage averaged over the previous and the ongoing sampling periods. Before the
    modulation, the angle of the voltage reference is advanced by `k_comp*T_s*w_c`,
    where `k_comp` is given by the inner controller. The value 1.5 compensates for the
    computational delay of one sampling period and the delay of half a sampling period
    caused by the zero-order hold of the PWM [#Bae2003]_. The value 0 means that the
    angle is not advanced, which is used if the controller takes the delays into
    account itself, e.g., in a direct discrete-time design.

    While the gates are blocked (see `set_enable`), the feedback voltages are the
    measured converter voltage, so the observers keep tracking the machine, and the
    inner controller computes its standby output: its regulators track the measured
    operating point, from which the control continues when the gates are released,
    while the speed controller is not updated.

    Parameters
    ----------
    pwm : PWM
        Pulse-width modulator.
    k_comp : float
        Angle-advance factor of the voltage reference.

    References
    ----------
    .. [#Bae2003] Bae, Sul, "A compensation method for time delay of full-digital
       synchronous frame current regulator of PWM AC drives," IEEE Trans. Ind. Appl.,
       2003, https://doi.org/10.1109/TIA.2003.810660

    """

    def __init__(self, pwm: PWM, k_comp: float) -> None:
        super().__init__()
        self.pwm = pwm
        self.k_comp = k_comp
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

    def _get_inner_feedback(self, u_s_ab: complex, meas: Measurements) -> Feedbacks:
        """Get the feedback signals from the inner controller."""
        raise NotImplementedError

    def _compute_inner_output(self, fbk: Feedbacks) -> References:
        """Compute the references, including the speed reference `w_M`."""
        raise NotImplementedError

    def _compute_inner_standby_output(self, fbk: Feedbacks) -> References:
        """Compute the references while the gates are blocked."""
        raise NotImplementedError

    def _measure_converter_voltage(self, mdl: Drive) -> complex | None:
        """Measure the converter voltage while the gates are blocked."""
        if self.enabled:
            return None
        return line2complex(mdl.converter.meas_voltages())

    def get_feedback(self, meas: Measurements) -> Feedbacks:
        """Get feedback signals."""
        # The observers use the voltage of the ongoing sampling period, while the
        # control laws use the voltage averaged over the previous and ongoing periods.
        # With blocked gates, both are the measured voltage.
        if meas.u_c_ab is None:
            u_c_ab_zoh = self.pwm.get_realized_voltage(
                meas.i_c_ab, meas.u_dc, average=False
            )
            u_c_ab = self.pwm.get_realized_voltage(meas.i_c_ab, meas.u_dc)
        else:
            u_c_ab_zoh = u_c_ab = meas.u_c_ab
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
        ref.u_s_lim = exp(-1j * fbk.theta_c) * self.pwm.limited_voltage
        ref.enabled = self.enabled
        return ref

    def modulate(self, T_s: float, u_s_ref: complex, fbk: Feedbacks) -> Sequence[float]:
        """
        Compute the duty ratios from the stator voltage reference.

        The reference is in the controller coordinates. Its angle is advanced by
        `k_comp*T_s*w_c`. Subclasses can override this method, e.g., to control an
        output filter between the converter and the machine.

        """
        u_s_ab_ref = exp(1j * fbk.theta_c) * u_s_ref
        u_s_ab_ref *= exp(1j * self.k_comp * T_s * fbk.w_c)
        return self.pwm(T_s, u_s_ab_ref, fbk.u_dc, fbk.w_c)


# %%
class VectorControlSystem(DriveControlSystem):
    """
    Vector control system.

    This class defines the interface for drive control systems. It is a generic class
    that can be used with different inner controllers (such as current-vector control
    and flux-vector control).

    Parameters
    ----------
    vector_ctrl : VectorController
        Vector controller whose input is the torque reference.
    speed_ctrl : SpeedController | PIController | None, optional
        Speed controller. If None (default), torque-control mode is used.
    pwm : PWM, optional
        Pulse-width modulator, defaults to `PWM()`.

    """

    def __init__(
        self,
        vector_ctrl: VectorController,
        speed_ctrl: SpeedController | PIController | None = None,
        pwm: PWM | None = None,
    ) -> None:
        super().__init__(pwm if pwm is not None else PWM(), vector_ctrl.k_comp)
        self.vector_ctrl = vector_ctrl
        self.speed_ctrl = speed_ctrl

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
        u_c_ab = self._measure_converter_voltage(mdl)
        return Measurements(i_c_ab, u_dc, w_M, theta_M, u_c_ab)

    def _get_inner_feedback(self, u_s_ab: complex, meas: Measurements) -> Feedbacks:
        return self.vector_ctrl.get_feedback(
            u_s_ab, meas.i_c_ab, meas.w_M, meas.theta_M
        )

    def _compute_inner_standby_output(self, fbk: Feedbacks) -> References:
        ref = self.vector_ctrl.compute_standby_output(fbk)
        if self.speed_ctrl is not None and self.ext_ref.w_M is not None:
            ref.w_M = get_value(self.ext_ref.w_M, self.t)
        return ref

    def _compute_inner_output(self, fbk: Feedbacks) -> References:
        if self.speed_ctrl is not None:  # Speed-control mode
            if self.ext_ref.w_M is None:
                raise ValueError("Speed reference must be set")
            w_M_ref = get_value(self.ext_ref.w_M, self.t)
            tau_M_ref = self.speed_ctrl.compute_output(w_M_ref, fbk.w_M)
        else:  # Torque-control mode
            if self.ext_ref.tau_M is None:
                raise ValueError("Torque reference must be set")
            w_M_ref = None
            tau_M_ref = get_value(self.ext_ref.tau_M, self.t)
        ref = self.vector_ctrl.compute_output(tau_M_ref, fbk)
        ref.w_M = w_M_ref
        return ref

    def update(self, ref: References, fbk: Feedbacks) -> None:
        """Update controller states."""
        super().update(ref, fbk)
        self.vector_ctrl.update(ref, fbk)
        if self.speed_ctrl is not None and self.enabled:
            self.speed_ctrl.update(ref.T_s, ref.tau_M)

    def post_process(self) -> TimeSeries:
        """Extend the post-process method."""
        ts = super().post_process()
        self.vector_ctrl.post_process(ts)
        return ts


# %%
class VHzControlSystem(DriveControlSystem):
    """
    V/Hz control system.

    Parameters
    ----------
    vhz_ctrl : VHzController
        V/Hz controller to be used in the drive control system.
    slew_rate : float, optional
        Slew rate (mechanical rad/s**2) for the speed reference, defaults to `inf`.
    pwm : PWM, optional
        Pulse-width modulator, defaults to `PWM(overmodulation="MME")`.

    """

    def __init__(
        self, vhz_ctrl: VHzController, slew_rate: float = inf, pwm: PWM | None = None
    ) -> None:
        pwm = pwm if pwm is not None else PWM(overmodulation="MME")
        super().__init__(pwm, vhz_ctrl.k_comp)
        self.vhz_ctrl = vhz_ctrl
        self.rate_limiter = RateLimiter(slew_rate)
        self._w_M_ref: float = 0  # For storing ramp-limited speed reference

    def get_measurement(self, mdl: Drive) -> Measurements:
        """Get measurements."""
        u_dc = mdl.converter.meas_dc_voltage()
        i_s_ab = abc2complex(mdl.machine.meas_currents())
        u_c_ab = self._measure_converter_voltage(mdl)
        return Measurements(i_s_ab, u_dc, u_c_ab=u_c_ab)

    def _get_inner_feedback(self, u_s_ab: complex, meas: Measurements) -> Feedbacks:
        if self.ext_ref.w_M is None:
            if self.enabled:
                raise ValueError("Speed reference must be set")
            w_M_ref = 0.0  # No speed reference is needed while the gates are blocked
        else:
            w_M_ref = get_value(self.ext_ref.w_M, self.t)
        self._w_M_ref = self.rate_limiter(self.vhz_ctrl.T_s, w_M_ref)
        return self.vhz_ctrl.get_feedback(u_s_ab, meas.i_c_ab, self._w_M_ref)

    def _compute_inner_output(self, fbk: Feedbacks) -> References:
        ref = self.vhz_ctrl.compute_output(fbk)
        ref.w_M = self._w_M_ref
        return ref

    def _compute_inner_standby_output(self, fbk: Feedbacks) -> References:
        ref = self.vhz_ctrl.compute_standby_output(fbk)
        ref.w_M = self._w_M_ref
        return ref

    def update(self, ref: References, fbk: Feedbacks) -> None:
        """Update controller states."""
        super().update(ref, fbk)
        self.vhz_ctrl.update(ref, fbk)

    def post_process(self) -> TimeSeries:
        """Extend the post-process method."""
        ts = super().post_process()
        self.vhz_ctrl.post_process(ts)
        return ts
