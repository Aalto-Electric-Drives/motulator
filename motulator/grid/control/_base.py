"""Base control system for grid converters."""

from cmath import exp
from collections.abc import Callable
from dataclasses import dataclass
from typing import Protocol

from motulator.common.control._base import ControlSystem, TimeSeries
from motulator.common.control._pwm import PWM
from motulator.common.utils._utils import abc2complex, get_value, line2complex
from motulator.grid.control._common import DCBusVoltageController, References
from motulator.grid.model import GridConverterSystem, VoltageSourceConverter


# %%
class Feedbacks(Protocol):
    """Protocol defining the required fields for feedback signals."""

    w_c: float  # Angular speed of the coordinate system (rad/s), not post-processed
    theta_c: float  # Angular position of the coordinate system (rad)
    u_c: complex  # Converter voltage (controller coordinates) for the control laws
    u_dc: float  # DC-bus voltage (V)


@dataclass
class ExternalReferences:
    """External reference signals."""

    p_g: float | Callable[[float], float] | None = None
    q_g: float | Callable[[float], float] | None = None
    u_dc: float | Callable[[float], float] | None = None
    v_c: float | Callable[[float], float] | None = None


@dataclass
class Measurements:
    """Measured signals."""

    i_c_ab: complex
    u_g_ab: complex
    u_dc: float


# %%
class InnerController[Ref: References, Fbk](Protocol):
    """Protocol defining the interface for the inner controllers."""

    # True: the controller takes the AC voltage reference (grid forming). False: it
    # takes the reactive power reference (grid following).
    grid_forming: bool
    # True: the controller takes the DC-bus voltage reference. False: it takes the
    # active power reference.
    dc_bus_forming: bool
    k_comp: float  # Angle-advance factor, see GridConverterControlSystem

    def get_feedback(self, u_c_ab: complex, meas: Measurements) -> Fbk:
        """Get feedback signals, `u_c_ab` being the voltage of the ongoing period."""
        ...

    def compute_output(self, dc_ref: float, ac_ref: float, fbk: Fbk, /) -> Ref:
        """
        Compute control output.

        The reference `dc_ref` is the active power or the DC-bus voltage, and `ac_ref`
        is the AC voltage or the reactive power, depending on the controller.

        """
        ...

    def compute_standby_output(self, fbk: Fbk) -> Ref:
        """Compute control output while the gates are blocked."""
        ...

    def update(self, ref: Ref, fbk: Fbk) -> None:
        """Update controller states."""
        ...

    def post_process(self, ts: TimeSeries) -> None:
        """
        Post-process controller outputs.

        If the signals are transformed to other coordinates, `fbk.theta_c` is updated
        to the angle of these coordinates. The angular speed `fbk.w_c` is not updated,
        i.e., it remains that of the controller coordinates, which equals that of the
        new coordinates in steady state.

        """
        ...


class GridConverterControlSystem(ControlSystem):
    """
    Grid converter control system.

    This class defines the interface for control systems of grid converters. It is a
    generic class that can be used with different models, measurements, feedback
    signals, and reference signals.

    The inner controller gets the realized voltage of the ongoing sampling period for
    its observer, while the feedback voltage `u_c` for the control laws is the realized
    voltage averaged over the previous and the ongoing sampling periods. Before the
    modulation, the angle of the voltage reference is advanced by `k_comp*T_s*w_c`,
    where `k_comp` is given by the inner controller. The value 1.5 compensates for the
    computational delay of one sampling period and the delay of half a sampling period
    caused by the zero-order hold of the PWM [#Bae2003]_. The value 0 means that the
    angle is not advanced, which is used if the controller takes the delays into account
    itself.

    Parameters
    ----------
    inner_ctrl : InnerController
        Inner controller. A grid-forming controller takes the AC voltage reference
        and a grid-following controller the reactive power reference. A
        DC-bus-forming controller takes the DC-bus voltage reference instead of the
        active power reference.
    dc_bus_voltage_ctrl : DCBusVoltageController, optional
        DC-bus voltage controller, which gives the active power reference. If not
        given, the active power reference is set externally, unless the inner
        controller is DC-bus forming.
    pwm : PWM, optional
        Pulse-width modulator, defaults to `PWM()`.

    References
    ----------
    .. [#Bae2003] Bae, Sul, "A compensation method for time delay of full-digital
       synchronous frame current regulator of PWM AC drives," IEEE Trans. Ind. Appl.,
       2003, https://doi.org/10.1109/TIA.2003.810660

    """

    def __init__(
        self,
        inner_ctrl: InnerController,
        dc_bus_voltage_ctrl: DCBusVoltageController | None = None,
        pwm: PWM | None = None,
    ) -> None:
        super().__init__()
        if inner_ctrl.dc_bus_forming and dc_bus_voltage_ctrl is not None:
            raise ValueError("The inner controller controls the DC-bus voltage")
        self.pwm = pwm if pwm is not None else PWM()
        self.inner_ctrl = inner_ctrl
        self.k_comp = inner_ctrl.k_comp
        self.dc_bus_voltage_ctrl = dc_bus_voltage_ctrl
        self.ext_ref: ExternalReferences = ExternalReferences()

    def set_power_ref(self, ref_fcn: float | Callable[[float], float]) -> None:
        """
        Set the external active power reference.

        Parameters
        ----------
        ref_fcn : float | Callable[[float], float]
            Active power reference (W), constant or a function of time.

        """
        if self.dc_bus_voltage_ctrl is not None:
            raise ValueError("The DC-bus voltage controller gives the power reference")
        if self.inner_ctrl.dc_bus_forming:
            raise ValueError("DC-bus-forming control takes the DC-bus voltage ref")
        self.ext_ref.p_g = ref_fcn

    def set_reactive_power_ref(self, ref_fcn: float | Callable[[float], float]) -> None:
        """
        Set the external reactive power reference.

        Parameters
        ----------
        ref_fcn : float | Callable[[float], float]
            Reactive power reference (VAr), constant or a function of time.

        """
        if self.inner_ctrl.grid_forming:
            raise ValueError("Grid-forming control takes the AC voltage reference")
        self.ext_ref.q_g = ref_fcn

    def set_ac_voltage_ref(self, ref_fcn: float | Callable[[float], float]) -> None:
        """
        Set the external ac voltage reference.

        Parameters
        ----------
        ref_fcn : float | Callable[[float], float]
            AC-side converter voltage reference (V), constant or a function of time.

        """
        if not self.inner_ctrl.grid_forming:
            raise ValueError(
                "Grid-following control takes the reactive power reference"
            )
        self.ext_ref.v_c = ref_fcn

    def set_dc_bus_voltage_ref(self, ref_fcn: float | Callable[[float], float]) -> None:
        """
        Set the external DC-bus voltage reference.

        Parameters
        ----------
        ref_fcn : float | Callable[[float], float]
            DC-bus voltage reference (V), constant or a function of time.

        """
        if self.dc_bus_voltage_ctrl is None and not self.inner_ctrl.dc_bus_forming:
            raise ValueError("DC-bus voltage control requires dc_bus_voltage_ctrl")
        self.ext_ref.u_dc = ref_fcn

    def get_measurement(
        self, mdl: GridConverterSystem[VoltageSourceConverter]
    ) -> Measurements:
        """Get measurements from sensors."""
        u_dc = mdl.converter.meas_dc_voltage()
        i_c_ab = abc2complex(mdl.ac_filter.meas_currents())
        u_g_ab = line2complex(mdl.ac_filter.meas_pcc_voltages())
        return Measurements(i_c_ab, u_g_ab, u_dc)

    def get_feedback(self, meas: Measurements) -> Feedbacks:
        """Get feedback signals."""
        # The observers use the voltage of the ongoing sampling period, while the
        # control laws use the voltage averaged over the previous and ongoing periods.
        # With blocked gates, the converter voltage equals the measured PCC voltage
        # (the filter capacitor voltage with an LCL filter) once the current has
        # decayed, so both are that voltage. The realized voltage is computed also
        # then, since it passes the measured currents to the PWM for the feedforward
        # compensation of the duty ratios applied after the release.
        u_c_ab_zoh = self.pwm.get_realized_voltage(
            meas.i_c_ab, meas.u_dc, average=False
        )
        u_c_ab = self.pwm.get_realized_voltage(meas.i_c_ab, meas.u_dc)
        if not self.enabled:
            u_c_ab_zoh = u_c_ab = meas.u_g_ab
        fbk = self.inner_ctrl.get_feedback(u_c_ab_zoh, meas)
        fbk.u_c = exp(-1j * fbk.theta_c) * u_c_ab
        fbk.u_dc = meas.u_dc
        return fbk

    def compute_output(self, fbk: Feedbacks) -> References:
        """Compute controller outputs based on feedback."""
        ext_ref = self.ext_ref

        # AC voltage (grid forming) or reactive power (grid following) reference
        v_c_or_q_g_ref = ext_ref.v_c if self.inner_ctrl.grid_forming else ext_ref.q_g
        if v_c_or_q_g_ref is None:
            quantity = (
                "AC voltage" if self.inner_ctrl.grid_forming else "Reactive power"
            )
            raise ValueError(f"{quantity} reference must be set")

        # DC-bus voltage reference (DC-bus forming) or active power reference (from
        # the DC-bus voltage controller or external)
        u_dc_ref = None
        if self.inner_ctrl.dc_bus_forming or self.dc_bus_voltage_ctrl is not None:
            if ext_ref.u_dc is None:
                raise ValueError("DC-bus voltage reference must be set")
            u_dc_ref = get_value(ext_ref.u_dc, self.t)
        if self.inner_ctrl.dc_bus_forming and u_dc_ref is not None:
            dc_ref = u_dc_ref
        elif self.dc_bus_voltage_ctrl is not None and u_dc_ref is not None:
            dc_ref = self.dc_bus_voltage_ctrl.compute_output(u_dc_ref, fbk.u_dc)
        elif ext_ref.p_g is not None:
            dc_ref = get_value(ext_ref.p_g, self.t)
        else:
            raise ValueError("Active power reference must be set")

        ref = self.inner_ctrl.compute_output(
            dc_ref, get_value(v_c_or_q_g_ref, self.t), fbk
        )
        ref.u_dc = u_dc_ref
        return self._add_duty_ratios(ref, fbk)

    def compute_standby_output(self, fbk: Feedbacks) -> References:
        """Compute controller output while the gates are blocked."""
        ref = self.inner_ctrl.compute_standby_output(fbk)
        if self.ext_ref.u_dc is not None:
            ref.u_dc = get_value(self.ext_ref.u_dc, self.t)
        return self._add_duty_ratios(ref, fbk)

    def _add_duty_ratios(self, ref: References, fbk: Feedbacks) -> References:
        """Compute the duty ratios from the voltage reference."""
        u_c_ab_ref = exp(1j * fbk.theta_c) * ref.u_c
        u_c_ab_ref *= exp(1j * self.k_comp * ref.T_s * fbk.w_c)
        ref.d_abc = self.pwm(ref.T_s, u_c_ab_ref, fbk.u_dc, fbk.w_c)
        ref.enabled = self.enabled
        return ref

    def update(self, ref: References, fbk: Feedbacks) -> None:
        """Update controller states."""
        super().update(ref, fbk)
        self.inner_ctrl.update(ref, fbk)
        if self.dc_bus_voltage_ctrl and ref.u_dc is not None and self.enabled:
            self.dc_bus_voltage_ctrl.update(ref.T_s, ref.p_g)

    def post_process(self) -> TimeSeries:
        """Extend the post-process method."""
        ts = super().post_process()
        self.inner_ctrl.post_process(ts)
        return ts
