"""Base control system for grid converters."""

from cmath import exp
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any, Protocol

from motulator.common.control._base import (
    ConverterControlSystem,
    ConverterFeedbacks,
    InnerController,
)
from motulator.common.control._pwm import PWM
from motulator.common.utils._utils import abc2complex, get_value, line2complex
from motulator.grid.control._common import DCBusVoltageController, References
from motulator.grid.model import GridConverterSystem, VoltageSourceConverter


# %%
class Feedbacks(ConverterFeedbacks, Protocol):
    """
    Protocol defining the feedback signals of the grid converter control system.

    In addition to the signals of `ConverterFeedbacks`, the feedback signals include the
    converter voltage `u_c` for the control laws, in the controller coordinates.

    """

    u_c: complex  # Converter voltage (controller coordinates) for the control laws


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
class GridController(InnerController, Protocol):
    """
    Protocol defining the interface for the inner controllers of grid converters.

    The attribute `grid_forming` is True if the controller takes the AC voltage
    reference and False if it takes the reactive-power reference. The attribute
    `dc_bus_forming` is True if the controller takes the DC-bus voltage reference and
    False if it takes the active-power reference.

    """

    grid_forming: bool
    dc_bus_forming: bool

    def get_feedback(self, u_c_ab: complex, meas: Measurements) -> Any:
        """Get feedback signals, `u_c_ab` being the voltage of the ongoing period."""
        ...

    def compute_output(self, dc_ref: float, ac_ref: float, fbk: Any, /) -> Any:
        """
        Compute control output.

        The reference `dc_ref` is the active power or the DC-bus voltage, and `ac_ref`
        is the AC voltage or the reactive power, depending on the controller.

        """
        ...

    def compute_standby_output(self, fbk: Any) -> Any:
        """Compute control output while the gates are blocked."""
        ...


class GridConverterControlSystem(ConverterControlSystem[GridController]):
    """
    Grid converter control system.

    The inner controller (e.g., grid-following or grid-forming control) gets its
    references externally or, for the active power, from the DC-bus voltage controller.
    The feedback voltages and the modulation are described in `ConverterControlSystem`.

    Parameters
    ----------
    inner_ctrl : GridController
        Inner controller. A grid-forming controller takes the AC voltage reference and a
        grid-following controller the reactive-power reference. A DC-bus-forming
        controller takes the DC-bus voltage reference instead of the active power
        reference.
    dc_bus_voltage_ctrl : DCBusVoltageController, optional
        DC-bus voltage controller, which gives the active-power reference. If not given,
        the active-power reference is set externally, unless the inner controller is
        DC-bus forming.
    pwm : PWM, optional
        Pulse-width modulator, defaults to `PWM()`.

    """

    def __init__(
        self,
        inner_ctrl: GridController,
        dc_bus_voltage_ctrl: DCBusVoltageController | None = None,
        pwm: PWM | None = None,
    ) -> None:
        if inner_ctrl.dc_bus_forming and dc_bus_voltage_ctrl is not None:
            raise ValueError("The inner controller controls the DC-bus voltage")
        super().__init__(inner_ctrl, pwm if pwm is not None else PWM())
        self.dc_bus_voltage_ctrl = dc_bus_voltage_ctrl
        self.ext_ref = ExternalReferences()

    def set_power_ref(self, ref_fcn: float | Callable[[float], float]) -> None:
        """
        Set the external active-power reference.

        Parameters
        ----------
        ref_fcn : float | Callable[[float], float]
            Active-power reference (W), constant or a function of time.

        """
        if self.dc_bus_voltage_ctrl is not None:
            raise ValueError("The DC-bus voltage controller gives the power reference")
        if self.inner_ctrl.dc_bus_forming:
            raise ValueError(
                "DC-bus-forming control takes the DC-bus voltage reference"
            )
        self.ext_ref.p_g = ref_fcn

    def set_reactive_power_ref(self, ref_fcn: float | Callable[[float], float]) -> None:
        """
        Set the external reactive-power reference.

        Parameters
        ----------
        ref_fcn : float | Callable[[float], float]
            Reactive-power reference (VAr), constant or a function of time.

        """
        if self.inner_ctrl.grid_forming:
            raise ValueError("Grid-forming control takes the AC voltage reference")
        self.ext_ref.q_g = ref_fcn

    def set_ac_voltage_ref(self, ref_fcn: float | Callable[[float], float]) -> None:
        """
        Set the external AC voltage reference.

        Parameters
        ----------
        ref_fcn : float | Callable[[float], float]
            Converter voltage magnitude reference (V), constant or a function of time.

        """
        if not self.inner_ctrl.grid_forming:
            raise ValueError(
                "Grid-following control takes the reactive-power reference"
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
        """Sample the sensors."""
        sensors = mdl.sensors
        i_c_ab = abc2complex(sensors.i_c.sample())
        u_g_ab = line2complex(sensors.u_g.sample())
        return Measurements(i_c_ab, u_g_ab, sensors.u_dc.sample())

    def get_feedback(self, meas: Measurements) -> Feedbacks:
        """Get feedback signals."""
        u_c_ab_zoh, u_c_ab = self.pwm.get_realized_voltages(meas.i_c_ab, meas.u_dc)
        # With blocked gates, once the current has decayed, the converter voltage equals
        # the measured PCC voltage with an L filter and approximately with an LCL
        # filter, in which it equals the filter capacitor voltage, so both feedback
        # voltages are the measured PCC voltage
        if not self.enabled:
            u_c_ab_zoh = u_c_ab = meas.u_g_ab
        fbk = self.inner_ctrl.get_feedback(u_c_ab_zoh, meas)
        fbk.u_c = exp(-1j * fbk.theta_c) * u_c_ab
        fbk.u_dc = meas.u_dc
        return fbk

    def compute_output(self, fbk: Feedbacks) -> References:
        """Compute controller outputs based on feedback."""
        inner_ctrl, ext_ref = self.inner_ctrl, self.ext_ref
        # AC voltage (grid forming) or reactive power (grid following) reference
        if inner_ctrl.grid_forming:
            ac_ref = self._get_ref(ext_ref.v_c, "AC voltage")
        else:
            ac_ref = self._get_ref(ext_ref.q_g, "Reactive-power")
        # DC-bus voltage reference (DC-bus forming) or active-power reference (from the
        # DC-bus voltage controller or external)
        u_dc_ref = None
        if inner_ctrl.dc_bus_forming:
            dc_ref = u_dc_ref = self._get_ref(ext_ref.u_dc, "DC-bus voltage")
        elif self.dc_bus_voltage_ctrl is not None:
            u_dc_ref = self._get_ref(ext_ref.u_dc, "DC-bus voltage")
            dc_ref = self.dc_bus_voltage_ctrl.compute_output(u_dc_ref, fbk.u_dc)
        else:
            dc_ref = self._get_ref(ext_ref.p_g, "Active-power")
        ref = inner_ctrl.compute_output(dc_ref, ac_ref, fbk)
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
        ref.d_abc = self.modulate(ref.T_s, ref.u_c, fbk)
        ref.enabled = self.enabled
        return ref

    def update(self, ref: References, fbk: Feedbacks) -> None:
        """Update controller states."""
        super().update(ref, fbk)
        if self.dc_bus_voltage_ctrl is not None and self.enabled:
            self.dc_bus_voltage_ctrl.update(ref.T_s, ref.p_g)
