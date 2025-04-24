"""Base control system for grid converters."""

from dataclasses import dataclass
from typing import Callable, Protocol, Sequence

from motulator.common.control._base import ControlSystem, TimeSeries
from motulator.common.utils import abc2complex, get_value
from motulator.grid.control._controllers import DCBusVoltageController
from motulator.grid.model import GridConverterSystem


# %%
class Feedbacks(Protocol):
    """Protocol defining the required fields for feedback signals."""

    u_dc: float


class References(Protocol):
    """Protocol defining the required fields for reference signals."""

    T_s: float
    d_abc: Sequence[float]
    p_g: float
    u_dc: float | None


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
class GridFormingController[Ref: References, Fbk: Feedbacks](Protocol):
    """Protocol defining the interface for grid-forming controllers."""

    def get_feedback(self, meas: Measurements) -> Fbk:
        """Get feedback signals from measurements."""
        ...

    def compute_output(self, p_g_ref: float, v_c_ref: float, fbk: Fbk) -> Ref:
        """Compute control output from feedback signals."""
        ...

    def update(self, ref: Ref, fbk: Fbk) -> None:
        """Update controller states."""
        ...

    def post_process(self, ts: TimeSeries) -> None:
        """Post-process controller outputs."""
        ...


class GridFollowingController[Ref, Fbk](Protocol):
    """Protocol defining the interface for grid-following controllers."""

    def get_feedback(self, meas: Measurements) -> Fbk:
        """Get feedback signals from measurements."""
        ...

    def compute_output(self, p_g_ref: float, q_g_ref: float, fbk: Fbk) -> Ref:
        """Compute control output from feedback signals."""
        ...

    def update(self, ref: Ref, fbk: Fbk) -> None:
        """Update controller states."""
        ...

    def post_process(self, ts: TimeSeries) -> None:
        """Post-process controller outputs."""
        ...


class GridConverterControlSystem(ControlSystem):
    """
    Grid converter control system.

    This class defines the interface for control systems of grid converters. It is a
    generic class that can be used with different models, measurements, feedback
    signals, and reference signals.

    Parameters
    ----------
    inner_ctrl : GridFormingController | GridFollowingController
        Inner controller.
    dc_bus_voltage_ctrl: DCBusVoltageController, optional
        DC-bus voltage controller. If not given, power-control mode is used.

    """

    def __init__(
        self,
        inner_ctrl: GridFormingController | GridFollowingController,
        dc_bus_voltage_ctrl: DCBusVoltageController | None = None,
    ) -> None:
        super().__init__()
        self.inner_ctrl = inner_ctrl
        self.dc_bus_voltage_ctrl = dc_bus_voltage_ctrl
        self.ext_ref: ExternalReferences = ExternalReferences()

    def set_power_ref(self, ref_fcn: float | Callable[[float], float]) -> None:
        """
        Set the external active power reference.

        Parameters
        ----------
        ref_fcn : Callable[[float], float]
            Active power reference (W), constant or a function of time.

        """
        self.ext_ref.p_g = ref_fcn

    def set_reactive_power_ref(self, ref_fcn: float | Callable[[float], float]) -> None:
        """
        Set the external reactive power reference.

        Parameters
        ----------
        ref_fcn : Callable[[float], float] | float
            Power reference (VAr), constant or a function of time.

        """
        self.ext_ref.q_g = ref_fcn

    def set_ac_voltage_ref(self, ref_fcn: float | Callable[[float], float]) -> None:
        """
        Set the external ac voltage reference.

        Parameters
        ----------
        ref_fcn : float | Callable[[float], float]
            AC-side converter voltage reference (V), constant or a function of time.

        """
        self.ext_ref.v_c = ref_fcn

    def set_dc_bus_voltage_ref(self, ref_fcn: float | Callable[[float], float]) -> None:
        """
        Set the external DC-bus voltage reference.

        Parameters
        ----------
        ref_fcn : float | Callable[[float], float]
            DC-bus voltage reference (V), constant or a function of time.

        """
        self.ext_ref.u_dc = ref_fcn

    def get_measurement(self, mdl: GridConverterSystem) -> Measurements:
        """Get measurements from sensors."""
        u_dc = mdl.converter.meas_dc_voltage()
        i_c_ab = abc2complex(mdl.ac_filter.meas_currents())
        u_g_ab = abc2complex(mdl.ac_filter.meas_pcc_voltages())
        return Measurements(i_c_ab, u_g_ab, u_dc)

    def get_feedback(self, meas: Measurements) -> Feedbacks:
        """Get feedback signals."""
        return self.inner_ctrl.get_feedback(meas)

    def compute_output(self, fbk: Feedbacks) -> References:
        """Compute controller outputs based on feedback."""
        # Determine control mode
        is_gfm = self.ext_ref.v_c is not None
        is_gfl = self.ext_ref.q_g is not None
        is_power_ctrl = self.ext_ref.p_g is not None
        is_dc_bus_ctrl = self.ext_ref.u_dc is not None

        if is_gfm and is_power_ctrl:
            # Grid-forming power control mode
            v_c_ref = get_value(self.ext_ref.v_c, self.t)
            p_g_ref = get_value(self.ext_ref.p_g, self.t)
            ref = self.inner_ctrl.compute_output(p_g_ref, v_c_ref, fbk)
            return ref
        if is_gfm and is_dc_bus_ctrl and self.dc_bus_voltage_ctrl:
            # Grid-forming DC-bus voltage control mode
            v_c_ref = get_value(self.ext_ref.v_c, self.t)
            u_dc_ref = get_value(self.ext_ref.u_dc, self.t)
            p_g_ref = self.dc_bus_voltage_ctrl.compute_output(u_dc_ref, fbk.u_dc)
            ref = self.inner_ctrl.compute_output(p_g_ref, v_c_ref, fbk)
            ref.u_dc = u_dc_ref
            return ref
        if is_gfl and is_power_ctrl:
            # Grid-following power control mode
            q_g_ref = get_value(self.ext_ref.q_g, self.t)
            p_g_ref = get_value(self.ext_ref.p_g, self.t)
            ref = self.inner_ctrl.compute_output(p_g_ref, q_g_ref, fbk)
            return ref
        if is_gfl and is_dc_bus_ctrl and self.dc_bus_voltage_ctrl:
            # Grid-following DC-bus voltage control mode
            q_g_ref = get_value(self.ext_ref.q_g, self.t)
            u_dc_ref = get_value(self.ext_ref.u_dc, self.t)
            p_g_ref = self.dc_bus_voltage_ctrl.compute_output(u_dc_ref, fbk.u_dc)
            ref = self.inner_ctrl.compute_output(p_g_ref, q_g_ref, fbk)
            ref.u_dc = u_dc_ref
            return ref
        raise ValueError

    def update(self, ref: References, fbk: Feedbacks) -> None:
        """Update controller states."""
        super().update(ref, fbk)
        self.inner_ctrl.update(ref, fbk)
        if self.dc_bus_voltage_ctrl and ref.p_g:
            self.dc_bus_voltage_ctrl.update(ref.T_s, ref.p_g)

    def post_process(self) -> TimeSeries:
        """Extend the post-process method."""
        ts = super().post_process()
        self.inner_ctrl.post_process(ts)
        return ts
