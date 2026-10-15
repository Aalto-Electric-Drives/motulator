"""
Tests for models sharing a DC bus and simulated with several control systems.

The closed-loop test checks the power balance of a drive with an active front end,
whose control systems have different sampling periods, with loose tolerances.

"""

from math import pi

import numpy as np
import pytest
from scipy.integrate import trapezoid

import motulator.drive.control.sm as sm_control
from motulator.common.model import CompositeModel, DCBus, Simulation
from motulator.drive import model as drive_model
from motulator.grid import control as grid_control
from motulator.grid import model as grid_model

U_DC = 650.0
C_DC = 470e-6
L_F = 15e-3
T_S_GRID, T_S_DRIVE, T_START_DRIVE = 100e-6, 125e-6, 40e-6
T_STOP = 0.4


def time_average(x: np.ndarray, t: np.ndarray, t_start: float) -> float:
    """Average over the unevenly spaced samples after `t_start`."""
    end = t > t_start
    return float(trapezoid(x[end], t[end]) / (t[end][-1] - t[end][0]))


def ipmsm_drive(
    converter: drive_model.VoltageSourceConverter,
    dc_bus: DCBus | None,
    pwm: bool = False,
) -> drive_model.Drive:
    """2.2-kW IPMSM drive."""
    par = drive_model.SynchronousMachinePars(
        n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545
    )
    return drive_model.Drive(
        drive_model.SynchronousMachine(par),
        drive_model.MechanicalSystem(J=0.015),
        converter,
        pwm=pwm,
        dc_bus=dc_bus,
    )


def grid_system(
    afe: grid_model.VoltageSourceConverter, dc_bus: DCBus, pwm: bool = False
) -> grid_model.GridConverterSystem:
    """Grid converter system of the active front end."""
    return grid_model.GridConverterSystem(
        afe,
        grid_model.LFilter(L_f=L_F),
        grid_model.ThreePhaseSource(w_g=2 * pi * 50, e_g=np.sqrt(2 / 3) * 400),
        pwm=pwm,
        dc_bus=dc_bus,
    )


def afe_ctrl() -> grid_control.GridConverterControlSystem:
    """Grid-following control with DC-bus voltage control."""
    cfg = grid_control.CurrentVectorControllerCfg(i_max=10.0, L=L_F, T_s=T_S_GRID)
    dc_ctrl = grid_control.DCBusVoltageController(C_dc=C_DC, alpha_dc=2 * pi * 50)
    ctrl = grid_control.GridConverterControlSystem(
        grid_control.CurrentVectorController(cfg), dc_ctrl
    )
    ctrl.set_dc_bus_voltage_ref(U_DC)
    ctrl.set_reactive_power_ref(0)
    return ctrl


class VaryingSamplingPeriod(sm_control.VectorControlSystem):
    """Vector control whose sampling period doubles at `t_change`."""

    def __init__(
        self, vector_ctrl: sm_control.CurrentVectorController, t_change: float
    ) -> None:
        super().__init__(
            vector_ctrl, sm_control.SpeedController(J=0.015, alpha_s=2 * pi * 4)
        )
        self.cvc = vector_ctrl
        self.t_change = t_change
        self.T_s0 = vector_ctrl.T_s

    def compute_output(self, fbk):  # noqa: ANN001, ANN201
        self.cvc.T_s = self.T_s0 if self.t < self.t_change else 2 * self.T_s0
        return super().compute_output(fbk)


def ipmsm_cvc(
    T_s: float, t_change: float | None = None
) -> sm_control.VectorControlSystem:
    """Sensorless current-vector control of the IPMSM."""
    par = sm_control.SynchronousMachinePars(
        n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545
    )
    cfg = sm_control.CurrentVectorControllerCfg(i_s_max=9.0, T_s=T_s)
    vector_ctrl = sm_control.CurrentVectorController(par, cfg)
    if t_change is not None:
        return VaryingSamplingPeriod(vector_ctrl, t_change)
    return sm_control.VectorControlSystem(
        vector_ctrl, sm_control.SpeedController(J=0.015, alpha_s=2 * pi * 4)
    )


@pytest.mark.parametrize("pwm", [False, True])
def test_composite_of_one_model_equals_model(pwm: bool) -> None:
    """A composite of a single model gives the same results as the model alone."""

    def simulate(composite: bool) -> drive_model.SimulationResults:
        conv = drive_model.VoltageSourceConverter()
        dc_bus = DCBus(C_dc=C_DC, u_dc=U_DC, converters=[conv])
        dc_bus.set_external_dc_current(lambda t: (t > 0.05) * 5)
        mdl = ipmsm_drive(conv, dc_bus, pwm)
        ctrl = ipmsm_cvc(T_S_DRIVE)
        ctrl.set_speed_ref(lambda t: (t > 0.02) * 50.0)
        if not composite:
            return Simulation(mdl, ctrl, show_progress=False).simulate(t_stop=0.1)
        sim = Simulation(CompositeModel({"drive": mdl}), {"drive": ctrl})
        res = sim.simulate(t_stop=0.1)
        return drive_model.SimulationResults(res.mdl.drive, res.ctrl.drive)

    ref, res = simulate(False), simulate(True)
    assert res.ctrl.t.shape == ref.ctrl.t.shape
    np.testing.assert_array_equal(res.mdl.t, ref.mdl.t)
    np.testing.assert_array_equal(res.mdl.dc_bus.u_dc, ref.mdl.dc_bus.u_dc)
    np.testing.assert_array_equal(res.mdl.converter.u_dc, ref.mdl.dc_bus.u_dc)
    np.testing.assert_array_equal(res.mdl.converter.u_c_ab, ref.mdl.converter.u_c_ab)
    np.testing.assert_array_equal(res.mdl.machine.w_M, ref.mdl.machine.w_M)


def test_diode_bridge_dc_bus() -> None:
    """The diode bridge charges the DC bus, and its current does not reverse."""
    conv = drive_model.VoltageSourceConverter()
    dc_bus = drive_model.DiodeBridgeDCBus(
        C_dc=235e-6, L_dc=1e-3, U_g=400, f_g=50, converters=[conv]
    )
    mdl = ipmsm_drive(conv, dc_bus, pwm=True)
    ctrl = ipmsm_cvc(T_S_DRIVE)
    ctrl.set_speed_ref(lambda t: (t > 0.02) * 100.0)
    mdl.mechanics.set_external_load_torque(lambda t: (t > 0.1) * 7.0)
    res = Simulation(mdl, ctrl, show_progress=False).simulate(t_stop=0.3)
    assert res.success

    # Averages over the last 100 ms
    t, dc_bus_ts = res.mdl.t, res.mdl.dc_bus
    u_dc = time_average(dc_bus_ts.u_dc, t, 0.2)
    p_dc = time_average(dc_bus_ts.u_di * dc_bus_ts.i_L, t, 0.2)
    p_M = time_average(res.mdl.machine.tau_M * res.mdl.machine.w_M, t, 0.2)
    w_M = time_average(res.mdl.machine.w_M, t, 0.2)

    assert 1.35 * 400 * 0.95 < u_dc < np.sqrt(2) * 400  # Between the mean and peak
    assert np.min(dc_bus_ts.i_L) > -0.01  # Only numerical undershoot
    assert w_M == pytest.approx(100.0, rel=0.05)  # Still recovering from the load step
    # The rectified power covers the mechanical power and the losses
    assert p_dc == pytest.approx(p_M, rel=0.15) and p_dc > p_M > 0.9 * 7.0 * 100.0


def test_active_front_end_drive() -> None:
    """The grid power balances the mechanical power through the shared DC bus."""
    afe = grid_model.VoltageSourceConverter(u_dc=U_DC)
    inv = drive_model.VoltageSourceConverter(u_dc=U_DC)
    dc_bus = DCBus(C_dc=C_DC, u_dc=U_DC, converters=[afe, inv])
    grid_sys = grid_system(afe, dc_bus, pwm=True)
    drive_sys = ipmsm_drive(inv, dc_bus, pwm=True)
    mdl = CompositeModel({"grid": grid_sys, "drive": drive_sys})

    ctrl_g, ctrl_m = afe_ctrl(), ipmsm_cvc(T_S_DRIVE)
    w_M_ref, tau_L = 100.0, 7.0
    ctrl_m.set_speed_ref(lambda t: (t > 0.02) * w_M_ref)
    drive_sys.mechanics.set_external_load_torque(lambda t: (t > 0.15) * tau_L)

    with pytest.raises(ValueError):  # A control system is needed for each model
        Simulation(mdl, ctrl_m)
    with pytest.raises(ValueError):
        Simulation(drive_sys, {"drive": ctrl_m})
    # The sampling instants coincide every 500 µs, exercising their merging
    sim = Simulation(mdl, {"grid": ctrl_g, "drive": ctrl_m}, show_progress=False)
    res = sim.simulate(t_stop=T_STOP)
    assert res.success

    # The control systems run at their own sampling instants, which are all included
    # in the model time series
    t_g, t_m = res.ctrl.grid.t, res.ctrl.drive.t
    assert np.allclose(np.diff(t_g), T_S_GRID) and t_g[0] == 0
    assert np.allclose(np.diff(t_m), T_S_DRIVE) and t_m[0] == 0
    assert np.all(np.isin(np.round(t_g, 10), np.round(res.mdl.t, 10)))
    assert np.all(np.isin(np.round(t_m, 10), np.round(res.mdl.t, 10)))

    # Averages over the last 50 ms
    t, grid_ts, drive_ts = res.mdl.t, res.mdl.grid, res.mdl.drive
    p_g = 1.5 * np.real(grid_ts.ac_filter.e_g_ab * np.conj(grid_ts.ac_filter.i_g_ab))
    p_mech = drive_ts.machine.tau_M * drive_ts.machine.w_M
    p_loss = 1.5 * 3.6 * np.abs(drive_ts.machine.i_s_ab) ** 2
    w_M = time_average(drive_ts.machine.w_M, t, T_STOP - 0.05)
    u_dc = time_average(drive_ts.dc_bus.u_dc, t, T_STOP - 0.05)
    p_drive = time_average(p_mech + p_loss, t, T_STOP - 0.05)

    assert w_M == pytest.approx(w_M_ref, rel=0.02)
    assert u_dc == pytest.approx(U_DC, rel=0.01)
    assert p_drive == pytest.approx(tau_L * w_M_ref, rel=0.2)
    assert -time_average(p_g, t, T_STOP - 0.05) == pytest.approx(p_drive, rel=0.03)


def test_simulation_continues_between_calls() -> None:
    """Two consecutive runs give the same result as a single run."""

    def simulate(t_stops: list[float]) -> np.ndarray:
        # PWM with dead time on both sides, whose blanking state continues across
        # the sampling periods
        afe = grid_model.VoltageSourceConverter(u_dc=U_DC, t_d=2e-6)
        inv = drive_model.VoltageSourceConverter(u_dc=U_DC, t_d=2e-6)
        dc_bus = DCBus(C_dc=C_DC, u_dc=U_DC, converters=[afe, inv])
        grid_sys = grid_system(afe, dc_bus, pwm=True)
        drive_sys = ipmsm_drive(inv, dc_bus, pwm=True)
        mdl = CompositeModel({"grid": grid_sys, "drive": drive_sys})
        ctrl_g, ctrl_m = afe_ctrl(), ipmsm_cvc(T_S_DRIVE)
        ctrl_m.set_speed_ref(50.0)
        # The first run stops at a sampling instant of the grid-side control, in the
        # middle of a switching period of the machine-side converter
        sim = Simulation(
            mdl,
            {"grid": ctrl_g, "drive": ctrl_m},
            show_progress=False,
            t_start={"drive": T_START_DRIVE},
        )
        results = [sim.simulate(t_stop=t_stop) for t_stop in t_stops]
        assert results[-1].ctrl.drive.t[0] == T_START_DRIVE
        return np.column_stack((results[-1].mdl.t, results[-1].mdl.drive.dc_bus.u_dc))

    np.testing.assert_array_equal(simulate([0.0123, 0.03]), simulate([0.03]))


def test_sampling_period_changes_during_run() -> None:
    """A control system can change its sampling period at any step."""
    afe = grid_model.VoltageSourceConverter(u_dc=U_DC)
    inv = drive_model.VoltageSourceConverter(u_dc=U_DC)
    dc_bus = DCBus(C_dc=C_DC, u_dc=U_DC, converters=[afe, inv])
    mdl = CompositeModel(
        {"grid": grid_system(afe, dc_bus), "drive": ipmsm_drive(inv, dc_bus)}
    )
    t_change = 0.1
    ctrl_g, ctrl_m = afe_ctrl(), ipmsm_cvc(T_S_DRIVE, t_change)
    ctrl_m.set_speed_ref(50.0)
    sim = Simulation(mdl, {"grid": ctrl_g, "drive": ctrl_m}, show_progress=False)
    res = sim.simulate(t_stop=0.2)
    assert res.success

    t_g, t_m = res.ctrl.grid.t, res.ctrl.drive.t
    assert np.allclose(np.diff(t_g), T_S_GRID)
    assert np.allclose(np.diff(t_m)[t_m[:-1] < t_change], T_S_DRIVE)
    assert np.allclose(np.diff(t_m)[t_m[:-1] >= t_change], 2 * T_S_DRIVE)
    assert np.all(np.isin(np.round(t_m, 10), np.round(res.mdl.t, 10)))
    w_M = time_average(res.mdl.drive.machine.w_M, res.mdl.t, 0.15)
    assert w_M == pytest.approx(50.0, rel=0.05)


def test_dc_bus_misuse_is_rejected() -> None:
    """The DC bus and the models check their mutual wiring."""
    conv = drive_model.VoltageSourceConverter(u_dc=U_DC)
    with pytest.raises(ValueError):  # The converter is not fed from the DC bus
        ipmsm_drive(conv, DCBus(C_dc=C_DC, u_dc=U_DC, converters=[]))
    with pytest.raises(ValueError):  # Neither a DC-bus voltage nor a DC bus
        mdl = ipmsm_drive(drive_model.VoltageSourceConverter(), None)
        Simulation(mdl, ipmsm_cvc(T_S_DRIVE), show_progress=False).simulate(1e-3)
    with pytest.raises(ValueError):  # The converters must be distinct
        DCBus(C_dc=C_DC, u_dc=U_DC, converters=[conv, conv])
    afe = grid_model.VoltageSourceConverter(u_dc=U_DC)
    dc_bus = DCBus(C_dc=C_DC, u_dc=U_DC, converters=[afe, conv])
    drive_sys = ipmsm_drive(conv, dc_bus)
    with pytest.raises(ValueError):  # The models must have distinct converters
        CompositeModel({"a": drive_sys, "b": drive_sys})
    with pytest.raises(ValueError):  # Invalid model name
        CompositeModel({"__class__": drive_sys})
    ctrl = ipmsm_cvc(T_S_DRIVE)
    for t_start in (-1e-3, float("nan"), float("inf")):
        with pytest.raises(ValueError):  # Invalid first sampling instant
            Simulation(drive_sys, ctrl, t_start=t_start)
    assert ctrl.t == 0  # The control system is left unchanged
    with pytest.raises(ValueError):  # The DC bus is not given to the grid model
        CompositeModel(
            {
                "grid": grid_model.GridConverterSystem(
                    afe,
                    grid_model.LFilter(L_f=L_F),
                    grid_model.ThreePhaseSource(w_g=2 * pi * 50, e_g=1.0),
                ),
                "drive": ipmsm_drive(conv, dc_bus),
            }
        )
