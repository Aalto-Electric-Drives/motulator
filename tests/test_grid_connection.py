"""
Tests for models without control systems.

The tests check the direct-on-line start of an induction machine against its
steady-state equivalent circuit and the commutation of a diode bridge against the
classical overlap formula, with loose tolerances, and that a diode bridge in a composite
model is simulated as if alone.

"""

from math import pi, sqrt

import numpy as np
import pytest
from scipy.integrate import trapezoid

from motulator.common.model import CompositeModel, DCBus, Simulation, ThreePhaseSource
from motulator.drive import model as drive_model
from motulator.grid import control as grid_control
from motulator.grid import model as grid_model

U_G, W_G = 400.0, 2 * pi * 50  # Line-to-line rms voltage and angular frequency


def grid() -> ThreePhaseSource:
    """Stiff grid."""
    return ThreePhaseSource(w_g=W_G, e_g=sqrt(2 / 3) * U_G)


def test_direct_on_line_start() -> None:
    """The induction machine settles at the slip of its equivalent circuit."""
    par = drive_model.InductionMachineInvGammaPars(
        n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224
    )
    mechanics = drive_model.MechanicalSystem(J=0.015)
    mechanics.set_external_load_torque(10.0)
    mdl = drive_model.GridConnectedMachine(
        drive_model.InductionMachine(par), mechanics, grid()
    )
    sim = Simulation(mdl, show_progress=False)
    sim.simulate(t_stop=0.5)
    res = sim.simulate(t_stop=1.0)  # Continues from 0.5 s
    assert res.success and res.ctrl is None
    assert res.mdl.t[-1] == 1.0 and np.all(np.diff(res.mdl.t) >= 0)
    np.testing.assert_array_equal(res.mdl.machine.u_s_ab, res.mdl.ac_source.e_g_ab)
    # Torque of the inverse-Γ equivalent circuit at the final speed
    w_m = 2 * res.mdl.machine.w_M[-1]
    w_r = W_G - w_m  # Slip angular frequency
    Z_r = 1j * W_G * par.L_M * par.R_R / (par.R_R + 1j * w_r * par.L_M)
    i_s = sqrt(2 / 3) * U_G / (par.R_s + 1j * W_G * par.L_sgm + Z_r)
    psi_R = Z_r * i_s / (1j * W_G)
    tau_M = 1.5 * 2 * abs(psi_R) ** 2 * w_r / par.R_R
    assert tau_M == pytest.approx(10.0, rel=0.01)
    assert res.mdl.machine.tau_M[-1] == pytest.approx(10.0, rel=0.01)


def diode_bridge(C_dc: float, I_d: float, L_f: float = 1e-3):
    """Diode bridge supplied from a stiff grid, loaded with a constant DC current."""
    dc_bus = DCBus(C_dc=C_dc, u_dc=sqrt(2) * U_G)
    dc_bus.set_external_dc_current(-I_d)
    return grid_model.GridConverterSystem(
        grid_model.DiodeBridge(), grid_model.LFilter(L_f=L_f), grid(), dc_bus=dc_bus
    )


def current_controller() -> grid_control.GridConverterControlSystem:
    """Current control system of a grid converter."""
    cfg = grid_control.CurrentVectorControllerCfg(i_max=20, L=1e-3)
    return grid_control.GridConverterControlSystem(
        grid_control.CurrentVectorController(cfg)
    )


def test_diode_bridge_commutation() -> None:
    """The mean DC voltage of a diode bridge with a constant DC current is reduced by
    the commutation overlap."""
    L_f, I_d = 1e-3, 10.0
    # The small capacitance keeps the DC current nearly constant
    mdl = diode_bridge(C_dc=2e-6, I_d=I_d, L_f=L_f)
    res = Simulation(mdl, show_progress=False).simulate(t_stop=0.2)
    assert res.success
    t, u_dc = res.mdl.t, res.mdl.dc_bus.u_dc
    end = t > 0.1
    u_dc = trapezoid(u_dc[end], t[end]) / (t[end][-1] - t[end][0])
    u_d0 = 3 * sqrt(2) / pi * U_G
    assert u_dc == pytest.approx(u_d0 - 3 / pi * W_G * L_f * I_d, rel=1e-3)


def test_control_systems_are_checked() -> None:
    """A model needs a control system if and only if its converter is controlled."""
    vsc = grid_model.GridConverterSystem(
        grid_model.VoltageSourceConverter(u_dc=650),
        grid_model.LFilter(L_f=1e-3),
        grid(),
    )
    with pytest.raises(ValueError):
        Simulation(vsc)
    bridge = grid_model.GridConverterSystem(
        grid_model.DiodeBridge(u_dc=500), grid_model.LFilter(L_f=1e-3), grid()
    )
    with pytest.raises(ValueError):
        Simulation(bridge, current_controller())


def test_diode_bridge_with_other_control_systems() -> None:
    """A diode bridge gives the same result alone and in a composite model with an
    independent control system, whose first sampling instant is at the stop time."""
    t_stop, N_eval = 0.1, 1000
    alone = diode_bridge(C_dc=235e-6, I_d=5.0)
    res_alone = Simulation(alone, show_progress=False).simulate(t_stop, N_eval)
    vsc = grid_model.GridConverterSystem(
        grid_model.VoltageSourceConverter(u_dc=650),
        grid_model.LFilter(L_f=1e-3),
        ThreePhaseSource(w_g=0, e_g=0),
    )
    ctrl = current_controller()
    ctrl.set_enable(False)  # No references needed
    mdl = CompositeModel({"bridge": diode_bridge(C_dc=235e-6, I_d=5.0), "vsc": vsc})
    sim = Simulation(mdl, {"vsc": ctrl}, t_start={"vsc": t_stop}, show_progress=False)
    res = sim.simulate(t_stop, N_eval)
    # The evaluation points of the first integration interval coincide
    u_dc = np.interp(res_alone.mdl.t, res.mdl.t, res.mdl.bridge.dc_bus.u_dc)
    np.testing.assert_allclose(u_dc, res_alone.mdl.dc_bus.u_dc, atol=0.1)


def test_clocks_of_composite_model() -> None:
    """The clocks of the models of a composite model advance to the stop time, even if
    no point is saved after the last diode transition."""
    bridge = diode_bridge(C_dc=235e-6, I_d=5.0)
    mdl = CompositeModel({"bridge": bridge})
    Simulation(mdl, show_progress=False).simulate(0.01, N_eval=1)
    assert bridge.t0 == mdl.t0 == 0.01
