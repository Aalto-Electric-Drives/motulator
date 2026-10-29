"""
Tests for models without control systems.

The tests check the direct-on-line start of an induction machine against its
steady-state equivalent circuit and the commutation of a diode bridge against the
classical overlap formula, with loose tolerances, and that a diode bridge in a composite
model is simulated as if alone. A current-fed diode bridge, feeding an EMF through a DC
choke, is checked against the same formula, and the gating of its devices is checked
to block the conduction.

"""

from dataclasses import dataclass, field
from math import pi, sqrt

import numpy as np
import pytest
from scipy.integrate import trapezoid

from motulator.common.model import (
    CompositeModel,
    DCBus,
    DCPort,
    DiodeBridge,
    Model,
    Simulation,
    Subsystem,
    SubsystemTimeSeries,
    ThreePhaseSource,
)
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


@dataclass
class EMFLoadInputs:
    """Input variables."""

    i_dc: float = 0.0


@dataclass
class EMFLoadOutputs:
    """Output variables."""

    u_o: float = 0.0


class EMFLoad(Subsystem):
    """EMF in series with a resistance and an inductance, fed from a bridge."""

    def __init__(self, e: float, R: float, L: float) -> None:
        self.e, self.R, self.L = e, R, L
        self.inp: EMFLoadInputs = EMFLoadInputs()
        self.out: EMFLoadOutputs = EMFLoadOutputs(e)
        self.state = None

    def set_outputs(self, t: float) -> None:
        self.out.u_o = self.e + self.R * self.inp.i_dc

    def rhs(self, t: float) -> list[complex]:
        return []

    def compute_dc_port(self) -> DCPort:
        return DCPort(self.e, self.R, self.L)

    def create_time_series(self, t: np.ndarray) -> SubsystemTimeSeries:
        return EMFLoadTimeSeries(t)


@dataclass
class EMFLoadTimeSeries(SubsystemTimeSeries):
    """Time series of the inputs."""

    t: np.ndarray
    i_dc: np.ndarray = field(default_factory=lambda: np.array([]))
    u_o: np.ndarray = field(default_factory=lambda: np.array([]))


class CurrentFedRectifier(Model[DiodeBridge]):
    """Diode bridge feeding an EMF load through a DC choke."""

    def __init__(self, converter: DiodeBridge, L_f: float, load: EMFLoad) -> None:
        ac_filter, ac_source = grid_model.LFilter(L_f=L_f), grid()
        connections = {
            ("converter", "i_c_ab"): ("ac_filter", "i_c_ab"),
            ("ac_filter", "u_c_ab"): ("converter", "u_c_ab"),
            ("ac_filter", "e_g_ab"): ("ac_source", "e_g_ab"),
            ("load", "i_dc"): ("converter", "i_dc"),
        }
        converter.ac_load, converter.dc_load = ac_filter, load
        subsystems = {
            "converter": converter,
            "ac_filter": ac_filter,
            "ac_source": ac_source,
            "load": load,
        }
        super().__init__(converter, subsystems, connections)


def test_current_fed_diode_bridge() -> None:
    """The mean DC voltage of a diode bridge feeding an EMF through a DC choke equals
    the ideal voltage reduced by the commutation overlap, and the DC voltage balances
    the load in the steady state."""
    L_f, R, L_dc, e = 1e-3, 2.0, 50e-3, 400.0
    mdl = CurrentFedRectifier(DiodeBridge(), L_f, EMFLoad(e, R, L_dc))
    res = Simulation(mdl, show_progress=False).simulate(t_stop=0.3)
    assert res.success
    t, c = res.mdl.t, res.mdl.converter
    end = t > 0.2

    def mean(x: np.ndarray) -> float:
        return trapezoid(x[end], t[end]) / (t[end][-1] - t[end][0])

    i_dc, u_dc = mean(-c.i_dc_int), mean(c.u_dc)
    assert i_dc > 10
    u_d0 = 3 * sqrt(2) / pi * U_G
    assert u_dc == pytest.approx(u_d0 - 3 / pi * W_G * L_f * i_dc, rel=2e-3)
    assert u_dc == pytest.approx(e + R * i_dc, rel=2e-3)
    # The DC current ripple is small and at the sixth harmonic
    assert np.ptp(-c.i_dc_int[end]) < 0.05 * i_dc


def test_gated_diode_bridge() -> None:
    """Gating all the devices gives the diode bridge, and gating none blocks it."""
    L_f, R, L_dc, e = 1e-3, 2.0, 20e-3, 400.0
    results = []
    for gated in (None, np.ones((2, 3), dtype=bool), np.zeros((2, 3), dtype=bool)):
        converter = DiodeBridge()
        converter.gated = gated
        mdl = CurrentFedRectifier(converter, L_f, EMFLoad(e, R, L_dc))
        results.append(Simulation(mdl, show_progress=False).simulate(t_stop=0.1))
    diodes, all_gated, none_gated = results
    np.testing.assert_array_equal(
        all_gated.mdl.converter.u_dc, diodes.mdl.converter.u_dc
    )
    np.testing.assert_array_equal(
        all_gated.mdl.converter.u_c_ab, diodes.mdl.converter.u_c_ab
    )
    assert np.max(np.abs(none_gated.mdl.ac_filter.i_c_ab)) < 1e-3
    np.testing.assert_allclose(
        none_gated.mdl.converter.u_c_ab, none_gated.mdl.ac_source.e_g_ab, atol=1e-6
    )


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
