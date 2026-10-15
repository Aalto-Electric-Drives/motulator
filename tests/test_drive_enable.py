"""Physical checks for a gate-blocked inverter and the model enable interface."""

from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np
import pytest

import motulator.drive.control.sm as control
from motulator.common.control import ControlSystem
from motulator.common.model import CompositeModel, Model, Simulation, SolverCfg
from motulator.drive import model


@dataclass
class FixedPWMReferences:
    """Sampling period and fixed duty ratios for the gate-blocking checks."""

    T_s: float
    d_abc: tuple[float, float, float] = (0.8, 0.2, 0.5)


class FixedPWM(ControlSystem):
    """Run a fixed, nonzero PWM command to exercise gate blocking independently."""

    def __init__(self, T_s=250e-6):
        super().__init__()
        self.T_s = T_s

    def get_measurement(self, mdl):
        return None

    def get_feedback(self, meas):
        return SimpleNamespace()

    def compute_output(self, fbk):
        return FixedPWMReferences(self.T_s)


def make_drive(
    w_M=100.0, u_dc=540.0, pwm=False, capacitor=False, core_losses=0.0, u_th=0.0
):
    """A PM machine driven by an external shaft, with zero torque command."""
    par = model.SynchronousMachinePars(3, 3.6, 0.036, 0.051, 0.545, core_losses)
    converter = model.VoltageSourceConverter(None if capacitor else u_dc, u_th=u_th)
    dc_bus = model.DCBus(2e-3, u_dc, [converter]) if capacitor else None
    mechanics = model.ExternalRotorSpeed()
    mechanics.set_external_rotor_speed(w_M)
    drive = model.Drive(
        model.SynchronousMachine(par), mechanics, converter, pwm=pwm, dc_bus=dc_bus
    )
    ctrl = control.VectorControlSystem(
        control.CurrentVectorController(
            par, control.CurrentVectorControllerCfg(i_s_max=10, sensorless=False)
        )
    )
    ctrl.set_torque_ref(0)
    return drive, ctrl


@pytest.mark.parametrize("pwm", [False, True])
def test_disabled_open_circuit(pwm):
    """Below diode turn-on, a spinning motor has back EMF and no stator current."""
    drive, _ = make_drive(pwm=pwm)
    # Exercise precisely the generic model interface, without a Drive override.
    mdl: Model[model.VoltageSourceConverter] = Model(
        drive.converter, drive.subsystems, drive.connections, pwm=pwm
    )
    mdl.set_enable(False)
    res = Simulation(mdl, FixedPWM(), show_progress=False).simulate(0.01)
    assert res.success
    assert not np.any(res.mdl.converter.enabled)
    assert np.max(np.abs(res.mdl.machine.i_s_ab)) < 1e-3
    emf = 1j * 163.5 * res.mdl.mechanics.exp_j_theta_M**3
    np.testing.assert_allclose(res.mdl.converter.u_c_ab, emf, atol=0.02)
    assert np.max(np.abs(res.mdl.converter.i_dc_int)) < 1e-3


@pytest.mark.parametrize("composite", [False, True])
@pytest.mark.parametrize("u_th", [0.0, 1.5])
def test_disabled_charges_dc_bus(composite, u_th):
    """Above diode turn-on, shaft power charges the DC capacitor without PWM."""
    drive, ctrl = make_drive(w_M=400, capacitor=True, u_th=u_th)
    drive.set_enable(False)
    mdl = CompositeModel({"drive": drive}) if composite else drive
    controllers = {"drive": ctrl} if composite else ctrl
    res = Simulation(mdl, controllers, show_progress=False).simulate(0.01)
    ts = res.mdl.drive if composite else res.mdl
    assert res.success
    assert ts.dc_bus.u_dc[-1] > 560
    assert np.min(ts.converter.i_dc_int) < -0.5
    assert np.min(ts.machine.tau_M) < -0.5
    assert np.all(ts.converter.i_dc_int < 1e-3)
    ac_power = 1.5 * np.real(ts.converter.u_c_ab * np.conj(ts.converter.i_c_ab))
    np.testing.assert_allclose(
        ac_power, (ts.converter.u_dc + 2 * u_th) * ts.converter.i_dc_int, atol=0.05
    )


@pytest.mark.parametrize("pwm", [False, True])
def test_disable_schedule_and_resume(pwm):
    """Blocking gates takes effect on sampling instants and can be reversed."""
    mdl, ctrl = make_drive(w_M=0, pwm=pwm)
    ctrl.set_torque_ref(3)
    mdl.set_enable(lambda t: t < 0.01 or t >= 0.025)
    sim = Simulation(
        mdl, ctrl, show_progress=False, cfg=SolverCfg(rtol=1e-6, atol=1e-8)
    )
    sim.simulate(0.02)
    res = sim.simulate(0.04)
    ts, conv = res.mdl, res.mdl.converter
    blocked = (ts.t > 0.011) & (ts.t < 0.024)
    assert np.all(~conv.enabled[blocked])
    assert np.all(conv.enabled[ts.t > 0.026])
    assert np.max(np.abs(ts.machine.i_s_ab[(ts.t > 0.021) & (ts.t < 0.024)])) < 1e-3
    assert np.max(np.abs(ts.machine.i_s_ab[ts.t > 0.035])) > 0.1


def test_disabled_with_core_losses():
    """An algebraic core-loss branch must also respect the diode constraints."""
    mdl, ctrl = make_drive(core_losses=0.003)
    mdl.set_enable(False)
    res = Simulation(mdl, ctrl, show_progress=False).simulate(0.01)
    assert res.success
    assert np.max(np.abs(res.mdl.machine.i_s_ab)) < 1e-3


@pytest.mark.parametrize("u_dc", [0.0, 540.0])
def test_disabled_shared_bus_and_output_sampling(u_dc):
    """Two blocked drives share one bus; t_eval must preserve the final state."""
    generating, _ = make_drive(w_M=400, u_dc=u_dc, capacitor=True)
    idle, _ = make_drive(w_M=0)
    dc_bus = generating.dc_bus
    assert dc_bus is not None
    # Rebuild the idle model with the same bus through its public constructor.
    dc_bus.converters.append(idle.converter)
    idle = model.Drive(idle.machine, idle.mechanics, idle.converter, dc_bus=dc_bus)
    generating.set_enable(False)
    idle.set_enable(False)
    mdl = CompositeModel({"generating": generating, "idle": idle})
    sim = Simulation(
        mdl,
        {"generating": FixedPWM(), "idle": FixedPWM()},
        show_progress=False,
        cfg=SolverCfg(rtol=1e-6, atol=1e-9),
    )
    res = sim.simulate(0.003, N_eval=3)
    ts = res.mdl.generating
    assert res.success
    assert ts.dc_bus.u_dc[-1] > u_dc + 1
    assert np.max(np.abs(res.mdl.idle.machine.i_s_ab)) < 1e-3
    np.testing.assert_allclose(
        ts.converter.u_dc, res.mdl.idle.converter.u_dc, atol=1e-12
    )
    assert len(ts.converter.enabled) == len(ts.t)
    assert generating.t0 == mdl.t0


def test_disabled_saturated_current_map():
    """Gate blocking works with a current map without requiring a flux map."""
    par = model.SaturatedSynchronousMachinePars(
        3,
        3.6,
        i_s_dq_fcn=lambda psi: (
            (np.real(psi) - 0.545) / 0.036 + 1j * np.imag(psi) / 0.051
        ),
    )
    mechanics = model.ExternalRotorSpeed()
    mechanics.set_external_rotor_speed(100)
    mdl = model.Drive(
        model.SynchronousMachine(par), mechanics, model.VoltageSourceConverter(540)
    )
    mdl.set_enable(False)
    res = Simulation(mdl, FixedPWM(), show_progress=False).simulate(0.005)
    assert res.success
    assert np.max(np.abs(res.mdl.machine.i_s_ab)) < 1e-3
    np.testing.assert_allclose(np.abs(res.mdl.converter.u_c_ab), 163.5, atol=0.02)


@pytest.mark.parametrize("G_c", [0.0, 0.003])
def test_disabled_induction_machine(G_c):
    """A blocked induction drive loses its initial magnetic energy passively."""
    par = model.InductionMachinePars(
        n_p=2, R_s=3.7, R_r=2.1, L_ell=0.021, L_s=0.245, G_c=G_c
    )
    machine = model.InductionMachine(par)
    machine.state.psi_s_ab = 0.5 + 0j
    machine.state.psi_r_ab = 0.5 + 0j
    mdl = model.Drive(
        machine, model.MechanicalSystem(0.015), model.VoltageSourceConverter(540)
    )
    mdl.set_enable(False)
    res = Simulation(
        mdl, FixedPWM(), show_progress=False, cfg=SolverCfg(rtol=1e-6, atol=1e-9)
    ).simulate(0.02)
    assert res.success
    assert abs(res.mdl.machine.i_s_ab[-1]) < 1e-3
    assert abs(res.mdl.machine.psi_r_ab[-1]) < 0.5
    assert np.max(res.mdl.converter.i_dc_int) < 1e-3


def test_enable_schedule_continuation_between_sampling_instants():
    """Continuing a composite simulation must not resample a slower enable input."""

    def simulate(split):
        fast, _ = make_drive(w_M=0)
        slow, _ = make_drive(w_M=0)
        slow.set_enable(lambda t: t < 0.0004)
        mdl = CompositeModel({"fast": fast, "slow": slow})
        sim = Simulation(
            mdl, {"fast": FixedPWM(), "slow": FixedPWM(1e-3)}, show_progress=False
        )
        if split:
            sim.simulate(0.0003)
        return sim.simulate(0.0012)

    uninterrupted, split = simulate(False), simulate(True)
    np.testing.assert_array_equal(split.mdl.slow.t, uninterrupted.mdl.slow.t)
    np.testing.assert_array_equal(
        split.mdl.slow.converter.enabled, uninterrupted.mdl.slow.converter.enabled
    )
