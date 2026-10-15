"""Drive control enable and sensored flying-start regression tests."""

import numpy as np
import pytest

import motulator.drive.control.im as im_control
import motulator.drive.control.sm as sm_control
from motulator.common.model import Simulation, SolverCfg
from motulator.drive import model


def spinning_drive(
    kind="cvc", w_M=80.0, pwm=False, delay=1, flying_start=None, single_update=False
):
    """Use a free shaft with a known initial speed and a nonzero rotor angle."""
    par = model.SynchronousMachinePars(3, 3.6, 0.036, 0.051, 0.545)
    if kind == "fvc":
        inner = sm_control.FluxVectorController(
            par, sm_control.FluxVectorControllerCfg(i_s_max=10, sensorless=False)
        )
    else:
        inner = sm_control.CurrentVectorController(
            par,
            sm_control.CurrentVectorControllerCfg(
                i_s_max=10, sensorless=False, discrete=kind == "discrete"
            ),
        )
    mechanics = model.MechanicalSystem(0.015)
    mechanics.state.w_M = complex(w_M)
    mechanics.state.theta_M = complex(0.7)
    machine = model.SynchronousMachine(par)
    machine.state.theta_m = complex(2.1)
    mdl = model.Drive(
        machine,
        mechanics,
        model.VoltageSourceConverter(540),
        pwm=pwm,
        delay=delay,
        single_update=single_update,
    )
    kwargs = {} if flying_start is None else {"flying_start": flying_start}
    ctrl = sm_control.VectorControlSystem(inner, **kwargs)
    return mdl, ctrl


def simulate(mdl, ctrl, t_stop=0.04):
    """Resolve the passive diode zero-current boundary layer."""
    return Simulation(
        mdl, ctrl, show_progress=False, cfg=SolverCfg(rtol=1e-8, atol=1e-10)
    ).simulate(t_stop)


def test_control_enable_blocks_gates_without_references():
    """A disabled control must not energize the motor or require torque references."""
    mdl, ctrl = spinning_drive()
    ctrl.set_enable(False)
    res = simulate(mdl, ctrl, 0.005)
    assert res.success
    assert not np.any(res.mdl.converter.enabled)
    assert not np.any(res.ctrl.ref.enabled)
    assert np.max(np.abs(res.mdl.machine.i_s_ab)) < 1e-3
    assert np.max(np.abs(res.ctrl.ref.tau_M)) == 0


def test_control_disabled_with_empty_dc_bus():
    """A blocked controller must also work before the DC capacitor is charged."""
    mdl, ctrl = spinning_drive(w_M=0)
    converter = model.VoltageSourceConverter(0)
    mdl = model.Drive(mdl.machine, mdl.mechanics, converter)
    ctrl.set_enable(False)
    res = simulate(mdl, ctrl, 0.001)
    assert res.success
    assert np.all(np.isfinite(res.ctrl.ref.d_abc))
    assert np.max(np.abs(res.mdl.machine.i_s_ab)) < 1e-9


def test_model_enable_interlocks_control_enable():
    """Neither enable command may override a disable command from the other side."""
    mdl, ctrl = spinning_drive(w_M=0)
    mdl.set_enable(False)
    ctrl.set_enable(True)
    ctrl.set_torque_ref(3)
    sim = Simulation(mdl, ctrl, show_progress=False)
    res = sim.simulate(0.002)
    assert not np.any(res.mdl.converter.enabled)
    mdl.set_enable(True)
    ctrl.set_enable(False)
    res = sim.simulate(0.004)
    assert not np.any(res.mdl.converter.enabled)
    ctrl.set_enable(True)
    res = sim.simulate(0.01)
    assert np.all(res.mdl.converter.enabled[res.mdl.t > 0.005])
    assert np.max(np.abs(res.mdl.machine.i_s_ab[res.mdl.t > 0.008])) > 0.1


@pytest.mark.parametrize("kind", ["cvc", "discrete", "fvc"])
@pytest.mark.parametrize("w_M", [-80.0, 80.0])
def test_sensored_flying_start(kind, w_M):
    """Synchronize a spinning shaft without an initial zero-torque current surge."""
    mdl, ctrl = spinning_drive(
        kind, w_M, flying_start=sm_control.FlyingStartCfg(T_ramp=0.02)
    )
    ctrl.set_enable(lambda t: t >= 0.005)
    ctrl.set_torque_ref(lambda t: np.sign(w_M) * 2 * (t >= 0.015))
    res = simulate(mdl, ctrl, 0.06)
    assert res.success
    enabled = res.ctrl.ref.enabled
    first = np.flatnonzero(enabled)[0]
    assert res.ctrl.t[first] > 0.005  # Prime the delayed PWM before closing gates.
    assert abs(res.ctrl.fbk.w_M[first] - w_M) < 0.1
    assert abs(res.ctrl.ref.tau_M[first]) < 1e-6
    zero_torque = (res.mdl.t > 0.006) & (res.mdl.t < 0.014)
    assert np.max(np.abs(res.mdl.machine.i_s_ab[zero_torque])) < 0.3
    steady = res.mdl.t > 0.05
    assert np.sign(w_M) * np.mean(res.mdl.machine.tau_M[steady]) > 1.5
    assert np.any(res.ctrl.ref.flying_start)
    assert not res.ctrl.ref.flying_start[-1]


@pytest.mark.parametrize("pwm,delay", [(False, 0), (True, 1), (False, 3)])
@pytest.mark.parametrize("single_update", [False, True])
def test_flying_start_reenable_and_speed_loop(pwm, delay, single_update):
    """Repeated enabling primes the delay again and avoids speed-integrator windup."""
    mdl, old_ctrl = spinning_drive(pwm=pwm, delay=delay, single_update=single_update)
    ctrl = sm_control.VectorControlSystem(
        old_ctrl.vector_ctrl,
        sm_control.SpeedController(0.015, 20),
        flying_start=sm_control.FlyingStartCfg(T_ramp=0.02),
    )
    ctrl.set_speed_ref(100)
    ctrl.set_enable(lambda t: 0.005 <= t < 0.025 or t >= 0.04)
    res = simulate(mdl, ctrl, 0.08)
    assert res.success
    blocked = (res.mdl.t > 0.027) & (res.mdl.t < 0.039)
    assert not np.any(res.mdl.converter.enabled[blocked])
    rising = np.flatnonzero(np.diff(res.ctrl.ref.enabled.astype(int)) == 1) + 1
    assert len(rising) == 2
    assert np.max(np.abs(res.ctrl.ref.tau_M[rising])) < 1e-6
    assert np.max(np.abs(res.mdl.machine.i_s_ab)) < 12
    np.testing.assert_allclose(res.ctrl.ref.w_M, 100)


def test_flying_start_rejects_sensorless():
    """The sensored feature must not pretend to acquire a sensorless spinning rotor."""
    par = model.SynchronousMachinePars(3, 3.6, 0.036, 0.051, 0.545)
    inner = sm_control.CurrentVectorController(
        par, sm_control.CurrentVectorControllerCfg(i_s_max=10, sensorless=True)
    )
    with pytest.raises(ValueError, match="sensored"):
        sm_control.VectorControlSystem(inner, flying_start=sm_control.FlyingStartCfg())


@pytest.mark.parametrize("kind", ["cvc", "fvc"])
def test_sensored_induction_flying_start(kind):
    """An unmagnetized but spinning induction motor can rebuild flux and take torque."""
    par = model.InductionMachineInvGammaPars(2, 3.7, 2.1, 0.021, 0.224)
    if kind == "cvc":
        inner = im_control.CurrentVectorController(
            par,
            im_control.CurrentVectorControllerCfg(
                psi_s_nom=1.04, i_s_max=10.6, sensorless=False
            ),
        )
    else:
        inner = im_control.FluxVectorController(
            par,
            im_control.FluxVectorControllerCfg(
                psi_s_nom=1.04, i_s_max=10.6, sensorless=False
            ),
        )
    mechanics = model.MechanicalSystem(0.015)
    mechanics.state.w_M = 80 + 0j
    mdl = model.Drive(
        model.InductionMachine(par), mechanics, model.VoltageSourceConverter(540)
    )
    ctrl = im_control.VectorControlSystem(
        inner, flying_start=im_control.FlyingStartCfg()
    )
    ctrl.set_enable(lambda t: t >= 0.005)
    ctrl.set_torque_ref(2)
    res = simulate(mdl, ctrl, 0.15)
    assert res.success
    first = np.flatnonzero(res.ctrl.ref.enabled)[0]
    assert abs(res.ctrl.fbk.w_M[first] - 80) < 0.1
    assert np.max(np.abs(res.mdl.machine.i_s_ab)) < 13
    assert np.mean(res.mdl.machine.tau_M[res.mdl.t > 0.13]) > 1.5


@pytest.mark.parametrize("machine_type", ["sm", "im"])
def test_vhz_control_can_be_disabled(machine_type):
    """Gate blocking must also bypass V/Hz power control and missing references."""
    if machine_type == "sm":
        mdl, _ = spinning_drive(w_M=0)
        par = model.SynchronousMachinePars(3, 3.6, 0.036, 0.051, 0.545)
        inner = sm_control.ObserverBasedVHzController(
            par, sm_control.ObserverBasedVHzControllerCfg(i_s_max=10)
        )
    else:
        par = model.InductionMachineInvGammaPars(2, 3.7, 2.1, 0.021, 0.224)
        mdl = model.Drive(
            model.InductionMachine(par),
            model.MechanicalSystem(0.015),
            model.VoltageSourceConverter(540),
        )
        inner = im_control.ObserverBasedVHzController(
            par, im_control.ObserverBasedVHzControllerCfg(psi_s_nom=1.04, i_s_max=10.6)
        )
    ctrl = sm_control.VHzControlSystem(inner)
    ctrl.set_enable(False)
    res = simulate(mdl, ctrl, 0.002)
    assert res.success
    assert not np.any(res.mdl.converter.enabled)
    assert np.max(np.abs(res.mdl.machine.i_s_ab)) < 1e-6


@pytest.mark.parametrize("T_ramp", [0, -1])
def test_invalid_flying_start_ramp(T_ramp):
    """Invalid ramp durations must fail at configuration rather than during takeover."""
    with pytest.raises(ValueError, match="T_ramp"):
        sm_control.FlyingStartCfg(T_ramp=T_ramp)
