"""
Tests for gate blocking.

The tests check the physical behavior of a converter with blocked gates (open circuit,
passive rectification through the diodes) and the standby operation of the control
systems (synchronization with blocked gates and the restart), with loose tolerances.

"""

from math import pi

import numpy as np
import pytest

import motulator.drive.control.im as im_control
import motulator.drive.control.sm as sm_control
from motulator.common.model import CompositeModel, Simulation, SolverCfg
from motulator.drive import model as drive_model
from motulator.grid import control as grid_control
from motulator.grid import model as grid_model


def sm_pars() -> drive_model.SynchronousMachinePars:
    """Parameters of the 2.2-kW IPMSM."""
    return drive_model.SynchronousMachinePars(
        n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545
    )


def spinning_pmsm(
    w_M: float, u_dc: float | None, u_th: float = 0.0, pwm: bool = False
) -> drive_model.Drive:
    """2.2-kW IPMSM driven by an external shaft, with a constant or capacitive bus."""
    converter = drive_model.VoltageSourceConverter(u_dc, u_th=u_th)
    dc_bus = None
    if u_dc is None:
        dc_bus = drive_model.DCBus(C_dc=2e-3, u_dc=540, converters=[converter])
    mechanics = drive_model.ExternalRotorSpeed()
    mechanics.set_external_rotor_speed(w_M)
    par = sm_pars()
    machine = drive_model.SynchronousMachine(par)
    return drive_model.Drive(machine, mechanics, converter, dc_bus=dc_bus, pwm=pwm)


def pmsm_cvc(
    sensorless: bool = True, discrete: bool = False, T_s: float = 125e-6
) -> sm_control.VectorControlSystem:
    """Current-vector control with a speed controller."""
    par = sm_pars()
    cfg = sm_control.CurrentVectorControllerCfg(
        i_s_max=6.5, sensorless=sensorless, discrete=discrete, T_s=T_s
    )
    return sm_control.VectorControlSystem(
        sm_control.CurrentVectorController(par, cfg),
        sm_control.SpeedController(J=0.015, alpha_s=2 * pi * 4),
    )


@pytest.mark.parametrize(
    "pwm, discrete", [(False, False), (True, False), (False, True)]
)
def test_open_circuit(pwm: bool, discrete: bool) -> None:
    """A spinning machine with blocked gates has no current and the EMF at terminals."""
    mdl = spinning_pmsm(w_M=100, u_dc=540, pwm=pwm)
    ctrl = pmsm_cvc(sensorless=False, discrete=discrete)
    ctrl.set_speed_ref(100)
    ctrl.set_enable(False)
    res = Simulation(mdl, ctrl, show_progress=False).simulate(t_stop=0.01)
    assert res.success
    machine, converter = res.mdl.machine, res.mdl.converter
    assert np.max(np.abs(machine.i_s_ab)) < 1e-6
    emf = 1j * 3 * 100 * 0.545 * np.exp(1j * machine.theta_m)
    np.testing.assert_allclose(converter.u_c_ab, emf, atol=1e-6)
    assert not np.any(converter.enabled)
    assert not np.any(res.ctrl.ref.enabled)
    # The current controller tracks the measured voltage in standby
    assert res.ctrl.ref.u_s[-1] == pytest.approx(res.ctrl.fbk.u_s[-1], abs=1)


# The long sampling period with a single evaluation point per period gives
# integration intervals with several diode transitions and no solution points
@pytest.mark.parametrize("N_eval, T_s", [(0, 125e-6), (1, 1e-3)])
def test_passive_rectification(N_eval: int, T_s: float) -> None:
    """Above the diode turn-on, the machine charges the DC bus through the diodes."""
    mdl = spinning_pmsm(w_M=400, u_dc=None, u_th=1.5)
    ctrl = pmsm_cvc(sensorless=False, T_s=T_s)
    ctrl.set_speed_ref(400)
    ctrl.set_enable(False)
    res = Simulation(mdl, ctrl, show_progress=False).simulate(0.01, N_eval=N_eval)
    assert res.success
    converter, dc_bus = res.mdl.converter, res.mdl.dc_bus
    assert dc_bus.u_dc[-1] > 560
    assert np.min(converter.i_dc_int) < -5
    assert np.all(converter.i_dc_int < 1e-6)
    # The AC power equals the DC power, the diode drops included
    p_ac = 1.5 * np.real(converter.u_c_ab * np.conj(converter.i_c_ab))
    p_dc = (converter.u_dc + 2 * 1.5) * converter.i_dc_int
    np.testing.assert_allclose(p_ac, p_dc, atol=1)


def test_two_blocked_converters() -> None:
    """Two converters with blocked gates on a shared DC bus have their own events."""
    converters = [drive_model.VoltageSourceConverter() for _ in range(2)]
    dc_bus = drive_model.DCBus(C_dc=2e-3, u_dc=540, converters=converters)
    models, ctrls = {}, {}
    for name, converter, w_M in zip(
        ["gen", "idle"], converters, [400, 100], strict=True
    ):
        mechanics = drive_model.ExternalRotorSpeed()
        mechanics.set_external_rotor_speed(w_M)
        machine = drive_model.SynchronousMachine(sm_pars())
        models[name] = drive_model.Drive(machine, mechanics, converter, dc_bus=dc_bus)
        ctrls[name] = pmsm_cvc(sensorless=False)
        ctrls[name].set_speed_ref(w_M)
        ctrls[name].set_enable(False)
    mdl = CompositeModel(models)
    res = Simulation(mdl, ctrls, show_progress=False).simulate(t_stop=0.01)
    assert res.success
    assert res.mdl.gen.dc_bus.u_dc[-1] > 560
    assert np.max(np.abs(res.mdl.idle.machine.i_s_ab)) < 1e-6


@pytest.mark.parametrize("system", ["drive", "grid"])
def test_discharged_dc_bus(system: str) -> None:
    """A discharged DC bus is charged with blocked gates, the PWM compensation set."""
    converter = drive_model.VoltageSourceConverter(u_th=1.5)
    dc_bus = drive_model.DCBus(C_dc=1e-3, u_dc=0, converters=[converter])
    d_err = sm_control.InverterNonlinearity(t_d=1e-6, u_th=1.5)
    if system == "drive":
        mechanics = drive_model.ExternalRotorSpeed()
        mechanics.set_external_rotor_speed(100)
        machine = drive_model.SynchronousMachine(sm_pars())
        mdl = drive_model.Drive(machine, mechanics, converter, dc_bus=dc_bus)
        ctrl = pmsm_cvc(sensorless=False)
        ctrl.pwm = sm_control.PWM(d_err=d_err)
        ctrl.set_speed_ref(100)
    else:
        mdl = grid_model.GridConverterSystem(
            converter,
            grid_model.LFilter(L_f=10e-3),
            grid_model.ThreePhaseSource(w_g=2 * pi * 50, e_g=np.sqrt(2 / 3) * 400),
            dc_bus=dc_bus,
        )
        cfg = grid_control.CurrentVectorControllerCfg(i_max=30, L=10e-3)
        ctrl = grid_control.GridConverterControlSystem(
            grid_control.CurrentVectorController(cfg), pwm=grid_control.PWM(d_err=d_err)
        )
        ctrl.set_power_ref(0)
        ctrl.set_reactive_power_ref(0)
    ctrl.set_enable(False)
    res = Simulation(mdl, ctrl, show_progress=False).simulate(t_stop=0.005)
    assert res.success
    assert res.mdl.dc_bus.u_dc[-1] > 10


@pytest.mark.parametrize("sensorless", [True, False])
def test_drive_restart(sensorless: bool) -> None:
    """The drive starts from standby and restarts after coasting with blocked gates."""
    par = sm_pars()
    mechanics = drive_model.MechanicalSystem(J=0.015)
    mdl = drive_model.Drive(
        drive_model.SynchronousMachine(par),
        mechanics,
        drive_model.VoltageSourceConverter(u_dc=540),
    )
    ctrl = pmsm_cvc(sensorless)
    ctrl.set_speed_ref(lambda t: (t > 0.02) * 2 * pi * 30)
    ctrl.set_enable(lambda t: 0.02 <= t < 0.3 or t >= 0.35)
    mechanics.set_external_load_torque(lambda t: (t > 0.2) * 5)
    res = Simulation(mdl, ctrl, show_progress=False).simulate(t_stop=0.6)
    assert res.success
    t, w_M = res.mdl.t, res.mdl.mechanics.w_M
    i_s = np.abs(res.mdl.machine.i_s_ab)
    # Standby before the start and coasting with the gates blocked, the current
    # having decayed through the diodes
    assert np.max(i_s[t < 0.02]) < 1e-6
    assert np.max(i_s[(t > 0.31) & (t < 0.35)]) < 1e-6
    assert np.interp(0.35, t, w_M) < 0.95 * np.interp(0.3, t, w_M)
    # Restart and speed tracking
    assert np.interp(0.6, t, w_M) == pytest.approx(2 * pi * 30, rel=0.05)
    assert np.max(i_s[t > 0.35]) < 1.1 * 6.5


def test_induction_machine_restart() -> None:
    """A sensored induction machine drive restarts after its flux has decayed."""
    par = drive_model.InductionMachineInvGammaPars(
        n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224
    )
    mechanics = drive_model.MechanicalSystem(J=0.015)
    mdl = drive_model.Drive(
        drive_model.InductionMachine(par),
        mechanics,
        drive_model.VoltageSourceConverter(u_dc=540),
    )
    cfg = im_control.CurrentVectorControllerCfg(
        psi_s_nom=0.9, i_s_max=7, sensorless=False
    )
    ctrl = im_control.VectorControlSystem(
        im_control.CurrentVectorController(par, cfg),
        im_control.SpeedController(J=0.015, alpha_s=2 * pi * 4),
    )
    ctrl.set_speed_ref(2 * pi * 30)
    ctrl.set_enable(lambda t: t < 0.3 or t >= 0.5)
    res = Simulation(mdl, ctrl, show_progress=False).simulate(t_stop=0.8)
    assert res.success
    t, w_M = res.mdl.t, res.mdl.mechanics.w_M
    assert np.max(np.abs(res.mdl.machine.i_s_ab[(t > 0.35) & (t < 0.5)])) < 1e-6
    assert np.abs(res.mdl.machine.psi_R_ab[np.searchsorted(t, 0.5)]) < 0.2
    assert np.interp(0.8, t, w_M) == pytest.approx(2 * pi * 30, rel=0.05)


def test_grid_converter_startup() -> None:
    """The DC bus is charged through the diodes and the PLL locks with blocked gates."""
    U_NOM, W_NOM = np.sqrt(2 / 3) * 400, 2 * pi * 50
    Z_BASE = U_NOM / (np.sqrt(2) * 14.5)
    L_BASE, R = Z_BASE / W_NOM, 0.02 * Z_BASE
    converter = grid_model.VoltageSourceConverter()
    dc_bus = grid_model.DCBus(C_dc=1e-3, u_dc=500, converters=[converter])
    ac_filter = grid_model.LCLFilter(
        L_fc=0.1 * L_BASE, L_fg=0.05 * L_BASE, C_f=0.1e-3, R_fc=R, R_fg=R
    )
    mdl = grid_model.GridConverterSystem(
        converter,
        ac_filter,
        grid_model.ThreePhaseSource(w_g=W_NOM, e_g=U_NOM),
        dc_bus=dc_bus,
    )
    cfg = grid_control.CurrentVectorControllerCfg(i_max=30, L=0.15 * L_BASE)
    dc_ctrl = grid_control.DCBusVoltageController(
        C_dc=1e-3, alpha_dc=2 * pi * 30, p_max=10e3
    )
    ctrl = grid_control.GridConverterControlSystem(
        grid_control.CurrentVectorController(cfg), dc_ctrl
    )
    ctrl.set_dc_bus_voltage_ref(650)
    ctrl.set_reactive_power_ref(0)
    ctrl.set_enable(lambda t: t >= 0.04)
    cfg_solver = SolverCfg(rtol=1e-5, atol=1e-8)
    res = Simulation(mdl, ctrl, show_progress=False, cfg=cfg_solver).simulate(0.12)
    assert res.success
    t = res.mdl.t
    # Charging through the diodes at least to the peak line-to-line voltage (the
    # filter rings, so the DC-bus voltage overshoots), then no current
    u_dc = np.interp(0.039, t, res.mdl.dc_bus.u_dc)
    assert np.sqrt(2) * 400 < u_dc < 1.2 * np.sqrt(2) * 400
    assert np.max(np.abs(res.mdl.ac_filter.i_c_ab[(t > 0.035) & (t < 0.04)])) < 1e-6
    # The PLL has locked before the gates are released
    k = np.searchsorted(res.ctrl.t, 0.04)
    theta_g = np.angle(np.interp(res.ctrl.t[k], t, res.mdl.ac_filter.e_g_ab))
    assert abs(np.angle(np.exp(1j * (res.ctrl.fbk.theta_c[k] - theta_g)))) < pi / 180
    # DC-bus voltage control after the start
    assert res.mdl.dc_bus.u_dc[-1] == pytest.approx(650, rel=0.01)


def test_unsupported_controllers() -> None:
    """Controllers without a standby output reject gate blocking."""
    cfg = grid_control.PowerSynchronizationControllerCfg(i_max=20)
    ctrl = grid_control.GridConverterControlSystem(
        grid_control.PowerSynchronizationController(cfg)
    )
    ctrl.set_power_ref(0)
    ctrl.set_ac_voltage_ref(np.sqrt(2 / 3) * 400)
    ctrl.set_enable(False)
    mdl = grid_model.GridConverterSystem(
        grid_model.VoltageSourceConverter(u_dc=650),
        grid_model.LFilter(L_f=10e-3),
        grid_model.ThreePhaseSource(w_g=2 * pi * 50, e_g=np.sqrt(2 / 3) * 400),
    )
    with pytest.raises(NotImplementedError):
        Simulation(mdl, ctrl, show_progress=False).simulate(0.001)
