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
from motulator.common.control import ComplexPIController, DiscreteComplexController
from motulator.common.model import CompositeModel, Simulation, SolverCfg
from motulator.common.utils import abc2complex
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
    res = Simulation(mdl, ctrl, show_progress=False).simulate(t_stop=0.05)
    assert res.success
    machine, converter = res.mdl.machine, res.mdl.converter
    assert np.max(np.abs(machine.i_s_ab)) < 1e-6
    emf = 1j * 3 * 100 * 0.545 * np.exp(1j * machine.theta_m)
    np.testing.assert_allclose(converter.u_c_ab, emf, atol=1e-6)
    assert not np.any(converter.enabled)
    assert not np.any(res.ctrl.ref.enabled)
    # The converter voltage is not measured: the feedback voltage is the open-circuit
    # voltage of the observer model, which converges to the EMF as the speed estimate
    # does, and the current controller tracks it in standby
    assert abs(res.ctrl.fbk.u_s[-1]) == pytest.approx(3 * 100 * 0.545, rel=1e-3)
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


def test_standby_feedforward_current() -> None:
    """With blocked gates, the PWM compensation uses the measured currents, so it
    vanishes in the standby duty ratios once the current has decayed."""

    def sign(i: np.ndarray) -> np.ndarray:
        return np.tanh(i / 0.1)  # Smooth, insensitive to numerical residuals

    mdl = drive_model.Drive(
        drive_model.SynchronousMachine(sm_pars()),
        drive_model.MechanicalSystem(J=0.015),
        drive_model.VoltageSourceConverter(u_dc=540, t_d=2e-6, sign=sign),
    )
    ctrl = pmsm_cvc(sensorless=False)
    d_err = sm_control.InverterNonlinearity(t_d=2e-6, sign=sign)
    ctrl.pwm = sm_control.PWM(d_err=d_err)
    ctrl.set_speed_ref(2 * pi * 15)
    ctrl.set_enable(lambda t: t < 0.1)
    mdl.mechanics.set_external_load_torque(5)
    res = Simulation(mdl, ctrl, show_progress=False).simulate(t_stop=0.11)
    assert res.success
    standby = res.ctrl.t > 0.105
    ref, fbk = res.ctrl.ref, res.ctrl.fbk
    assert np.max(np.abs(fbk.i_s[standby])) < 1e-6
    # The duty ratios equal those of the limited voltage reference
    u_c_ab = 540 * abc2complex(ref.d_abc[standby].T)
    u_c_ab_lim = np.exp(1j * fbk.theta_c[standby]) * ref.u_s_lim[standby]
    assert u_c_ab == pytest.approx(u_c_ab_lim, abs=1e-6)


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
    # The flying start: the standby output and the first output after the release are
    # the zero vector, and the catch at the second step gives the speed. The speed
    # estimate holds while the current decays through the diodes.
    if sensorless:
        k = np.searchsorted(res.ctrl.t, 0.3)
        w_M_hold = res.ctrl.fbk.w_M[k - 1 : k + 16]
        assert w_M_hold == pytest.approx(w_M_hold[0], rel=0.01)
        k = np.searchsorted(res.ctrl.t, 0.35)
        assert res.ctrl.ref.d_abc[k - 1 : k + 1] == pytest.approx(0.5)
        w_M_est = res.ctrl.fbk.w_M[k + 1]
        assert w_M_est == pytest.approx(np.interp(res.ctrl.t[k + 1], t, w_M), rel=0.02)
    # Restart and speed tracking
    assert np.interp(0.6, t, w_M) == pytest.approx(2 * pi * 30, rel=0.05)
    assert np.max(i_s[t > 0.35]) < 1.1 * 6.5


@pytest.mark.parametrize("ctrl_type", ["cvc", "discrete_cvc", "fvc"])
def test_flying_start_zero_torque(ctrl_type: str) -> None:
    """A sensorless drive released with zero torque after a decelerating coast draws no
    current beyond the short-circuit pulse of the zero-voltage periods."""
    par = sm_pars()
    mechanics = drive_model.MechanicalSystem(J=0.015)
    mdl = drive_model.Drive(
        drive_model.SynchronousMachine(par),
        mechanics,
        drive_model.VoltageSourceConverter(u_dc=540),
    )
    if ctrl_type == "fvc":
        vector_ctrl = sm_control.FluxVectorController(
            par, sm_control.FluxVectorControllerCfg(i_s_max=6.5)
        )
    else:
        cfg = sm_control.CurrentVectorControllerCfg(
            i_s_max=6.5, discrete=ctrl_type == "discrete_cvc"
        )
        vector_ctrl = sm_control.CurrentVectorController(par, cfg)
    ctrl = sm_control.VectorControlSystem(vector_ctrl)
    ctrl.set_torque_ref(lambda t: (t < 0.3) * 8.0)
    ctrl.set_enable(lambda t: t < 0.3 or t >= 0.43)
    mechanics.set_external_load_torque(lambda t: (t > 0.3) * 10.0)
    res = Simulation(mdl, ctrl, show_progress=False).simulate(t_stop=0.48)
    assert res.success
    t, w_M = res.mdl.t, res.mdl.mechanics.w_M
    # The machine decelerates while coasting, so the open-circuit model of the
    # observer and the integral state of the current controller are off at the release
    w_M_rel = np.interp(0.43, t, w_M)
    assert w_M_rel < 0.5 * np.interp(0.3, t, w_M)
    # The catch at the second sampling instant after the release
    k = np.searchsorted(res.ctrl.t, 0.43) + 1
    assert np.flatnonzero(res.ctrl.fbk.caught).tolist() == [k]
    assert res.ctrl.fbk.w_M[k] == pytest.approx(w_M_rel, rel=0.02)
    theta_m = np.interp(res.ctrl.t[k], t, res.mdl.machine.theta_m)
    assert abs(np.angle(np.exp(1j * (res.ctrl.fbk.theta_m[k] - theta_m)))) < 0.05
    # The back-EMF drives the current during the two zero-voltage periods, after which
    # the control continues with zero torque from the initialized states
    i_sc = 2 * 3 * w_M_rel * 0.545 * 125e-6 / 0.051
    assert np.max(np.abs(res.mdl.machine.i_s_ab[t > 0.43])) < 1.2 * i_sc


def test_catch_with_residual_current() -> None:
    """The flying start leaves the estimates if the current has not decayed, since
    the resistive decay of the current would be taken for the back-EMF."""
    par = sm_pars()
    machine = drive_model.SynchronousMachine(par)
    machine.state.psi_s_dq = complex(par.psi_s_dq(5 + 0j))  # Residual d-axis current
    mechanics = drive_model.ExternalRotorSpeed()
    mechanics.set_external_rotor_speed(0)
    mdl = drive_model.Drive(
        machine, mechanics, drive_model.VoltageSourceConverter(u_dc=540)
    )
    vector_ctrl = sm_control.CurrentVectorController(
        par, sm_control.CurrentVectorControllerCfg(i_s_max=6.5)
    )
    vector_ctrl.observer.flux_observer.psi_s = machine.state.psi_s_dq
    ctrl = sm_control.VectorControlSystem(vector_ctrl)
    ctrl.set_torque_ref(0)
    ctrl.set_enable(lambda t: t >= 125e-6)  # Blocked for one sampling period
    res = Simulation(mdl, ctrl, show_progress=False).simulate(t_stop=1e-3)
    assert res.success
    assert abs(res.ctrl.fbk.i_s[1]) > 3  # The current has not decayed at the release
    assert np.max(np.abs(res.ctrl.fbk.w_M)) < 0.1
    assert np.max(np.abs(res.ctrl.fbk.theta_m)) < 0.01


def test_catch_without_magnets() -> None:
    """The flying start leaves the estimates of a machine without magnets."""
    par = drive_model.SynchronousMachinePars(
        n_p=2, R_s=0.54, L_d=41.5e-3, L_q=6.2e-3, psi_f=0
    )
    ctrl = sm_control.CurrentVectorController(
        par, sm_control.CurrentVectorControllerCfg(i_s_max=10, psi_s_min=0.5)
    )
    observer = ctrl.observer
    observer.w_M = 100
    observer.flux_observer.theta_m = 1
    assert not observer.catch(delta_i_s_ab=1j, i_s_ab=1j, T_s=125e-6)
    assert observer.w_M == 100
    assert observer.flux_observer.theta_m == 1


@pytest.mark.parametrize("discrete", [False, True])
def test_controller_preset(discrete: bool) -> None:
    """The preset integral state (used after the catch) is the steady state, in which
    the output with zero reference and feedback holds."""
    T_s, u, w_c = 125e-6, 100j, 300.0
    ctrl: ComplexPIController | DiscreteComplexController
    if discrete:
        ctrl = DiscreteComplexController(alpha_c=2 * pi * 200, T_s=T_s)
    else:
        ctrl = ComplexPIController(k_p=2500, k_i=1.6e6, k_t=1250)
    ctrl.preset(u, w_c)
    if isinstance(ctrl, DiscreteComplexController):
        ctrl.u_lim = u  # The steady state includes the previous output
    for _ in range(3):
        assert ctrl.compute_output(0j, 0j) == pytest.approx(u)
        ctrl.update(T_s, u, w_c)


@pytest.mark.parametrize("sensorless", [False, True])
@pytest.mark.parametrize("ctrl_type", ["cvc", "fvc"])
def test_induction_machine_restart(ctrl_type: str, sensorless: bool) -> None:
    """An induction machine drive starts and restarts after its flux has decayed
    within the current limit, the flux estimate decaying with it (the speed estimate
    of a sensorless drive holds, which suffices here, since the unloaded machine keeps
    its speed)."""
    par = drive_model.InductionMachineInvGammaPars(
        n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224
    )
    mechanics = drive_model.MechanicalSystem(J=0.015)
    mdl = drive_model.Drive(
        drive_model.InductionMachine(par),
        mechanics,
        drive_model.VoltageSourceConverter(u_dc=540),
    )
    if ctrl_type == "fvc":
        vector_ctrl = im_control.FluxVectorController(
            par,
            im_control.FluxVectorControllerCfg(
                psi_s_nom=0.9, i_s_max=7, sensorless=sensorless
            ),
        )
    else:
        cfg = im_control.CurrentVectorControllerCfg(
            psi_s_nom=0.9, i_s_max=7, sensorless=sensorless
        )
        vector_ctrl = im_control.CurrentVectorController(par, cfg)
    ctrl = im_control.VectorControlSystem(
        vector_ctrl, im_control.SpeedController(J=0.015, alpha_s=2 * pi * 4)
    )
    ctrl.set_speed_ref(2 * pi * 30)
    ctrl.set_enable(lambda t: t < 0.3 or t >= 0.5)
    res = Simulation(mdl, ctrl, show_progress=False).simulate(t_stop=0.8)
    assert res.success
    t, w_M = res.mdl.t, res.mdl.mechanics.w_M
    assert np.max(np.abs(res.mdl.machine.i_s_ab)) < 1.05 * 7
    assert np.max(np.abs(res.mdl.machine.i_s_ab[(t > 0.35) & (t < 0.5)])) < 1e-6
    psi_R = np.abs(res.mdl.machine.psi_R_ab[np.searchsorted(t, 0.5)])
    assert psi_R < 0.2
    k = np.searchsorted(res.ctrl.t, 0.5) - 1
    assert np.abs(res.ctrl.fbk.psi_R[k]) == pytest.approx(psi_R, abs=0.02)
    assert np.interp(0.8, t, w_M) == pytest.approx(2 * pi * 30, rel=0.05)


def test_signal_injection_start() -> None:
    """A signal-injection drive starts from standby."""
    par = sm_pars()
    mdl = drive_model.Drive(
        drive_model.SynchronousMachine(par),
        drive_model.MechanicalSystem(J=0.015),
        drive_model.VoltageSourceConverter(u_dc=540),
    )
    cfg = sm_control.CurrentVectorControllerCfg(i_s_max=6.5, alpha_o=2 * pi * 40)
    ctrl = sm_control.VectorControlSystem(
        sm_control.SignalInjectionController(par, cfg),
        sm_control.SpeedController(J=0.015, alpha_s=2 * pi * 4),
    )
    ctrl.set_speed_ref(lambda t: (t > 0.02) * 20)
    ctrl.set_enable(lambda t: t >= 0.02)
    res = Simulation(mdl, ctrl, show_progress=False).simulate(t_stop=0.4)
    assert res.success
    assert res.mdl.mechanics.w_M[-1] == pytest.approx(20, rel=0.05)


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
