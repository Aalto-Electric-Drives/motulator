"""
Closed-loop tests for machine drives.

The tests check physical properties of the closed-loop response (speed tracking, torque
balance, and speed estimation) with loose tolerances. They catch broken interfaces and
sign errors but tolerate retuning of the controllers.

"""

from math import pi

import numpy as np
import pytest

import motulator.drive.control.im as im_control
import motulator.drive.control.sm as sm_control
from motulator.drive import model

T_STOP = 0.6
T_LOAD = 0.3


def ipmsm_drive(pwm: bool) -> model.Drive:
    """2.2-kW IPMSM drive."""
    par = model.SynchronousMachinePars(
        n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545
    )
    return model.Drive(
        model.SynchronousMachine(par),
        model.MechanicalSystem(J=0.015),
        model.VoltageSourceConverter(u_dc=540),
        pwm=pwm,
    )


def im_drive() -> model.Drive:
    """2.2-kW induction machine drive."""
    par = model.InductionMachineInvGammaPars(
        n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224
    )
    return model.Drive(
        model.InductionMachine(par),
        model.MechanicalSystem(J=0.015),
        model.VoltageSourceConverter(u_dc=540),
    )


def ipmsm_fvc(sensorless: bool) -> sm_control.VectorControlSystem:
    par = sm_control.SynchronousMachinePars(
        n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545
    )
    cfg = sm_control.FluxVectorControllerCfg(i_s_max=6.5, sensorless=sensorless)
    return sm_control.VectorControlSystem(
        sm_control.FluxVectorController(par, cfg),
        sm_control.SpeedController(J=0.015, alpha_s=2 * pi * 4),
    )


def ipmsm_cvc(sensorless: bool) -> sm_control.VectorControlSystem:
    par = sm_control.SynchronousMachinePars(
        n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545
    )
    cfg = sm_control.CurrentVectorControllerCfg(i_s_max=6.5, sensorless=sensorless)
    return sm_control.VectorControlSystem(
        sm_control.CurrentVectorController(par, cfg),
        sm_control.SpeedController(J=0.015, alpha_s=2 * pi * 4),
    )


def ipmsm_cvc_discrete() -> sm_control.VectorControlSystem:
    par = sm_control.SynchronousMachinePars(
        n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545
    )
    cfg = sm_control.CurrentVectorControllerCfg(i_s_max=6.5, discrete=True)
    return sm_control.VectorControlSystem(
        sm_control.CurrentVectorController(par, cfg),
        sm_control.SpeedController(J=0.015, alpha_s=2 * pi * 4),
        sm_control.PWM(k_comp=0, average=False),
    )


def ipmsm_signal_inj() -> sm_control.VectorControlSystem:
    par = sm_control.SynchronousMachinePars(
        n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545
    )
    # Signal injection is always sensorless, regardless of the cfg
    cfg = sm_control.CurrentVectorControllerCfg(
        i_s_max=6.5, alpha_o=2 * pi * 40, sensorless=False
    )
    return sm_control.VectorControlSystem(
        sm_control.SignalInjectionController(par, cfg),
        sm_control.SpeedController(J=0.015, alpha_s=2 * pi * 4),
    )


def im_par() -> im_control.InductionMachineInvGammaPars:
    return im_control.InductionMachineInvGammaPars(
        n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224
    )


def im_cvc() -> im_control.VectorControlSystem:
    cfg = im_control.CurrentVectorControllerCfg(
        psi_s_nom=1.04, i_s_max=10.6, sensorless=True
    )
    return im_control.VectorControlSystem(
        im_control.CurrentVectorController(im_par(), cfg),
        im_control.SpeedController(J=0.015, alpha_s=2 * pi * 4),
    )


def im_fvc() -> im_control.VectorControlSystem:
    cfg = im_control.FluxVectorControllerCfg(
        psi_s_nom=1.04, i_s_max=10.6, sensorless=True
    )
    return im_control.VectorControlSystem(
        im_control.FluxVectorController(im_par(), cfg),
        im_control.SpeedController(J=0.015, alpha_s=2 * pi * 4),
    )


CASES = {
    "ipmsm_fvc_sensorless": (lambda: ipmsm_drive(False), lambda: ipmsm_fvc(True)),
    "ipmsm_fvc_sensored": (lambda: ipmsm_drive(False), lambda: ipmsm_fvc(False)),
    "ipmsm_fvc_pwm": (lambda: ipmsm_drive(True), lambda: ipmsm_fvc(True)),
    "ipmsm_cvc_sensorless": (lambda: ipmsm_drive(False), lambda: ipmsm_cvc(True)),
    "ipmsm_cvc_sensored": (lambda: ipmsm_drive(False), lambda: ipmsm_cvc(False)),
    "ipmsm_cvc_discrete": (lambda: ipmsm_drive(False), ipmsm_cvc_discrete),
    "ipmsm_signal_inj": (lambda: ipmsm_drive(False), ipmsm_signal_inj),
    "im_cvc_sensorless": (im_drive, im_cvc),
    "im_fvc_sensorless": (im_drive, im_fvc),
}


@pytest.mark.parametrize("case", CASES)
def test_speed_control(case: str) -> None:
    """Speed follows the reference, and the torque balances the load in steady state."""
    make_mdl, make_ctrl = CASES[case]
    mdl, ctrl = make_mdl(), make_ctrl()
    w_M_ref, tau_L = 50.0, 7.0  # About half of the nominal speed and torque

    ctrl.set_speed_ref(lambda t: (t > 0.05) * w_M_ref)
    mdl.mechanics.set_external_load_torque(lambda t: (t > T_LOAD) * tau_L)
    res = model.Simulation(mdl, ctrl, show_progress=False).simulate(t_stop=T_STOP)

    # Averages over the last 50 ms, which remove the ripple caused by PWM
    end = res.mdl.t > T_STOP - 0.05
    w_M = np.mean(res.mdl.machine.w_M[end])
    tau_M = np.mean(res.mdl.machine.tau_M[end])
    ctrl_end = res.ctrl.t > T_STOP - 0.05
    w_M_est = np.mean(res.ctrl.fbk.w_M[ctrl_end])
    tau_M_ref = np.mean(res.ctrl.ref.tau_M[ctrl_end])

    assert w_M == pytest.approx(w_M_ref, rel=0.02)
    assert tau_M == pytest.approx(tau_L, abs=0.05 * 14)
    assert w_M_est == pytest.approx(w_M, rel=0.02)
    # The speed controller would hide errors in the torque control loop, so check the
    # torque reference too
    assert tau_M_ref == pytest.approx(tau_M, abs=0.05 * 14)


def test_observer_based_vhz() -> None:
    """Observer-based V/Hz control of an induction machine reaches the speed."""
    cfg = im_control.ObserverBasedVHzControllerCfg(psi_s_nom=1.04, i_s_max=10.6)
    ctrl = im_control.VHzControlSystem(
        im_control.ObserverBasedVHzController(im_par(), cfg), slew_rate=2 * pi * 120
    )
    mdl = im_drive()
    w_M_ref, tau_L = 50.0, 7.0

    ctrl.set_speed_ref(lambda t: (t > 0.05) * w_M_ref)
    mdl.mechanics.set_external_load_torque(lambda t: (t > T_LOAD) * tau_L)
    res = model.Simulation(mdl, ctrl, show_progress=False).simulate(t_stop=T_STOP)

    end = res.mdl.t > T_STOP - 0.05
    # V/Hz control has no speed controller, so the slip is allowed for
    assert np.mean(res.mdl.machine.w_M[end]) == pytest.approx(w_M_ref, rel=0.05)
    assert np.mean(res.mdl.machine.tau_M[end]) == pytest.approx(tau_L, abs=0.05 * 14)


def test_pm_flux_adaptation_keeps_parameters() -> None:
    """PM-flux adaptation does not modify the parameter object given by the user."""
    par = sm_control.SynchronousMachinePars(
        n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545
    )
    cfg = sm_control.CurrentVectorControllerCfg(i_s_max=6.5, k_f=lambda w_m: 1.0)
    ctrl = sm_control.VectorControlSystem(
        sm_control.CurrentVectorController(par, cfg),
        sm_control.SpeedController(J=0.015, alpha_s=2 * pi * 4),
    )
    # The same parameter object is used in the system model
    mdl = model.Drive(
        model.SynchronousMachine(par),
        model.MechanicalSystem(J=0.015),
        model.VoltageSourceConverter(u_dc=540),
    )
    ctrl.set_speed_ref(lambda t: (t > 0.05) * 50)
    res = model.Simulation(mdl, ctrl, show_progress=False).simulate(t_stop=0.2)

    assert par.psi_f == 0.545
    assert res.ctrl.fbk.psi_f[-1] != 0.545  # The estimate is adapted


@pytest.mark.parametrize("w", [0, 2000, -6000])
def test_discrete_current_control(w: float) -> None:
    """Designed dynamics in the linear range and recovery from voltage limitation."""
    T_s, L, alpha_c = 200e-6, 0.01, 2 * pi * 500
    par = sm_control.SynchronousMachinePars(n_p=1, R_s=0, L_d=L, L_q=L, psi_f=0)
    ctrl = sm_control.DiscreteCurrentController(par, alpha_c, T_s)
    ctrl.update(T_s, w)  # Initial angular speed of the coordinates
    pwm = sm_control.PWM(k_comp=0, average=False)

    def run(i_ref: float, n: int, u_dc: float, psi_ab: complex, theta: float):
        # Exact hold-equivalent plant for R_s = 0 in stationary coordinates
        i = np.zeros(n, complex)
        for k in range(n):
            u_ab = pwm.get_realized_voltage(psi_ab / L, u_dc)
            i[k] = np.exp(-1j * theta) * psi_ab / L
            u_ref = ctrl.compute_output(i_ref, i[k], np.exp(-1j * theta) * u_ab)
            pwm(T_s, np.exp(1j * theta) * u_ref, u_dc, w)
            ctrl.update(T_s, w)
            psi_ab, theta = psi_ab + T_s * u_ab, theta + w * T_s
        return i, psi_ab, theta

    # Step in the linear range, compared with (1 - beta)/(z*(z - beta))
    i, psi_ab, theta = run(1, 50, 1000, 0j, 0)
    beta = np.exp(-alpha_c * T_s)
    k = np.arange(50)
    assert i == pytest.approx(np.where(k > 1, 1 - beta ** (k - 1.0), 0), abs=1e-9)
    # Unreachable reference under voltage limitation, then back to zero
    _, psi_ab, theta = run(50, 1000, 173, psi_ab, theta)
    i, _, _ = run(0, 500, 173, psi_ab, theta)
    assert abs(ctrl.u_i) < 1e-3
    assert abs(i[-1]) < 1e-3
