"""
Closed-loop tests for grid converters.

The tests check that the active and reactive powers follow their references in steady
state, with loose tolerances.

"""

from dataclasses import replace

import numpy as np
import pytest

from motulator.common.model import SimulationResults
from motulator.grid import control, model
from motulator.grid.control._gfm_observer import Observer
from motulator.grid.control._gfm_sequence_observer import SequenceObserver

U_NOM = np.sqrt(2 / 3) * 400  # Peak phase voltage
W_NOM = 2 * np.pi * 50
Z_BASE = U_NOM / (np.sqrt(2) * 14.5)  # Base impedance of a 10-kVA converter
L_BASE = Z_BASE / W_NOM


def grid_powers(res: SimulationResults, t_start: float) -> tuple[float, float]:
    """Average grid powers at the grid voltage source after `t_start`."""
    end = res.mdl.t > t_start
    s_g = 1.5 * res.mdl.ac_filter.e_g_ab[end] * np.conj(res.mdl.ac_filter.i_g_ab[end])
    return float(np.mean(s_g.real)), float(np.mean(s_g.imag))


def assert_controller_coordinates(res: SimulationResults) -> None:
    """The post-processed controller signals are in coordinates at the angle theta_c."""
    i_c_ab = res.mdl.ac_filter.i_c_ab
    i_c_ab_meas = np.interp(res.ctrl.t, res.mdl.t, i_c_ab.real) + 1j * np.interp(
        res.ctrl.t, res.mdl.t, i_c_ab.imag
    )
    i_c_ab_ctrl = np.exp(1j * res.ctrl.fbk.theta_c) * res.ctrl.fbk.i_c
    assert i_c_ab_ctrl == pytest.approx(i_c_ab_meas, abs=1e-3 * np.max(np.abs(i_c_ab)))


def system(L_g: float = 0.0, u_dc: float | None = 650) -> model.GridConverterSystem:
    """Grid converter system, fed by a DC bus if `u_dc` is None."""
    converter = model.VoltageSourceConverter(u_dc=u_dc)
    dc_bus = None if u_dc else model.DCBus(C_dc=1e-3, u_dc=650, converters=[converter])
    return model.GridConverterSystem(
        converter,
        model.LFilter(L_f=0.2 * L_BASE, L_g=L_g),
        model.ThreePhaseSource(w_g=W_NOM, e_g=U_NOM),
        dc_bus=dc_bus,
    )


def test_grid_following() -> None:
    """Grid-following control tracks the active and reactive power references."""
    cfg = control.CurrentVectorControllerCfg(i_max=30, L=0.2 * L_BASE)
    ctrl = control.GridConverterControlSystem(control.CurrentVectorController(cfg))
    p_g_ref, q_g_ref = 5e3, 4e3
    ctrl.set_power_ref(lambda t: (t > 0.02) * p_g_ref)
    ctrl.set_reactive_power_ref(lambda t: (t > 0.04) * q_g_ref)
    with pytest.raises(ValueError):  # References of other modes are rejected
        ctrl.set_ac_voltage_ref(U_NOM)
    with pytest.raises(ValueError):
        ctrl.set_dc_bus_voltage_ref(650)
    dc_ctrl = control.DCBusVoltageController(C_dc=1e-3, alpha_dc=2 * np.pi * 30)
    with pytest.raises(ValueError):  # The DC-bus voltage controller gives p_g_ref
        control.GridConverterControlSystem(ctrl.inner_ctrl, dc_ctrl).set_power_ref(0)

    res = model.Simulation(system(), ctrl, show_progress=False).simulate(t_stop=0.1)

    p_g, q_g = grid_powers(res, 0.08)
    assert p_g == pytest.approx(p_g_ref, abs=0.02 * 10e3)
    assert q_g == pytest.approx(q_g_ref, abs=0.02 * 10e3)
    assert_controller_coordinates(res)


def test_dual_voltage_forming() -> None:
    """Dual-voltage-forming control tracks the DC-bus voltage and feeds the power."""
    u_dc_ref, p_ext = 650.0, -5e3
    converter = model.VoltageSourceConverter()
    dc_bus = model.DCBus(C_dc=1e-3, u_dc=u_dc_ref, converters=[converter])
    dc_bus.set_external_dc_current(lambda t: (t > 0.05) * p_ext / u_dc_ref)
    R_f = 0.05 * Z_BASE
    mdl = model.GridConverterSystem(
        converter,
        model.LFilter(L_f=0.15 * L_BASE, R_f=R_f, L_g=0.6 * L_BASE),
        model.ThreePhaseSource(w_g=W_NOM, e_g=U_NOM),
        dc_bus=dc_bus,
    )
    cfg = control.DualVoltageFormingControllerCfg(
        i_max=30, L=0.3 * L_BASE, C_dc=1e-3, R=R_f, u_nom=U_NOM, w_nom=W_NOM
    )
    inner_ctrl = control.DualVoltageFormingController(cfg)
    with pytest.raises(ValueError):  # The inner controller controls the DC-bus voltage
        dc_ctrl = control.DCBusVoltageController(C_dc=1e-3, alpha_dc=2 * np.pi * 30)
        control.GridConverterControlSystem(inner_ctrl, dc_ctrl)
    ctrl = control.GridConverterControlSystem(inner_ctrl)
    with pytest.raises(ValueError):  # Takes the DC-bus voltage reference
        ctrl.set_power_ref(0)
    ctrl.set_ac_voltage_ref(U_NOM)
    ctrl.set_dc_bus_voltage_ref(lambda t: u_dc_ref + (t > 0.3) * 50)

    res = model.Simulation(mdl, ctrl, show_progress=False).simulate(t_stop=0.6)
    assert res.success

    t, u_dc = res.mdl.t, res.mdl.dc_bus.u_dc
    mid, end = (t > 0.25) & (t < 0.3), t > 0.55
    p_g, _ = grid_powers(res, 0.55)
    # The external current is constant, so the external power scales with the voltage
    p_ext_end = p_ext / u_dc_ref * np.mean(u_dc[end])
    assert np.mean(u_dc[mid]) == pytest.approx(u_dc_ref, rel=0.01)
    assert np.mean(u_dc[end]) == pytest.approx(u_dc_ref + 50, rel=0.01)
    assert p_g + 1.5 * R_f * np.mean(np.abs(res.mdl.ac_filter.i_c_ab[end]) ** 2) == (
        pytest.approx(p_ext_end, rel=0.03)
    )
    assert np.min(u_dc) > 0.85 * u_dc_ref  # Dip at the power step
    # The estimated external power, excluding the losses, matches the actual one
    fbk, ctrl_end = res.ctrl.fbk, res.ctrl.t > 0.55
    p_ext_est = fbk.p_dis + fbk.p_g + 1.5 * R_f * np.abs(fbk.i_c) ** 2
    assert np.mean(p_ext_est[ctrl_end]) == pytest.approx(p_ext_end, rel=0.03)
    assert_controller_coordinates(res)


@pytest.mark.parametrize("method", ["observer", "psc"])
def test_grid_forming(method: str) -> None:
    """Grid-forming control tracks the active power reference."""
    if method == "observer":
        cfg = control.ObserverBasedGridFormingControllerCfg(
            i_max=30, L=0.35 * L_BASE, R_a=0.2 * Z_BASE, u_nom=U_NOM, w_nom=W_NOM
        )
        inner_ctrl = control.ObserverBasedGridFormingController(cfg)
    else:
        cfg = control.PowerSynchronizationControllerCfg(
            i_max=30, u_nom=U_NOM, w_nom=W_NOM, R_a=0.2 * Z_BASE
        )
        inner_ctrl = control.PowerSynchronizationController(cfg)
    ctrl = control.GridConverterControlSystem(inner_ctrl)
    p_g_ref = 5e3
    ctrl.set_ac_voltage_ref(U_NOM)
    ctrl.set_power_ref(lambda t: (t > 0.05) * p_g_ref)
    with pytest.raises(ValueError):  # References of other modes are rejected
        ctrl.set_reactive_power_ref(0)

    mdl = system(L_g=0.74 * L_BASE)
    res = model.Simulation(mdl, ctrl, show_progress=False).simulate(t_stop=0.4)

    p_g, _ = grid_powers(res, 0.35)
    assert p_g == pytest.approx(p_g_ref, abs=0.02 * 10e3)
    assert_controller_coordinates(res)
    if method == "psc":
        return
    # The post-processed signals are in the same coordinates, aligned with v_c
    end = res.ctrl.t > 0.35
    assert np.abs(res.ctrl.fbk.v_c.imag).max() < 1e-9 * U_NOM
    u_c_err = np.mean(res.ctrl.fbk.u_c[end] - res.ctrl.ref.u_c[end])
    assert abs(u_c_err) < 0.02 * U_NOM


@pytest.mark.parametrize("u_dc", [650, 520], ids=["linear", "overmodulation"])
def test_grid_forming_unbalanced(u_dc: float) -> None:
    """Sequence-separated grid-forming control suppresses the negative sequence and
    tracks the power reference, also when the PWM saturates."""
    cfg = control.SequenceObserverBasedGridFormingControllerCfg(
        i_max=30, i_d_max=25, L=0.35 * L_BASE, u_nom=U_NOM, w_nom=W_NOM, T_s=250e-6
    )
    inner_ctrl = control.SequenceObserverBasedGridFormingController(cfg)
    ctrl = control.GridConverterControlSystem(inner_ctrl)
    p_g_ref = 5e3
    ctrl.set_ac_voltage_ref(U_NOM)
    ctrl.set_power_ref(lambda t: (t > 0.05) * p_g_ref)

    e_g_neg = 0.1 * U_NOM
    mdl = system(L_g=0.74 * L_BASE, u_dc=u_dc)
    mdl.ac_source.e_g_neg = e_g_neg
    res = model.Simulation(mdl, ctrl, show_progress=False).simulate(t_stop=0.4)

    # The observer uses the realized voltage, which avoids a power error in saturation
    p_g, _ = grid_powers(res, 0.36)
    assert p_g == pytest.approx(p_g_ref, abs=0.01 * 10e3)
    assert_controller_coordinates(res)
    # Without the negative-sequence converter voltage, the negative-sequence grid
    # voltage drives the negative-sequence current through the total inductance
    t = 0.36 + 1e-4 * np.arange(400)  # Two fundamental periods, evenly sampled
    i_c_ab = np.interp(t, res.mdl.t, res.mdl.ac_filter.i_c_ab)
    i_n = np.mean(i_c_ab * np.exp(1j * W_NOM * t))
    assert i_n == pytest.approx(-1j * e_g_neg / (W_NOM * 0.94 * L_BASE), rel=0.02)


def exact_model(
    u_gp: complex, u_gn: complex, L: float, t: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Converter voltages held over the sampling periods and the currents of the exact
    hold-equivalent model, with the grid-voltage sequences `u_gp` and `u_gn`."""
    T_s = t[1] - t[0]
    u_c_ab = 0.9 * U_NOM * np.exp(1j * (W_NOM * t + 0.3))
    exp_p = np.exp(1j * W_NOM * t)
    exp_t = np.exp(1j * W_NOM * T_s)
    e_g_int = u_gp * exp_p * (exp_t - 1) - u_gn / exp_p * (1 / exp_t - 1)
    di_c_ab = (T_s * u_c_ab - e_g_int / (1j * W_NOM)) / L
    return u_c_ab, np.concatenate(([0j], np.cumsum(di_c_ab)[:-1]))


def test_observers() -> None:
    """The disturbance observers have the designed estimation-error dynamics, and the
    sequence separation is exact in the periodic steady state."""
    T_s, L, alpha_o = 125e-6, 0.2 * L_BASE, W_NOM
    t = T_s * np.arange(10)

    # The error of the balanced observer decays at the pole exp(-alpha_o*T_s)
    u_g, err = 0.8 * U_NOM, []
    observer = Observer(U_NOM, W_NOM, L, alpha_o, T_s)
    for u_c_ab, i_c_ab in zip(*exact_model(u_g, 0, L, t), strict=True):
        out = observer.compute_output(u_c_ab, i_c_ab)
        observer.update(T_s, out)
        err.append(u_g - out.u_g)
    e = np.array(err)
    assert e[1:] == pytest.approx(np.exp(-alpha_o * T_s) * e[:-1], abs=1e-9 * U_NOM)

    # The error of the sequence observer satisfies the characteristic equation of the
    # poles exp(p*alpha_o*T_s)
    p, u_gp, u_gn, err = (-1 - 1j, -0.58 - 1j), 0.8 * U_NOM, 0.2j * U_NOM, []
    observer = SequenceObserver(U_NOM, W_NOM, L, alpha_o, p, T_s)
    u_c, i_c = exact_model(u_gp, u_gn, L, t)
    for u_c_ab, i_c_ab, t_k in zip(u_c, i_c, t, strict=True):
        out = observer.compute_output(u_c_ab, i_c_ab)
        observer.update(T_s, out)
        err.append([u_gp - out.u_gp, u_gn * np.exp(-2j * W_NOM * t_k) - out.u_gn])
    e = np.array(err)
    z_1, z_2 = np.exp(np.array(p) * alpha_o * T_s)
    e_pred = (z_1 + z_2) * e[1:-1] - z_1 * z_2 * e[:-2]
    assert e[2:] == pytest.approx(e_pred, abs=1e-9 * U_NOM)

    # Exact sequence separation
    observer = SequenceObserver(U_NOM, W_NOM, L, alpha_o, p, T_s)
    i_cp, i_cn, seq = 10.0, 4j, []
    for t_k in T_s * np.arange(1000):
        i_c_ab = i_cp * np.exp(1j * W_NOM * t_k) + i_cn * np.exp(-1j * W_NOM * t_k)
        out = observer.compute_output(0j, i_c_ab)
        observer.update(T_s, out)
        seq.append((out.i_cp, out.i_cn * np.exp(2j * W_NOM * t_k)))
    assert seq[-1] == pytest.approx((i_cp, i_cn))


@pytest.mark.parametrize("ctrl", ["psc", "observer"])
def test_default_active_resistance(ctrl: str) -> None:
    """The default active resistance follows a configuration changed by replace()."""
    if ctrl == "psc":
        cfg = control.PowerSynchronizationControllerCfg(i_max=30)
        inner_ctrl = control.PowerSynchronizationController(replace(cfg, i_max=60))
    else:
        cfg = control.ObserverBasedGridFormingControllerCfg(i_max=30, L=L_BASE)
        inner_ctrl = control.ObserverBasedGridFormingController(replace(cfg, i_max=60))
    assert inner_ctrl.R_a == pytest.approx(0.25 * cfg.u_nom / 60)


def test_power_references_follow_current_limit() -> None:
    """The power references correspond to the limited current reference."""
    cfg = control.CurrentVectorControllerCfg(i_max=10, L=L_BASE)
    inner_ctrl = control.CurrentVectorController(cfg)
    fbk = control.PLLOutputs(u_g=300.0)
    ref = inner_ctrl.compute_output(9e3, 3e3, fbk)
    assert abs(ref.i_c) == pytest.approx(10)
    s_g = 1.5 * fbk.u_g * ref.i_c.conjugate()
    assert (ref.p_g, ref.q_g) == pytest.approx((s_g.real, s_g.imag))
    assert abs(ref.p_g + 1j * ref.q_g) == pytest.approx(1.5 * 300 * 10)
