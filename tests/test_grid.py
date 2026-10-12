"""
Closed-loop tests for grid converters.

The tests check that the active and reactive powers follow their references in steady
state, with loose tolerances.

"""

from dataclasses import replace

import numpy as np
import pytest

from motulator.common.model import SimulationResults
from motulator.grid import control, model, utils
from motulator.grid.utils._identification import dft

U_NOM = np.sqrt(2 / 3) * 400  # Peak phase voltage
W_NOM = 2 * np.pi * 50
Z_BASE = U_NOM / (np.sqrt(2) * 14.5)  # Base impedance of a 10-kVA converter
L_BASE = Z_BASE / W_NOM


def grid_powers(res: SimulationResults, t_start: float) -> tuple[float, float]:
    """Average grid powers at the grid voltage source after `t_start`."""
    end = res.mdl.t > t_start
    s_g = 1.5 * res.mdl.ac_filter.e_g_ab[end] * np.conj(res.mdl.ac_filter.i_g_ab[end])
    return float(np.mean(s_g.real)), float(np.mean(s_g.imag))


def system(L_g: float = 0.0) -> model.GridConverterSystem:
    return model.GridConverterSystem(
        model.VoltageSourceConverter(u_dc=650),
        model.LFilter(L_f=0.2 * L_BASE, L_g=L_g),
        model.ThreePhaseSource(w_g=W_NOM, e_g=U_NOM),
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
    if method == "psc":
        return
    # The post-processed signals are in the same coordinates, aligned with v_c
    end = res.ctrl.t > 0.35
    assert np.abs(res.ctrl.fbk.v_c.imag).max() < 1e-9 * U_NOM
    u_c_err = np.mean(res.ctrl.fbk.u_c[end] - res.ctrl.ref.u_c[end])
    assert abs(u_c_err) < 0.02 * U_NOM


def test_dft_of_unevenly_sampled_signal() -> None:
    """The DFT of the identification finds a sinusoid sampled at uneven instants."""
    cfg = utils.IdentificationCfg(abs_u_e=1.0)
    t = np.sort(np.random.default_rng(0).uniform(0, 0.1, 4000))
    u = 2.0 * np.cos(2 * np.pi * 50 * t + 0.3)
    y = dft(cfg, t, u, 50.0, initial_simulation=True)
    assert y == pytest.approx(2.0 * np.exp(0.3j), rel=1e-3)


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


def test_identification_at_sampling_frequency() -> None:
    """The identified GFL admittance at the sampling frequency is that of the filter."""
    # At the sampling frequency, the excitation aliases to zero frequency in the
    # sampled control system, so the converter voltage does not respond to it
    T_s, L_f = 1e-4, 0.2 * L_BASE
    cfg = control.CurrentVectorControllerCfg(i_max=30, L=L_f, T_s=T_s)
    ctrl = control.GridConverterControlSystem(control.CurrentVectorController(cfg))
    ctrl.set_power_ref(5e3)
    ctrl.set_reactive_power_ref(0.0)
    freqs = np.array([1 / T_s])
    id_cfg = utils.IdentificationCfg(
        abs_u_e=0.05 * U_NOM, freqs=freqs, t0=0.1, multiprocess=False
    )
    res = utils.run_identification(id_cfg, system(), ctrl)

    Y_L = 1 / (2j * np.pi * freqs[0] * L_f)
    assert res.Y_dd[0] == pytest.approx(Y_L, rel=0.05)
    freqs *= 2  # The results do not share the frequency array with the configuration
    assert res.f_e[0] == pytest.approx(1 / T_s)
