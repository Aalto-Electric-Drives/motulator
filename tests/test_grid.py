"""
Closed-loop tests for grid converters.

The tests check that the active and reactive powers follow their references in steady
state, with loose tolerances.

"""

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

    res = model.Simulation(system(), ctrl, show_progress=False).simulate(t_stop=0.1)

    p_g, q_g = grid_powers(res, 0.08)
    assert p_g == pytest.approx(p_g_ref, abs=0.02 * 10e3)
    assert q_g == pytest.approx(q_g_ref, abs=0.02 * 10e3)


def test_grid_forming() -> None:
    """Observer-based grid-forming control tracks the active power reference."""
    cfg = control.ObserverBasedGridFormingControllerCfg(
        i_max=30, L=0.35 * L_BASE, R_a=0.2 * Z_BASE, u_nom=U_NOM, w_nom=W_NOM
    )
    ctrl = control.GridConverterControlSystem(
        control.ObserverBasedGridFormingController(cfg)
    )
    p_g_ref = 5e3
    ctrl.set_ac_voltage_ref(U_NOM)
    ctrl.set_power_ref(lambda t: (t > 0.05) * p_g_ref)

    mdl = system(L_g=0.74 * L_BASE)
    res = model.Simulation(mdl, ctrl, show_progress=False).simulate(t_stop=0.4)

    p_g, _ = grid_powers(res, 0.35)
    assert p_g == pytest.approx(p_g_ref, abs=0.02 * 10e3)


def test_dft_of_unevenly_sampled_signal() -> None:
    """The DFT of the identification finds a sinusoid sampled at uneven instants."""
    cfg = utils.IdentificationCfg(abs_u_e=1.0)
    t = np.sort(np.random.default_rng(0).uniform(0, 0.1, 4000))
    u = 2.0 * np.cos(2 * np.pi * 50 * t + 0.3)
    y = dft(cfg, t, u, 50.0, initial_simulation=True)
    assert y == pytest.approx(2.0 * np.exp(0.3j), rel=1e-3)
