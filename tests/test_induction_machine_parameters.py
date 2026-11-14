"""Flux-dependent induction-machine parameters."""

import numpy as np
import pytest

from motulator.drive.model import InductionMachine
from motulator.drive.utils._parameters import (
    InductionMachineInvGammaPars,
    InductionMachinePars,
)


@pytest.mark.parametrize("name", ["L_s", "L_ell", "R_r"])
def test_flux_dependent_parameters(name: str) -> None:
    """Derived parameters and machine dynamics use the current flux magnitude."""
    par = InductionMachinePars(
        n_p=2,
        R_s=1.0,
        L_s=(lambda psi: 0.4 * (1 + psi)) if name == "L_s" else 0.4,
        L_ell=(lambda psi: 0.02 * (1 + psi)) if name == "L_ell" else 0.02,
        R_r=(lambda psi: 2.0 * (1 + psi)) if name == "R_r" else 2.0,
    )
    machine = InductionMachine(par)
    for psi in (0.0, 0.5, 1.0):
        par.update_psi_s(psi)
        constant = InductionMachinePars(
            n_p=2,
            R_s=1.0,
            L_s=0.4 * (1 + psi) if name == "L_s" else 0.4,
            L_ell=0.02 * (1 + psi) if name == "L_ell" else 0.02,
            R_r=2.0 * (1 + psi) if name == "R_r" else 2.0,
        )
        for attr in ("gamma", "L_M", "L_sgm", "R_R", "R_sgm", "alpha", "w_rb"):
            assert getattr(par, attr) == pytest.approx(getattr(constant, attr))
        reference = InductionMachine(constant)
        for mdl in (machine, reference):
            mdl.state.psi_s_ab = psi * (0.6 + 0.8j)
            mdl.state.psi_r_ab = 0.2 + 0.1j
            mdl.inp.u_s_ab = 10 + 20j
            mdl.inp.w_M = 5.0
            mdl.set_outputs(0.0)
        assert machine.out.i_s_ab == pytest.approx(reference.out.i_s_ab)
        assert machine.out.i_r_ab == pytest.approx(reference.out.i_r_ab)
        assert machine.out.tau_M == pytest.approx(reference.out.tau_M)
        assert machine.rhs(0.0) == pytest.approx(reference.rhs(0.0))

    with pytest.raises(ValueError, match=name):
        InductionMachineInvGammaPars.from_gamma_pars(par)


def test_callable_leakage_time_series() -> None:
    """Result processing evaluates leakage inductance at every flux sample."""
    machine = InductionMachine(
        InductionMachinePars(2, 1.0, lambda psi: 2 + psi, lambda psi: 0.1 + psi, 1.0)
    )
    machine.init_state_history()
    machine.extend_state_history([[0.0, 1.0], [0.2, 0.4]], 0)
    series = machine.create_time_series(np.array([0.0, 1.0]))
    assert series.psi_R_ab == pytest.approx([0.2 / 1.1, 0.4 / 2.1])


def test_callable_parameters_ac_port() -> None:
    """The blocked-converter port includes flux-dependent leakage derivatives."""
    machine = InductionMachine(
        InductionMachinePars(
            2,
            1.0,
            lambda psi: 2 + psi,
            lambda psi: 0.1 + 0.02 * psi,
            lambda psi: 0.4 / (1 + psi),
        )
    )
    machine.state.psi_s_ab = 0.6 + 0.8j
    machine.state.psi_r_ab = 0.5 + 0.6j
    machine.inp.u_s_ab = 10 + 20j
    machine.inp.w_M = 5.0
    machine.set_outputs(0.0)
    port = machine.compute_ac_port()
    voltage = machine.inp.u_s_ab - port.e
    predicted = port.Y @ [voltage.real, voltage.imag]
    d_psi_s, d_psi_r = machine.rhs(0.0)
    initial_current = machine.out.i_s_ab
    dt = 1e-8
    machine.state.psi_s_ab += dt * d_psi_s
    machine.state.psi_r_ab += dt * d_psi_r
    machine.set_outputs(dt)
    derivative = (machine.out.i_s_ab - initial_current) / dt
    assert predicted == pytest.approx([derivative.real, derivative.imag], rel=1e-5)
