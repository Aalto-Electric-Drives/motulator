"""
Tests for the machine models.

The tests check the induction and synchronous machine models against their equivalent
circuits, analytic derivatives, and the power balance at operating points, with tight
tolerances, and the time series of short simulations. The published models and
parameters are from the following references.

[Tuo2010] Tuovinen, Hinkkanen, Luomi, "Modeling of saturation due to main and leakage
flux interaction in induction machines," IEEE Trans. Ind. Appl., 2010,
https://doi.org/10.1109/TIA.2010.2045210

[Mol2020] Mölsä, Saarakkala, Hinkkanen, Arkkio, Routimo, "A dynamic model for saturated
induction machines with closed rotor slots and deep bars," IEEE Trans. Energy Convers.,
2020, https://doi.org/10.1109/TEC.2019.2950810

"""

from dataclasses import astuple
from math import pi
from typing import Any

import numpy as np
import pytest

import motulator.drive.control.im as im_control
from motulator.drive import model
from motulator.drive.utils import BaseValues, NominalValues


def mutual_saturation_map(psi_s: Any, psi_r: Any) -> tuple[Any, Any]:
    """Mutual saturation model (12) of [Tuo2010], with the per-unit parameters fitted to
    the experimental data of the 2.2-kW machine (Table I)."""
    base = BaseValues.from_nominal(NominalValues(U=400, I=5, f=50, P=2.2e3))
    L_su, L_ellu, alpha, beta, gamma, a = 2.28, 0.216, 0.383, 0.511, 3.20, 7.5
    psi_ell = psi_r - psi_s
    x, y = np.abs(psi_s) / base.psi, np.abs(psi_ell) / base.psi
    k_m = 1 + alpha * x**a + gamma * L_su / 2 * x * y**2
    k_r = 1 + beta * y + gamma * L_ellu / 3 * x**3
    i_r = k_r * psi_ell / (L_ellu * base.L)
    return k_m * psi_s / (L_su * base.L) - i_r, i_r


def set_im_operating_point(machine: model.InductionMachine, w_M: float = 50.0) -> None:
    """Set an operating point with nonparallel fluxes."""
    machine.state.psi_s_dq, machine.state.psi_r_dq = 0.9 + 0.5j, 0.6 + 0.7j
    machine.state.theta_m = 0.4
    machine.inp.u_s_ab, machine.inp.w_M = 100 + 200j, w_M
    machine.set_outputs(0.0)


def assert_ac_port(machine: model.InductionMachine) -> None:
    """The AC port gives the stator current derivative in stator coordinates, which is
    compared with the derivative along the state derivatives of the model."""
    port = machine.compute_ac_port()
    assert port.i == pytest.approx(machine.out.i_s_ab)
    u = machine.inp.u_s_ab - port.e
    h = 1e-6
    state, d_state = np.array(machine.get_states()), np.array(machine.rhs(0.0))
    i_s_ab = []
    for step in (h, -h, 0.0):
        machine.set_states(list(state + step * d_state), 0)
        machine.set_outputs(0.0)
        i_s_ab.append(machine.out.i_s_ab)
    d_i_s = (i_s_ab[0] - i_s_ab[1]) / (2 * h)
    assert port.Y @ [u.real, u.imag] == pytest.approx(
        [d_i_s.real, d_i_s.imag], rel=1e-5
    )


def test_im_gamma_and_inverse_gamma_models() -> None:
    """The Γ and inverse-Γ parameters of the same machine give the same machine model,
    whose rotor quantities are scaled as in the parameters."""
    par = model.InductionMachinePars(
        n_p=2, R_s=3.7, R_r=2.5, L_ell=0.023, L_s=0.245, G_c=0.02
    )
    par_inv = model.InductionMachineInvGammaPars(
        n_p=2, R_s=3.7, R_R=par.R_R, L_sgm=par.L_sgm, L_M=par.L_M, G_c=0.02
    )
    assert (par_inv.alpha, par_inv.w_rb) == pytest.approx((par.alpha, par.w_rb))
    machines = [model.InductionMachine(par), model.InductionMachine(par_inv)]
    for machine, gamma in zip(machines, [1, par.gamma], strict=True):
        set_im_operating_point(machine)
        machine.state.psi_r_dq *= gamma
        machine.set_outputs(0.0)
    ref, mdl = machines
    assert mdl.out.i_s_ab == pytest.approx(ref.out.i_s_ab)
    assert mdl.out.tau_M == pytest.approx(ref.out.tau_M)
    assert mdl.out.i_r_dq * par.gamma == pytest.approx(ref.out.i_r_dq)
    d_psi_s, d_psi_r, d_theta_m = ref.rhs(0.0)
    assert mdl.rhs(0.0) == pytest.approx([d_psi_s, par.gamma * d_psi_r, d_theta_m])


def test_im_current_map_of_scalar_inductances() -> None:
    """A current map built of the scalar inductances gives the same machine model as the
    Γ-model parameters."""

    def L_s(psi: float) -> float:
        return 0.34 / (1 + (0.84 * psi) ** 7)

    def current_map(psi_s: Any, psi_r: Any) -> tuple[Any, Any]:
        i_r = (psi_r - psi_s) / 0.023
        return psi_s / L_s(abs(psi_s)) - i_r, i_r

    kwargs: dict[str, Any] = {"n_p": 2, "R_s": 3.7, "R_r": 2.5}
    machines = [
        model.InductionMachine(
            model.InductionMachinePars(**kwargs, L_ell=0.023, L_s=L_s)
        ),
        model.InductionMachine(
            model.SaturatedInductionMachinePars(**kwargs, current_map_fcn=current_map)
        ),
    ]
    for machine in machines:
        set_im_operating_point(machine)
    ref, mdl = machines
    assert astuple(mdl.out) == pytest.approx(astuple(ref.out), rel=1e-12)
    assert mdl.rhs(0.0) == pytest.approx(ref.rhs(0.0), rel=1e-12)
    port, port_ref = mdl.compute_ac_port(), ref.compute_ac_port()
    assert port.e == pytest.approx(port_ref.e, rel=1e-6)
    assert port.Y == pytest.approx(port_ref.Y, rel=1e-6)


def test_im_current_map_with_mutual_saturation() -> None:
    """With a reciprocal current map, where the main flux and the leakage flux saturate
    each other, the torque equals the torque computed from the rotor quantities and the
    AC port gives the stator current derivative. The time series are computed with the
    same map."""
    base = BaseValues.from_nominal(NominalValues(U=400, I=5, f=50, P=2.2e3))
    par = model.SaturatedInductionMachinePars(
        n_p=2,
        R_s=0.0628 * base.Z,
        R_r=0.0433 * base.Z,
        current_map_fcn=mutual_saturation_map,
    )
    machine = model.InductionMachine(par)
    set_im_operating_point(machine)
    i_r, psi_r = machine.out.i_r_dq, machine.state.psi_r_dq
    tau_M = -1.5 * par.n_p * np.imag(i_r * np.conj(psi_r))
    assert machine.out.tau_M == pytest.approx(tau_M)
    assert_ac_port(machine)

    mdl = model.Drive(
        model.InductionMachine(par),
        model.MechanicalSystem(J=0.015),
        model.VoltageSourceConverter(u_dc=540),
    )
    est_par = im_control.InductionMachineInvGammaPars(
        n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224
    )
    cfg = im_control.ObserverBasedVHzControllerCfg(psi_s_nom=1.04, i_s_max=10.6)
    ctrl = im_control.VHzControlSystem(
        im_control.ObserverBasedVHzController(est_par, cfg), slew_rate=2 * pi * 120
    )
    ctrl.set_speed_ref(50.0)
    res = model.Simulation(mdl, ctrl, show_progress=False).simulate(t_stop=0.2)
    assert res.success
    ts = res.mdl.machine
    i_s_dq, i_r_dq = mutual_saturation_map(ts.psi_s_dq, ts.psi_r_dq)
    assert ts.i_s_dq == pytest.approx(i_s_dq)
    assert ts.i_r_dq == pytest.approx(i_r_dq)
    assert ts.i_s_ab == pytest.approx(ts.exp_j_theta_m * ts.i_s_dq)
    assert ts.psi_c.shape == (res.mdl.t.size, 0)  # The cage is a resistance


def test_im_cage_impedance() -> None:
    """The ladder of rectangular bars equals the second-order impedance (8) of [Mol2020]
    and approaches the bar impedance (6) as its order increases. The parameters are
    those of the 5.6-kW machine (Table I). Elements that are not physical are
    rejected."""
    R_0, L_0 = 0.16, 6e-3
    s = 2j * pi * np.array([1.0, 50.0, 150.0])
    Z_2 = (15 * (L_0 * s) ** 2 + 140 * L_0 * R_0 * s + 105 * R_0**2) / (
        (L_0 * s) ** 2 + 35 * L_0 * R_0 * s + 105 * R_0**2
    )
    cage = model.CageImpedance.from_rectangular_bars(R_0, L_0, 2)
    assert cage.Z(s) == pytest.approx(R_0 * Z_2)
    x = np.sqrt(3 * L_0 / R_0 * s)
    cage = model.CageImpedance.from_rectangular_bars(R_0, L_0, 20)
    assert cage.Z(s) == pytest.approx(R_0 * x / np.tanh(x))
    with pytest.raises(ValueError, match="one more"):
        model.CageImpedance([R_0], [L_0])
    with pytest.raises(ValueError, match="R must be"):
        model.CageImpedance([-R_0])
    # A zero resistance inside the ladder would short-circuit an inductance
    with pytest.raises(ValueError, match="R must be"):
        model.CageImpedance([R_0, 0.0], [L_0])
    with pytest.raises(ValueError, match="L must be"):
        model.CageImpedance([R_0, R_0], [0.0])
    assert model.CageImpedance([0.0]).Z(0j) == 0


def test_im_deep_bars() -> None:
    """With the cage impedance `Z_c`, the stator impedance of the machine model at
    standstill is ``R_s + s*L_s || (s*L_ell + Z_c)`` for the Γ-model inductances. The
    parameters are those of the 5.6-kW machine of [Mol2020] (Table I), with the
    unsaturated stator inductance and the fully saturated inductance of the slot
    bridges. The Γ-model parameters with the cage impedance give the same model, and
    their inverse-Γ parameters are those of the low-frequency equivalent of the cage. In
    a simulation, the flux linkages of the cage inductances are in the time series and
    the rotor angle follows the angle of the mechanical system."""
    R_s, L_s, L_ell = 1.0, 0.18, 0.015
    cage = model.CageImpedance.from_rectangular_bars(R_0=0.16, L_0=6e-3, N=2)

    def current_map(psi_s: Any, psi_r: Any) -> tuple[Any, Any]:
        i_r = (psi_r - psi_s) / L_ell
        return psi_s / L_s - i_r, i_r

    par = model.SaturatedInductionMachinePars(
        n_p=2, R_s=R_s, R_r=cage, current_map_fcn=current_map
    )
    machine = model.InductionMachine(par)
    n = len(machine.get_states())
    assert n == 5

    def evaluate(state: Any, u_s_ab: complex) -> tuple[np.ndarray, complex]:
        machine.set_states(list(state), 0)
        machine.inp.u_s_ab = u_s_ab
        machine.set_outputs(0.0)
        return np.array(machine.rhs(0.0)), machine.out.i_s_ab

    # The model is linear at standstill, so its matrices are obtained column by column
    unit = np.eye(n, dtype=complex)
    A = np.column_stack([evaluate(x, 0j)[0] for x in unit])
    C = np.array([evaluate(x, 0j)[1] for x in unit])
    B = evaluate(np.zeros(n), 1 + 0j)[0]
    for f in (2.0, 50.0, 500.0):
        s = 2j * pi * f
        Z_r = s * L_ell + cage.Z(s)
        Z = R_s + s * L_s * Z_r / (s * L_s + Z_r)
        assert C @ np.linalg.solve(s * unit - A, B) == pytest.approx(1 / Z)

    par_gamma = model.InductionMachinePars(
        n_p=2, R_s=R_s, R_r=cage, L_ell=L_ell, L_s=L_s
    )
    par_dc = model.InductionMachinePars(
        n_p=2, R_s=R_s, R_r=0.16, L_ell=L_ell + 6e-3, L_s=L_s
    )
    for attr in ("R_R", "L_sgm", "L_M", "alpha", "w_rb"):
        assert getattr(par_gamma, attr) == pytest.approx(getattr(par_dc, attr))
    machine_gamma = model.InductionMachine(par_gamma)
    state = [0.9 + 0.5j, 0.6 + 0.7j, 0.4, 1e-3 - 2e-3j, 3e-3 + 1e-3j]
    for m in (machine, machine_gamma):
        m.set_states(state, 0)
        m.inp.u_s_ab, m.inp.w_M = 100 + 200j, 50.0
        m.set_outputs(0.0)
    assert machine_gamma.rhs(0.0) == pytest.approx(machine.rhs(0.0))

    machine = model.InductionMachine(par)
    mdl = model.Drive(
        machine, model.MechanicalSystem(J=0.05), model.VoltageSourceConverter(u_dc=650)
    )
    est_par = im_control.InductionMachineInvGammaPars(
        n_p=2, R_s=R_s, R_R=0.16, L_sgm=L_ell + 6e-3, L_M=L_s
    )
    cfg = im_control.ObserverBasedVHzControllerCfg(psi_s_nom=1.0, i_s_max=30)
    ctrl = im_control.VHzControlSystem(
        im_control.ObserverBasedVHzController(est_par, cfg), slew_rate=2 * pi * 120
    )
    ctrl.set_speed_ref(50.0)
    mdl.mechanics.set_external_load_torque(20.0)
    res = model.Simulation(mdl, ctrl, show_progress=False).simulate(t_stop=0.5)
    assert res.success
    ts = res.mdl.machine
    assert ts.psi_c.shape == (res.mdl.t.size, 2)
    assert np.mean(ts.tau_M[res.mdl.t > 0.45]) == pytest.approx(20.0, rel=0.05)
    # At the slip frequency, the rotor current flows mainly through the first inductance
    assert abs(ts.psi_c[-1, 0]) == pytest.approx(6e-3 * abs(ts.i_r_dq[-1]), rel=0.1)
    # The angle state is kept within [-pi, pi) between the integration intervals
    theta_M = np.unwrap(res.mdl.mechanics.theta_M)
    err = np.angle(np.exp(1j * (ts.theta_m - par.n_p * theta_M)))
    assert err == pytest.approx(0, abs=1e-9)
    assert abs(machine.state.theta_m) <= pi


@pytest.mark.parametrize("w_M", [-120.0, 0.0, 50.0])
def test_im_spatial_harmonics(w_M: float) -> None:
    """With a magnetic map that depends on the rotor angle, the machine model fulfills
    the power balance and the AC port gives the stator current derivative, also at
    standstill and in the reverse direction. The map is synthetic: the inverse leakage
    inductance of the Γ model varies with the rotor angle."""
    R_s, R_r, L_s, L_ell, eps, n = 3.7, 2.5, 0.245, 0.023, 0.1, 18

    def energy(psi_s: complex, psi_r: complex, theta_m: float) -> float:
        k = 1 + eps * np.cos(n * theta_m)
        return abs(psi_s) ** 2 / (2 * L_s) + k * abs(psi_r - psi_s) ** 2 / (2 * L_ell)

    def magnetic_map(
        psi_s: Any, psi_r: Any, exp_j_theta_m: Any
    ) -> tuple[Any, Any, Any]:
        i_r = (1 + eps * np.real(exp_j_theta_m**n)) * (psi_r - psi_s) / L_ell
        i_s = psi_s / L_s - i_r
        # Derivative of the energy with respect to the rotor angle
        d_W = -eps * n * np.imag(exp_j_theta_m**n) * np.abs(psi_r - psi_s) ** 2
        return i_s, i_r, 1.5 * (np.imag(i_s * np.conj(psi_s)) - d_W / (2 * L_ell))

    par = model.SpatialSaturatedInductionMachinePars(
        n_p=2, R_s=R_s, R_r=R_r, magnetic_map_fcn=magnetic_map
    )
    machine = model.InductionMachine(par)
    set_im_operating_point(machine, w_M)
    inp, out = machine.inp, machine.out
    p_s = 1.5 * np.real(inp.u_s_ab * np.conj(out.i_s_ab))
    p_loss = 1.5 * (R_s * abs(out.i_s_ab) ** 2 + R_r * abs(out.i_r_dq) ** 2)
    h = 1e-6
    state, d_state = np.array(machine.get_states()), np.array(machine.rhs(0.0))
    psi_s, psi_r, theta_m = state + h * d_state
    W_p = energy(psi_s, psi_r, theta_m.real)
    psi_s, psi_r, theta_m = state - h * d_state
    W_n = energy(psi_s, psi_r, theta_m.real)
    p_field = 1.5 * (W_p - W_n) / (2 * h)
    assert p_s - p_loss - out.tau_M * inp.w_M == pytest.approx(p_field, rel=1e-6)
    assert_ac_port(machine)


@pytest.mark.parametrize("w_M", [-120.0, 0.0, 50.0])
def test_sm_ac_port(w_M: float) -> None:
    """The AC port of a synchronous machine gives the analytic stator current derivative
    for an anisotropic magnetic map that depends on the rotor angle, also at standstill
    and in the reverse direction. The map is synthetic: a spatial harmonic modulates the
    inverse inductances of the 2.2-kW IPMSM used in the other tests."""
    n_p, R_s, L_d, L_q, psi_f, eps, n = 3, 3.6, 0.036, 0.051, 0.545, 0.1, 6

    def current(psi_s: Any) -> Any:
        """Current map without the spatial harmonic."""
        return (np.real(psi_s) - psi_f) / L_d + 1j * np.imag(psi_s) / L_q

    def magnetic_map(psi_s: Any, exp_j_theta_m: Any) -> tuple[Any, Any]:
        k = 1 + eps * np.real(exp_j_theta_m**n)
        i_0 = current(psi_s)
        i_s = k * i_0
        # Torque from the energy k*W_0, where W_0 is the energy without the harmonic
        W_0 = 0.5 * (L_d * np.real(i_0) ** 2 + L_q * np.imag(i_0) ** 2)
        d_W = -eps * n * np.imag(exp_j_theta_m**n) * W_0
        return i_s, 1.5 * (np.imag(i_s * np.conj(psi_s)) - d_W)

    par = model.SpatialSaturatedSynchronousMachinePars(
        n_p=n_p, R_s=R_s, magnetic_map_fcn=magnetic_map
    )
    machine = model.SynchronousMachine(par)
    psi_s, theta_m, u_s_ab = 0.6 + 0.3j, 0.4, 170 - 80j
    machine.state.psi_s_dq, machine.state.theta_m = psi_s, theta_m
    machine.inp.u_s_ab, machine.inp.w_M = u_s_ab, w_M
    machine.set_outputs(0.0)
    port = machine.compute_ac_port()

    # Analytic derivative of the stator current in stator coordinates
    w_m, exp_j_theta_m = n_p * w_M, np.exp(1j * theta_m)
    k, d_k = 1 + eps * np.cos(n * theta_m), -eps * n * np.sin(n * theta_m)
    i_0 = current(psi_s)
    i_s = k * i_0
    d_psi_s = u_s_ab / exp_j_theta_m - R_s * i_s - 1j * w_m * psi_s
    d_i_s = k * (d_psi_s.real / L_d + 1j * d_psi_s.imag / L_q) + d_k * w_m * i_0
    d_i_s_ab = exp_j_theta_m * (d_i_s + 1j * w_m * i_s)
    u = u_s_ab - port.e
    assert port.i == pytest.approx(exp_j_theta_m * i_s)
    assert complex(*(port.Y @ [u.real, u.imag])) == pytest.approx(d_i_s_ab, rel=1e-6)
