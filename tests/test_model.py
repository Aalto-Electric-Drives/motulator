"""
Tests for the model base class with custom subsystems.

The model checks its wiring at construction, the names of the subsystems name their
results, the evaluation order of the outputs follows the declared direct dependencies,
and the signals derived from the connected inputs are computed from the time series of
the inputs.

"""

from dataclasses import dataclass
from math import pi
from typing import Any

import numpy as np
import pytest

from motulator.common.model import (
    CompositeModel,
    Model,
    Simulation,
    Subsystem,
    SubsystemTimeSeries,
    ThreePhaseSource,
)
from motulator.drive.model import (
    MechanicalSystem,
    SynchronousMachine,
    SynchronousMachinePars,
    TwoMassMechanicalSystem,
)

K = 10.0  # Slope of the ramp load torque (Nm/s)
B_L = 0.1  # Friction coefficient (Nm/(rad/s))


@dataclass
class _Outputs:
    tau_L: float = 0.0


@dataclass
class _TimeSeries(SubsystemTimeSeries):
    t: np.ndarray
    tau_L: np.ndarray


class _RampTorque(Subsystem):
    """Load torque increasing linearly with time."""

    def __init__(self) -> None:
        self.inp = None
        self.out = _Outputs()
        self.state = None

    def set_outputs(self, t: float) -> None:
        self.out.tau_L = K * t

    def rhs(self, t: float) -> list[complex]:
        return []

    def create_time_series(self, t: np.ndarray) -> _TimeSeries:
        return _TimeSeries(t, K * t)


def mechanics(two_mass: bool) -> MechanicalSystem | TwoMassMechanicalSystem:
    """Mechanical system with friction, initially rotating."""
    if two_mass:
        mech = TwoMassMechanicalSystem(J_M=0.01, J_L=0.01, K_S=100.0, C_S=0.1, B_L=B_L)
        mech.state.w_L = 5.0 + 0j
    else:
        mech = MechanicalSystem(J=0.02, B_L=B_L)
    mech.state.w_M = 5.0 + 0j
    return mech


def loaded(mech: Subsystem) -> Model:
    """Mechanical system loaded by the ramp torque."""
    load = _RampTorque()
    connections = {("mechanics", "tau_L"): ("load", "tau_L")}
    return Model(None, {"load": load, "mechanics": mech}, connections)


@pytest.mark.parametrize("two_mass", [False, True])
@pytest.mark.parametrize("connected", [False, True])
def test_load_torque_in_results(two_mass: bool, connected: bool) -> None:
    """The total load torque in the results consists of the friction and the load
    torque, which is either connected from another subsystem or an external function
    of time."""
    mech = mechanics(two_mass)
    if connected:
        mdl = loaded(mech)
    else:
        mech.set_external_load_torque(lambda t: K * t)
        mdl = Model(None, {"mechanics": mech}, {})
    res = Simulation(mdl, show_progress=False).simulate(0.05)
    ts = res.mdl.mechanics
    w = ts.w_L if two_mass else ts.w_M
    assert ts.tau_L == pytest.approx(K * res.mdl.t)
    assert ts.tau_L_tot == pytest.approx(B_L * w + K * res.mdl.t)
    assert ts.w_M[-1] < ts.w_M[0]  # Decelerates under the load


def test_shared_subsystem_under_different_names() -> None:
    """A subsystem shared by the models of a composite is integrated once, and its
    results, computed with the complete wiring, are the same under its name in each
    model."""
    mech = mechanics(two_mass=False)
    mdl = CompositeModel({"a": loaded(mech), "b": Model(None, {"rotor": mech}, {})})
    assert len(mdl.get_initial_values()) == 2
    res = Simulation(mdl, show_progress=False).simulate(0.05)
    assert res.mdl.b.rotor is res.mdl.a.mechanics
    ts = res.mdl.b.rotor
    assert ts.tau_L == pytest.approx(K * res.mdl.t)  # Connected in model a only
    assert ts.tau_L_tot == pytest.approx(B_L * ts.w_M + K * res.mdl.t)
    assert ts.w_M[-1] < ts.w_M[0]  # Loaded through a


def test_wiring_is_checked() -> None:
    """The names must be valid and the subsystems distinct, and the connections must
    refer to the subsystems of the model and to their existing signals."""
    load, mech = _RampTorque(), MechanicalSystem(J=1.0)
    for name in ("t", "_mech", "1", "a.b"):
        with pytest.raises(ValueError):  # Invalid name
            Model(None, {name: mech}, {})
    with pytest.raises(ValueError):  # The same subsystem under two names
        Model(None, {"a": mech, "b": mech}, {})
    with pytest.raises(ValueError):  # Unknown subsystem
        Model(None, {"load": load}, {("mechanics", "tau_L"): ("load", "tau_L")})
    with pytest.raises(ValueError):  # Nonexistent input
        Model(None, {"load": load, "mech": mech}, {("mech", "tau"): ("load", "tau_L")})


@dataclass
class _Signal:
    x: float = 0.0


class _Gain(Subsystem):
    """Output equal to the input, i.e., a direct dependency."""

    feedthrough = frozenset({"x"})

    def __init__(self) -> None:
        self.inp = _Signal()
        self.out = _Signal()
        self.state = None

    def set_outputs(self, t: float) -> None:
        self.out.x = self.inp.x

    def rhs(self, t: float) -> list[complex]:
        return []

    def create_time_series(self, t: np.ndarray) -> SubsystemTimeSeries:
        raise NotImplementedError


def test_direct_dependencies_order_the_evaluation() -> None:
    """A subsystem is evaluated after the sources of the inputs its outputs depend on
    directly, wherever it is listed, and a loop of direct dependencies is rejected."""
    a, b = _Gain(), _Gain()
    mdl = Model(None, {"b": b, "a": a}, {("b", "x"): ("a", "x")})
    a.inp.x = 1.0
    mdl.set_outputs(0.0)
    assert b.out.x == 1.0
    with pytest.raises(ValueError):  # Loop
        Model(None, {"a": a, "b": b}, {("a", "x"): ("b", "x"), ("b", "x"): ("a", "x")})


@pytest.mark.parametrize("G_c", [0.0, 0.01])
def test_listing_order_does_not_affect_results(G_c: float) -> None:
    """A machine listed before the voltage source, which its current depends on
    directly, gives the same results as listed after it, in the simulation and in the
    post-processing."""

    def simulate(machine_first: bool) -> Any:
        par = SynchronousMachinePars(
            n_p=2, R_s=1.0, L_d=0.01, L_q=0.02, psi_f=0.1, G_c=G_c
        )
        machine = SynchronousMachine(par)
        ac_source = ThreePhaseSource(w_g=2 * pi * 50, e_g=100.0)
        subsystems = {"machine": machine, "ac_source": ac_source}
        if not machine_first:
            subsystems = dict(reversed(subsystems.items()))
        connections = {("machine", "u_s_ab"): ("ac_source", "e_g_ab")}
        mdl = Model(None, subsystems, connections)
        return Simulation(mdl, show_progress=False).simulate(5e-3)

    res, res_first = simulate(False), simulate(True)
    np.testing.assert_array_equal(res_first.mdl.t, res.mdl.t)
    np.testing.assert_array_equal(res_first.mdl.machine.i_s_ab, res.mdl.machine.i_s_ab)
    np.testing.assert_array_equal(res_first.mdl.machine.u_s_ab, res.mdl.machine.u_s_ab)
