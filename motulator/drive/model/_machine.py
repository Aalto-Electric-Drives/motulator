"""
Continuous-time models for electric machines.

Peak-valued complex space vectors are used. Quantities in stator coordinates are marked
with ab and quantities in rotor coordinates are marked with dq.

"""

from cmath import exp
from collections.abc import Callable
from dataclasses import InitVar, dataclass, field
from typing import Any

import numpy as np

from motulator.common.model import ACPort, Subsystem, SubsystemTimeSeries
from motulator.common.model._ac_port import derivative, jacobian
from motulator.common.utils._utils import empty_array, wrap
from motulator.drive.utils._parameters import (
    BaseInductionMachinePars,
    BaseSynchronousMachinePars,
    CageImpedance,
)


# %%
def _compute_ac_port(
    current: Callable[[complex, complex], Any],
    psi_s_dq: complex,
    theta_m: float,
    w_m: float,
    R_s: float,
    d_i_s_dq: complex = 0j,
) -> ACPort:
    """
    Compute the AC port of a machine model in rotor coordinates, see `ACPort`.

    The stator current in rotor coordinates is ``current(psi_s_dq, exp_j_theta_m)``,
    the other states being constant, and `d_i_s_dq` is its rate of change due to the
    other states.

    """
    exp_j_theta_m = exp(1j * theta_m)
    i_s_dq = complex(current(psi_s_dq, exp_j_theta_m))

    def i_s_ab(d_psi_s_ab: complex) -> complex:
        """Stator current in stator coordinates after a stator flux step."""
        psi = psi_s_dq + d_psi_s_ab * exp_j_theta_m.conjugate()
        return complex(current(psi, exp_j_theta_m)) * exp_j_theta_m

    # Inverse differential inductance in stator coordinates
    Y = jacobian(i_s_ab, 0j)
    # Current derivative at a constant stator flux linkage in rotor coordinates, i.e.,
    # at u_s = R_s*i_s + 1j*w_m*psi_s, due to the other states, the spatial harmonics,
    # and the rotation of the coordinates
    d_i_s_dq += derivative(lambda th: current(psi_s_dq, exp(1j * th)), theta_m, w_m)
    d_i_s_ab = (d_i_s_dq + 1j * w_m * i_s_dq) * exp_j_theta_m
    # The current derivative is Y @ (u_s - e), which gives the EMF e
    u = np.linalg.solve(Y, [d_i_s_ab.real, d_i_s_ab.imag])
    e_ab = (R_s * i_s_dq + 1j * w_m * psi_s_dq) * exp_j_theta_m - complex(u[0], u[1])
    return ACPort(i_s_dq * exp_j_theta_m, e_ab, Y)


# %%
@dataclass
class InductionMachineInputs:
    """Machine inputs."""

    u_s_ab: complex = 0j
    w_M: float = 0.0  # Mechanical rotor speed (rad/s)


@dataclass
class InductionMachineOutputs:
    """Machine outputs."""

    i_s_ab: complex
    i_s_dq: complex
    i_r_dq: complex
    tau_M: float


@dataclass
class InductionMachineStates:
    """State variables."""

    cage: InitVar[CageImpedance]
    psi_s_dq: complex = 0j
    psi_r_dq: complex = 0j
    theta_m: complex = 0j  # Electrical rotor angle (rad), imaginary part always zero
    psi_c: np.ndarray = field(init=False)  # Flux linkages of the cage inductances

    def __post_init__(self, cage: CageImpedance) -> None:
        self.psi_c = np.zeros(np.size(cage.L), dtype=complex)


class InductionMachine(Subsystem):
    """
    Induction machine model.

    The model consists of the stator resistance, the magnetic model, and the rotor cage.
    It is internally represented in rotor coordinates, in which the cage impedance is
    time invariant and the magnetic model can depend on the rotor angle. The interfaces
    are in stator coordinates. The magnetic model gives the stator and rotor currents as
    functions of the stator and rotor flux linkages. Depending on the parameters, it is
    the inverse-Γ model (`InductionMachineInvGammaPars`), the Γ model with optional
    main-flux saturation (`InductionMachinePars`), or a general magnetic map, either
    without spatial harmonics (`SaturatedInductionMachinePars`) or with them
    (`SpatialSaturatedInductionMachinePars`). The rotor cage is a resistance or, for
    modeling the deep-bar effect, a ladder circuit (`CageImpedance`). Optionally,
    eddy-current core losses can be modeled by means of a constant core-loss conductance
    `G_c`, connected between the stator resistance and the magnetic model.

    Parameters
    ----------
    par : BaseInductionMachinePars
        Machine parameters. Core losses are modeled if `par.G_c` is nonzero.

    Notes
    -----
    The rotor flux linkage `psi_r_dq` and the rotor current `i_r_dq` are scaled as in
    the parameters, e.g., they are the inverse-Γ quantities if the inverse-Γ parameters
    are used.

    The voltage across the core-loss conductance is `u_s - R_s*i_s`. Consequently, the
    stator current depends directly on the stator voltage. This algebraic loop is solved
    in a closed form. The electromagnetic torque is produced by the current flowing into
    the magnetic model, i.e., the core-loss current is excluded from the torque.

    """

    # The stator current depends directly on the voltage through the core losses
    feedthrough = frozenset({"u_s_ab"})

    def __init__(self, par: BaseInductionMachinePars) -> None:
        self.par = par
        self.cage = par.cage
        self.inp: InductionMachineInputs = InductionMachineInputs()
        self.state: InductionMachineStates = InductionMachineStates(self.cage)
        i_s_dq, i_r_dq, i_s_ab, tau_M = self._compute_outputs(
            self.state, self.inp, 1 + 0j
        )
        self.out: InductionMachineOutputs = InductionMachineOutputs(
            i_s_ab=i_s_ab, i_s_dq=i_s_dq, i_r_dq=i_r_dq, tau_M=tau_M
        )

    def _compute_outputs(
        self, state: Any, inp: Any, exp_j_theta_m: Any
    ) -> tuple[Any, Any, Any, Any]:
        """Compute output variables."""
        par = self.par
        # Currents into the magnetic model, i.e., the core-loss current excluded
        i_s_dq, i_r_dq, tau_m = par.magnetic_map(
            state.psi_s_dq, state.psi_r_dq, exp_j_theta_m
        )
        if par.G_c:
            # In rotor coordinates, the rotational EMF does not affect the core-loss
            # branch
            u_s_dq = inp.u_s_ab * exp_j_theta_m.conjugate()
            i_s_dq = (i_s_dq + par.G_c * u_s_dq) / (1 + par.G_c * par.R_s)
        i_s_ab = i_s_dq * exp_j_theta_m
        tau_M = par.n_p * tau_m
        return i_s_dq, i_r_dq, i_s_ab, tau_M

    def set_states(self, state_list: list[complex], index: int) -> int:
        """Set states, keeping the rotor angle within [-pi, pi)."""
        # The angle is wrapped only in the state of the model, which the next
        # integration interval starts from, so the solver sees no discontinuity
        index = super().set_states(state_list, index)
        self.state.theta_m = wrap(self.state.theta_m.real)
        return index

    def set_outputs(self, t: float) -> None:
        """Set output variables."""
        out, state = self.out, self.state
        out.i_s_dq, out.i_r_dq, out.i_s_ab, out.tau_M = self._compute_outputs(
            state, self.inp, exp(1j * state.theta_m.real)
        )

    def rhs(self, t: float) -> list[complex]:
        """Compute state derivatives."""
        # Rotor coordinates are internally used
        state, inp, out, par = self.state, self.inp, self.out, self.par
        u_s_dq = inp.u_s_ab * exp(-1j * state.theta_m.real)
        w_m = par.n_p * inp.w_M
        d_psi_s_dq = u_s_dq - par.R_s * out.i_s_dq - 1j * w_m * state.psi_s_dq
        # The rotor winding is short-circuited, so only the cage impedance remains
        u_cage_dq, d_psi_c = self.cage.rhs(state.psi_c, out.i_r_dq)
        d_psi_r_dq = -u_cage_dq
        d_theta_m = w_m
        return [d_psi_s_dq, d_psi_r_dq, d_theta_m, *d_psi_c]

    def compute_ac_port(self) -> ACPort:
        """Compute the AC port for a converter with blocked gates, see `ACPort`."""
        state, inp, par = self.state, self.inp, self.par
        if par.G_c:
            raise NotImplementedError("Gate blocking is not supported with core losses")
        theta_m = state.theta_m.real
        exp_j_theta_m = exp(1j * theta_m)
        psi_s_dq, psi_r_dq = state.psi_s_dq, state.psi_r_dq
        i_r_dq = complex(par.magnetic_map(psi_s_dq, psi_r_dq, exp_j_theta_m)[1])
        # Current derivative due to the rotor flux linkage, driven by the cage
        d_psi_r_dq = -self.cage.rhs(state.psi_c, i_r_dq)[0]
        d_i_s_dq = derivative(
            lambda psi: par.magnetic_map(psi_s_dq, psi, exp_j_theta_m)[0],
            psi_r_dq,
            d_psi_r_dq,
        )
        return _compute_ac_port(
            lambda psi, ej: par.magnetic_map(psi, psi_r_dq, ej)[0],
            psi_s_dq,
            theta_m,
            par.n_p * inp.w_M,
            par.R_s,
            d_i_s_dq,
        )

    def create_time_series(self, t: np.ndarray) -> "InductionMachineTimeSeries":
        """Create the time series."""
        return InductionMachineTimeSeries(t, self)


@dataclass
class InductionMachineTimeSeries(SubsystemTimeSeries):
    """Continuous time series."""

    t: InitVar[np.ndarray]
    subsystem: InitVar[InductionMachine]
    # States
    psi_s_dq: np.ndarray = field(default_factory=empty_array)
    psi_r_dq: np.ndarray = field(default_factory=empty_array)
    theta_m: np.ndarray = field(default_factory=empty_array)
    psi_c: np.ndarray = field(default_factory=empty_array)
    # Outputs
    i_s_ab: np.ndarray = field(default_factory=empty_array)
    i_s_dq: np.ndarray = field(default_factory=empty_array)
    i_r_dq: np.ndarray = field(default_factory=empty_array)
    tau_M: np.ndarray = field(default_factory=empty_array)
    # Inputs
    u_s_ab: np.ndarray = field(default_factory=empty_array)
    w_M: np.ndarray = field(default_factory=empty_array)
    # Derived signals
    psi_s_ab: np.ndarray = field(default_factory=empty_array)
    exp_j_theta_m: np.ndarray = field(default_factory=empty_array)
    w_m: np.ndarray = field(default_factory=empty_array)

    def __post_init__(self, t: np.ndarray, subsystem: InductionMachine) -> None:
        """Compute time series from states."""
        self.psi_s_dq = np.array(subsystem._history["psi_s_dq"])
        self.psi_r_dq = np.array(subsystem._history["psi_r_dq"])
        psi_c = np.array(subsystem._history["psi_c"], dtype=complex)
        self.psi_c = psi_c.reshape(np.size(t), np.size(subsystem.cage.L))
        theta_m = np.real(np.array(subsystem._history["theta_m"]))
        self.exp_j_theta_m = np.exp(1j * theta_m)
        self.theta_m = np.angle(self.exp_j_theta_m)
        self.psi_s_ab = self.exp_j_theta_m * self.psi_s_dq

    def set_outputs(self, t: np.ndarray, subsystem: InductionMachine) -> None:
        """Compute the outputs, which depend on the voltage due to core losses."""
        i_s_dq, i_r_dq, i_s_ab, tau_M = subsystem._compute_outputs(
            self, self, self.exp_j_theta_m
        )
        self.i_s_dq = np.asarray(i_s_dq)
        self.i_r_dq = np.asarray(i_r_dq)
        self.i_s_ab = np.asarray(i_s_ab)
        self.tau_M = np.asarray(tau_M)

    def compute_derived_signals(
        self, t: np.ndarray, subsystem: InductionMachine
    ) -> None:
        """Compute the electrical rotor speed."""
        self.w_m = subsystem.par.n_p * self.w_M


# %%
@dataclass
class SynchronousMachineInputs:
    """Machine inputs."""

    u_s_ab: complex = 0j
    w_M: float = 0.0  # Mechanical rotor speed (rad/s)


@dataclass
class SynchronousMachineOutputs:
    """Machine outputs."""

    i_s_ab: complex
    i_s_dq: complex
    tau_M: float


@dataclass
class SynchronousMachineStates:
    """State variables."""

    par: InitVar[BaseSynchronousMachinePars]
    psi_s_dq: complex = 0j
    theta_m: complex = 0j  # Electrical rotor angle (rad), imaginary part always zero

    def __post_init__(self, par: BaseSynchronousMachinePars) -> None:
        self.psi_s_dq = complex(par.psi_f)


class SynchronousMachine(Subsystem):
    """
    Synchronous machine model.

    The model consists of the stator resistance and the magnetic model. It is internally
    represented in rotor coordinates, in which the magnetic model is time invariant or
    depends only on the rotor angle. The interfaces are in stator coordinates. The
    magnetic model gives the stator current as a function of the stator flux linkage.
    Depending on the parameters, it is linear (`SynchronousMachinePars`), a saturated
    current map (`SaturatedSynchronousMachinePars`), or a magnetic map with spatial
    harmonics (`SpatialSaturatedSynchronousMachinePars`). Optionally, eddy-current core
    losses can be modeled by means of a constant core-loss conductance `G_c`, connected
    between the stator resistance and the magnetic model.

    Parameters
    ----------
    par : BaseSynchronousMachinePars
        Machine parameters. Core losses are modeled if `par.G_c` is nonzero.

    Notes
    -----
    The voltage across the core-loss conductance is `u_s - R_s*i_s`. Consequently, the
    stator current depends directly on the stator voltage. This algebraic loop is solved
    in a closed form. The electromagnetic torque is produced by the current flowing into
    the magnetic model, i.e., the core-loss current is excluded from the torque.

    """

    # The stator current depends directly on the voltage through the core losses
    feedthrough = frozenset({"u_s_ab"})

    def __init__(self, par: BaseSynchronousMachinePars) -> None:
        self.par = par
        self.inp: SynchronousMachineInputs = SynchronousMachineInputs()
        self.state: SynchronousMachineStates = SynchronousMachineStates(par)
        i_s_dq, i_s_ab, tau_M = self._compute_outputs(self.state, self.inp, 1 + 0j)
        self.out: SynchronousMachineOutputs = SynchronousMachineOutputs(
            i_s_ab=i_s_ab, i_s_dq=i_s_dq, tau_M=tau_M
        )

    def _compute_outputs(
        self, state: Any, inp: Any, exp_j_theta_m: Any
    ) -> tuple[Any, Any, Any]:
        """Compute output variables."""
        par = self.par
        # Current into the magnetic model, i.e., the core-loss current excluded
        i_s_dq, tau_m = par.magnetic_map(state.psi_s_dq, exp_j_theta_m)
        if par.G_c:
            # In rotor coordinates, the rotational EMF does not affect the core-loss
            # branch
            u_s_dq = inp.u_s_ab * exp_j_theta_m.conjugate()
            i_s_dq = (i_s_dq + par.G_c * u_s_dq) / (1 + par.G_c * par.R_s)
        i_s_ab = i_s_dq * exp_j_theta_m
        tau_M = par.n_p * tau_m
        return i_s_dq, i_s_ab, tau_M

    def set_states(self, state_list: list[complex], index: int) -> int:
        """Set states, keeping the rotor angle within [-pi, pi)."""
        # The angle is wrapped only in the state of the model, which the next
        # integration interval starts from, so the solver sees no discontinuity
        index = super().set_states(state_list, index)
        self.state.theta_m = wrap(self.state.theta_m.real)
        return index

    def set_outputs(self, t: float) -> None:
        """Set output variables."""
        out, state = self.out, self.state
        out.i_s_dq, out.i_s_ab, out.tau_M = self._compute_outputs(
            state, self.inp, exp(1j * state.theta_m.real)
        )

    def rhs(self, t: float) -> list[complex]:
        """Compute state derivatives."""
        # Rotor coordinates are internally used
        state, inp, out, par = self.state, self.inp, self.out, self.par
        u_s_dq = inp.u_s_ab * exp(-1j * state.theta_m.real)
        w_m = par.n_p * inp.w_M
        d_psi_s_dq = u_s_dq - par.R_s * out.i_s_dq - 1j * w_m * state.psi_s_dq
        d_theta_m = w_m
        return [d_psi_s_dq, d_theta_m]

    def compute_ac_port(self) -> ACPort:
        """Compute the AC port for a converter with blocked gates, see `ACPort`."""
        state, inp, par = self.state, self.inp, self.par
        if par.G_c:
            raise NotImplementedError("Gate blocking is not supported with core losses")
        return _compute_ac_port(
            lambda psi, ej: par.magnetic_map(psi, ej)[0],
            state.psi_s_dq,
            state.theta_m.real,
            par.n_p * inp.w_M,
            par.R_s,
        )

    def create_time_series(self, t: np.ndarray) -> "SynchronousMachineTimeSeries":
        """Create the time series."""
        return SynchronousMachineTimeSeries(t, self)


@dataclass
class SynchronousMachineTimeSeries(SubsystemTimeSeries):
    """Continuous time series."""

    t: InitVar[np.ndarray]
    subsystem: InitVar[SynchronousMachine]
    # States
    psi_s_dq: np.ndarray = field(default_factory=empty_array)
    theta_m: np.ndarray = field(default_factory=empty_array)
    # Outputs
    i_s_ab: np.ndarray = field(default_factory=empty_array)
    i_s_dq: np.ndarray = field(default_factory=empty_array)
    tau_M: np.ndarray = field(default_factory=empty_array)
    # Inputs
    u_s_ab: np.ndarray = field(default_factory=empty_array)
    w_M: np.ndarray = field(default_factory=empty_array)
    # Derived signals
    psi_s_ab: np.ndarray = field(default_factory=empty_array)
    exp_j_theta_m: np.ndarray = field(default_factory=empty_array)
    w_m: np.ndarray = field(default_factory=empty_array)

    def __post_init__(self, t: np.ndarray, subsystem: SynchronousMachine) -> None:
        """Compute time series from states."""
        self.psi_s_dq = np.array(subsystem._history["psi_s_dq"])
        theta_m = np.real(np.array(subsystem._history["theta_m"]))
        self.exp_j_theta_m = np.exp(1j * theta_m)
        self.theta_m = np.angle(self.exp_j_theta_m)
        self.psi_s_ab = self.exp_j_theta_m * self.psi_s_dq

    def set_outputs(self, t: np.ndarray, subsystem: SynchronousMachine) -> None:
        """Compute the outputs, which depend on the voltage due to core losses."""
        i_s_dq, i_s_ab, tau_M = subsystem._compute_outputs(
            self, self, self.exp_j_theta_m
        )
        self.i_s_dq = np.asarray(i_s_dq)
        self.i_s_ab = np.asarray(i_s_ab)
        self.tau_M = np.asarray(tau_M)

    def compute_derived_signals(
        self, t: np.ndarray, subsystem: SynchronousMachine
    ) -> None:
        """Compute the electrical rotor speed."""
        self.w_m = subsystem.par.n_p * self.w_M
