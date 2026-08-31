"""
Continuous-time models for electric machines.

Peak-valued complex space vectors are used. Quantities in stationary coordinates are
marked with ab and quantities in synchronous coordinates are marked with dq.

"""

from dataclasses import InitVar, dataclass, field
from typing import Any

import numpy as np

from motulator.common.model import Subsystem, SubsystemTimeSeries
from motulator.common.utils._utils import complex2abc, empty_array, get_value
from motulator.drive.utils._parameters import (
    InductionMachineInvGammaPars,
    InductionMachinePars,
    SaturatedSynchronousMachinePars,
    SpatialSaturatedSynchronousMachinePars,
    SynchronousMachinePars,
)


# %%
@dataclass
class InductionMachineInputs:
    """Base class for machine inputs."""

    u_s_ab: complex = 0j
    w_M: float = 0.0  # Mechanical rotor speed (rad/s)


@dataclass
class InductionMachineOutputs:
    """Machine outputs."""

    i_s_ab: complex
    i_r_ab: complex
    tau_M: float


@dataclass
class InductionMachineStates:
    """State variables."""

    psi_s_ab: complex = 0j
    psi_r_ab: complex = 0j


@dataclass
class InductionMachineStateHistory:
    """State history."""

    psi_s_ab: list[complex] = field(default_factory=list)
    psi_r_ab: list[complex] = field(default_factory=list)


class InductionMachine(Subsystem):
    """
    Γ-equivalent model of an induction machine.

    An induction machine is modeled using the Γ-equivalent model [#Sle1989]_. The stator
    inductance `L_s` can either be constant or a function of the stator flux magnitude::

        L_s = L_s(abs(psi_s_ab))

    Optionally, eddy-current core losses can be modeled by means of a constant core-loss
    conductance `G_c`, connected in parallel with the magnetizing branch.

    Parameters
    ----------
    par : InductionMachinePars | InductionMachineInvGammaPars
        Machine parameters. Core losses are modeled if `par.G_c` is nonzero.

    Notes
    -----
    The Γ model is chosen here since it can be extended with the magnetic saturation
    model in a straightforward manner. If the magnetic saturation is omitted, the Γ
    model is mathematically identical to the inverse-Γ and T models [#Sle1989]_.

    The core-loss branch is located between the stator resistance and the magnetizing
    inductance, i.e., the voltage across `G_c` is `u_s_ab - R_s*i_s_ab`. Consequently,
    the stator current depends directly on the stator voltage. This algebraic loop is
    solved in a closed form. The electromagnetic torque is produced by the current
    flowing into the magnetic circuit, i.e., the core-loss current is excluded from the
    torque.

    References
    ----------
    .. [#Sle1989] Slemon, "Modelling of induction machines for electric
       drives," IEEE Trans. Ind. Appl., 1989, https://doi.org/10.1109/28.44251

    """

    def __init__(
        self, par: InductionMachinePars | InductionMachineInvGammaPars
    ) -> None:
        if isinstance(par, InductionMachineInvGammaPars):
            par = InductionMachinePars.from_inv_gamma_pars(par)
        self.par = par
        self.inp: InductionMachineInputs = InductionMachineInputs()
        self.state: InductionMachineStates = InductionMachineStates()
        i_s_ab, i_r_ab, tau_M = self.compute_outputs(self.state, self.inp)
        self.out: InductionMachineOutputs = InductionMachineOutputs(
            i_s_ab=i_s_ab, i_r_ab=i_r_ab, tau_M=tau_M
        )
        self._history: InductionMachineStateHistory = InductionMachineStateHistory()

    def compute_outputs(self, state: Any, inp: Any) -> tuple[Any, Any, Any]:
        """Compute output variables."""
        par = self.par
        L_s = get_value(par.L_s, abs(state.psi_s_ab))
        i_r_ab = (state.psi_r_ab - state.psi_s_ab) / par.L_ell
        # Current into the magnetic circuit, i.e., the core-loss current excluded
        i_m_ab = state.psi_s_ab / L_s - i_r_ab
        tau_M = 1.5 * par.n_p * np.imag(i_m_ab * np.conj(state.psi_s_ab))
        i_s_ab = (i_m_ab + par.G_c * inp.u_s_ab) / (1 + par.G_c * par.R_s)
        return i_s_ab, i_r_ab, tau_M

    def set_outputs(self, t: float) -> None:
        """Set output variables."""
        self.out.i_s_ab, self.out.i_r_ab, self.out.tau_M = self.compute_outputs(
            self.state, self.inp
        )

    def rhs(self, t: float) -> list[complex]:
        """Compute state derivatives."""
        state, inp, out, par = self.state, self.inp, self.out, self.par
        d_psi_s_ab = inp.u_s_ab - par.R_s * out.i_s_ab
        d_psi_r_ab = -par.R_r * out.i_r_ab + 1j * par.n_p * inp.w_M * state.psi_r_ab
        return [d_psi_s_ab, d_psi_r_ab]

    def meas_currents(self) -> Any:
        """Measure phase currents (A)."""
        return complex2abc(self.out.i_s_ab)

    def create_time_series(
        self, t: np.ndarray
    ) -> tuple[str, "InductionMachineTimeSeries"]:
        """Create time series from state list."""
        return "machine", InductionMachineTimeSeries(t, self)


@dataclass
class InductionMachineTimeSeries(SubsystemTimeSeries):
    """Continuous time series."""

    t: InitVar[np.ndarray]
    subsystem: InitVar[InductionMachine]
    # States
    psi_s_ab: np.ndarray = field(default_factory=empty_array)
    psi_r_ab: np.ndarray = field(default_factory=empty_array)
    # Outputs
    i_s_ab: np.ndarray = field(default_factory=empty_array)
    i_r_ab: np.ndarray = field(default_factory=empty_array)
    tau_M: np.ndarray = field(default_factory=empty_array)
    # Inputs
    u_s_ab: np.ndarray = field(default_factory=empty_array)
    w_M: np.ndarray = field(default_factory=empty_array)
    # Derived signals
    psi_R_ab: np.ndarray = field(default_factory=empty_array)
    w_m: np.ndarray = field(default_factory=empty_array)

    def __post_init__(self, t: np.ndarray, subsystem: InductionMachine) -> None:
        """Compute output time series from the states."""
        self.psi_s_ab = np.array(subsystem._history.psi_s_ab)
        self.psi_r_ab = np.array(subsystem._history.psi_r_ab)
        # Provisional stator current, since u_s_ab needed for the core-loss current is
        # not yet available; corrected in compute_input_derived_signals
        self.i_s_ab, self.i_r_ab, self.tau_M = subsystem.compute_outputs(
            self, InductionMachineInputs()
        )
        L_s = get_value(subsystem.par.L_s, np.abs(self.psi_s_ab))
        # Inverse-Γ quantities
        gamma = L_s / (L_s + subsystem.par.L_ell)
        self.psi_R_ab = gamma * self.psi_r_ab

    def compute_input_derived_signals(
        self, t: np.ndarray, subsystem: InductionMachine
    ) -> None:
        """Compute signals derived from inputs."""
        self.w_m = subsystem.par.n_p * self.w_M  # Electrical rotor speed
        if subsystem.par.G_c:
            self.i_s_ab, _, _ = subsystem.compute_outputs(self, self)


# %%
@dataclass
class SynchronousMachineInputs:
    """Machine inputs."""

    u_s_ab: complex = 0j
    w_M: float = 0.0  # Mechanical rotor speed (rad/s)


@dataclass
class SynchronousMachineOutputs:
    """Output variables for interconnection."""

    i_s_ab: complex
    i_s_dq: complex
    tau_M: float


@dataclass
class SynchronousMachineStates:
    """State variables."""

    par: InitVar[
        SynchronousMachinePars
        | SaturatedSynchronousMachinePars
        | SpatialSaturatedSynchronousMachinePars
    ]
    psi_s_dq: complex = 0j
    exp_j_theta_m: complex = complex(1)

    def __post_init__(self, par) -> None:
        self.psi_s_dq = complex(par.psi_f)


@dataclass
class SynchronousMachineStateHistory:
    """State history."""

    psi_s_dq: list[complex] = field(default_factory=list)
    exp_j_theta_m: list[complex] = field(default_factory=list)


class SynchronousMachine(Subsystem):
    """
    Synchronous machine model.

    This model is internally represented in rotor coordinates, which results in the
    simplest implementation. The interfaces are in stator coordinates. The magnetic
    saturation can be modeled by providing a nonlinear current map `par.i_s_dq`.
    Optionally, eddy-current core losses can be modeled by means of a constant core-loss
    conductance `G_c`, connected in parallel with the magnetizing branch.

    Parameters
    ----------
    par : SynchronousMachinePars | SaturatedSynchronousMachinePars \
        | SpatialSaturatedSynchronousMachinePars
        Machine parameters. Core losses are modeled if `par.G_c` is nonzero.

    Notes
    -----
    The core-loss branch is located between the stator resistance and the magnetizing
    branch, i.e., the voltage across `G_c` is `u_s - R_s*i_s`. Consequently, the stator
    current depends directly on the stator voltage. This algebraic loop is solved in a
    closed form. The electromagnetic torque is produced by the current flowing into the
    magnetic circuit, i.e., the core-loss current is excluded from the torque.

    """

    def __init__(
        self,
        par: SynchronousMachinePars
        | SaturatedSynchronousMachinePars
        | SpatialSaturatedSynchronousMachinePars,
    ) -> None:
        self.par = par
        self.inp: SynchronousMachineInputs = SynchronousMachineInputs()
        self.state: SynchronousMachineStates = SynchronousMachineStates(par)
        i_s_dq, i_s_ab, tau_M = self.compute_outputs(self.state, self.inp)
        self.out: SynchronousMachineOutputs = SynchronousMachineOutputs(
            i_s_ab=i_s_ab, i_s_dq=i_s_dq, tau_M=tau_M
        )
        self._history: SynchronousMachineStateHistory = SynchronousMachineStateHistory()

    def compute_outputs(self, state: Any, inp: Any) -> tuple[Any, Any, Any]:
        """Compute output variables."""
        par = self.par
        # Current into the magnetic circuit, i.e., the core-loss current excluded
        i_m_dq, tau_m = par.magnetic_map(state.psi_s_dq, state.exp_j_theta_m)
        # In rotor coordinates, the rotational EMF does not affect the core-loss branch
        u_s_dq = inp.u_s_ab * np.conj(state.exp_j_theta_m)
        i_s_dq = (i_m_dq + par.G_c * u_s_dq) / (1 + par.G_c * par.R_s)
        i_s_ab = i_s_dq * state.exp_j_theta_m
        tau_M = par.n_p * tau_m
        return i_s_dq, i_s_ab, tau_M

    def set_outputs(self, t: float) -> None:
        """Set output variables."""
        self.out.i_s_dq, self.out.i_s_ab, self.out.tau_M = self.compute_outputs(
            self.state, self.inp
        )

    def rhs(self, t: float) -> list[complex]:
        """Compute state derivatives."""
        # Rotor coordinates are internally used
        state, inp, out, par = self.state, self.inp, self.out, self.par
        u_s_dq = inp.u_s_ab * np.conj(state.exp_j_theta_m)
        w_m = par.n_p * inp.w_M
        d_psi_s_dq = u_s_dq - par.R_s * out.i_s_dq - 1j * w_m * state.psi_s_dq
        d_exp_j_theta_m = 1j * w_m * state.exp_j_theta_m
        return [d_psi_s_dq, d_exp_j_theta_m]

    def meas_currents(self) -> Any:
        """Measure phase currents (A)."""
        return complex2abc(self.out.i_s_ab)

    def create_time_series(
        self, t: np.ndarray
    ) -> tuple[str, "SynchronousMachineTimeSeries"]:
        """Create time series from state list."""
        return "machine", SynchronousMachineTimeSeries(t, self)


@dataclass
class SynchronousMachineTimeSeries(SubsystemTimeSeries):
    """Continuous time series."""

    t: InitVar[np.ndarray]
    subsystem: InitVar[SynchronousMachine]
    # States
    psi_s_ab: np.ndarray = field(default_factory=empty_array)
    psi_s_dq: np.ndarray = field(default_factory=empty_array)
    exp_j_theta_m: np.ndarray = field(default_factory=empty_array)
    # Outputs
    i_s_ab: np.ndarray = field(default_factory=empty_array)
    i_s_dq: np.ndarray = field(default_factory=empty_array)
    tau_M: np.ndarray = field(default_factory=empty_array)
    # Inputs
    u_s_ab: np.ndarray = field(default_factory=empty_array)
    w_M: np.ndarray = field(default_factory=empty_array)
    # Derived signals
    w_m: np.ndarray = field(default_factory=empty_array)
    theta_m: np.ndarray = field(default_factory=empty_array)

    def __post_init__(self, t: np.ndarray, subsystem: SynchronousMachine) -> None:
        """Compute time series from states."""
        self.psi_s_dq = np.array(subsystem._history.psi_s_dq)
        self.exp_j_theta_m = np.array(subsystem._history.exp_j_theta_m)
        self.theta_m = np.angle(self.exp_j_theta_m)
        self.psi_s_ab = self.exp_j_theta_m * self.psi_s_dq
        # Provisional stator current, since u_s_ab needed for the core-loss current is
        # not yet available; corrected in compute_input_derived_signals
        i_s_dq, i_s_ab, tau_M = subsystem.compute_outputs(
            self, SynchronousMachineInputs()
        )
        self.i_s_dq = np.asarray(i_s_dq)
        self.i_s_ab = np.asarray(i_s_ab)
        self.tau_M = np.asarray(tau_M)

    def compute_input_derived_signals(
        self, t: np.ndarray, subsystem: SynchronousMachine
    ) -> None:
        """Compute signals derived from inputs."""
        self.w_m = subsystem.par.n_p * self.w_M  # Electrical rotor speed
        if subsystem.par.G_c:
            self.i_s_dq, self.i_s_ab, _ = subsystem.compute_outputs(self, self)
