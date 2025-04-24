"""
Continuous-time models for electric machines.

Peak-valued complex space vectors are used. Quantities in stationary coordinates are
marked with ab and quantities in synchronous coordinates are marked with dq.

"""

from dataclasses import InitVar, dataclass, field
from typing import Any

import numpy as np

from motulator.common.model import Subsystem, SubsystemTimeSeries
from motulator.common.utils import complex2abc, empty_array, get_value
from motulator.drive.utils._parameters import (
    InductionMachineInvGammaPars,
    InductionMachinePars,
    SaturatedSynchronousMachinePars,
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

    Parameters
    ----------
    par : InductionMachinePars
        Machine parameters.

    Notes
    -----
    The Γ model is chosen here since it can be extended with the magnetic saturation
    model in a straightforward manner. If the magnetic saturation is omitted, the Γ
    model is mathematically identical to the inverse-Γ and T models [#Sle1989]_.

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
        i_s_ab, i_r_ab, tau_M = self.compute_outputs(self.state)
        self.out: InductionMachineOutputs = InductionMachineOutputs(
            i_s_ab=i_s_ab, i_r_ab=i_r_ab, tau_M=tau_M
        )
        self._history: InductionMachineStateHistory = InductionMachineStateHistory()

    def compute_outputs(self, state: Any) -> tuple[Any, Any, Any]:
        """Compute output variables."""
        L_s = get_value(self.par.L_s, abs(state.psi_s_ab))
        i_r_ab = (state.psi_r_ab - state.psi_s_ab) / self.par.L_ell
        i_s_ab = state.psi_s_ab / L_s - i_r_ab
        tau_M = 1.5 * self.par.n_p * np.imag(i_s_ab * np.conj(state.psi_s_ab))
        return i_s_ab, i_r_ab, tau_M

    def set_outputs(self, t: float) -> None:
        """Set output variables."""
        self.out.i_s_ab, self.out.i_r_ab, self.out.tau_M = self.compute_outputs(
            self.state
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
        self.i_s_ab, self.i_r_ab, self.tau_M = subsystem.compute_outputs(self)
        L_s = get_value(subsystem.par.L_s, np.abs(self.psi_s_ab))
        # Inverse-Γ quantities
        gamma = L_s / (L_s + subsystem.par.L_ell)
        self.psi_R_ab = gamma * self.psi_r_ab

    def compute_input_derived_signals(
        self, t: np.ndarray, subsystem: InductionMachine
    ) -> None:
        """Compute signals derived from inputs."""
        self.w_m = subsystem.par.n_p * self.w_M  # Electrical rotor speed


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

    par: InitVar[SynchronousMachinePars | SaturatedSynchronousMachinePars]
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
    simplest implementation. The interfaces are in stator coordinates.

    Parameters
    ----------
    par : SynchronousMachinePars | SaturatedSynchronousMachinePars
        Machine parameters. The magnetic saturation can be modeled by providing a
        nonlinear current map par.i_s_dq (callable).

    """

    def __init__(
        self, par: SynchronousMachinePars | SaturatedSynchronousMachinePars
    ) -> None:
        self.par = par
        self.inp: SynchronousMachineInputs = SynchronousMachineInputs()
        self.state: SynchronousMachineStates = SynchronousMachineStates(par)
        i_s_dq, i_s_ab, tau_M = self.compute_outputs(self.state)
        self.out: SynchronousMachineOutputs = SynchronousMachineOutputs(
            i_s_ab=i_s_ab, i_s_dq=i_s_dq, tau_M=tau_M
        )
        self._history: SynchronousMachineStateHistory = SynchronousMachineStateHistory()

    def compute_outputs(self, state: Any) -> tuple[Any, Any, Any]:
        """Compute output variables."""
        i_s_dq = self.par.i_s_dq(state.psi_s_dq)
        i_s_ab = i_s_dq * state.exp_j_theta_m
        tau_M = 1.5 * self.par.n_p * np.imag(i_s_dq * np.conj(state.psi_s_dq))
        return i_s_dq, i_s_ab, tau_M

    def set_outputs(self, t: float) -> None:
        """Set output variables."""
        self.out.i_s_dq, self.out.i_s_ab, self.out.tau_M = self.compute_outputs(
            self.state
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
    exp_j_theta_m: np.ndarray = field(default_factory=empty_array)
    # Outputs
    i_s_ab: np.ndarray = field(default_factory=empty_array)
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
        self.i_s_dq, self.i_s_ab, self.tau_M = subsystem.compute_outputs(self)
        self.psi_s_ab = self.exp_j_theta_m * self.psi_s_dq

    def compute_input_derived_signals(
        self, t: np.ndarray, subsystem: SynchronousMachine
    ) -> None:
        """Compute signals derived from inputs."""
        self.w_m = subsystem.par.n_p * self.w_M  # Electrical rotor speed
