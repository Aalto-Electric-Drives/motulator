"""Common dataclasses usable in models and control of machine drives."""

from dataclasses import dataclass, field
from typing import Callable, Protocol, Tuple

import numpy as np
from scipy.optimize import root, root_scalar

EPS: float = 1e-3


# %%
class BaseSynchronousMachinePars(Protocol):
    """Base class for synchronous machine parameters."""

    n_p: int
    R_s: float

    def magnetic_map(
        self,
        psi_s_dq: complex | np.ndarray,
        exp_j_theta_m: complex | np.ndarray | None = None,
    ) -> Tuple[complex | np.ndarray, float | np.ndarray]:
        """
        Magnetic map as a function of flux linkage and electrical rotor angle.

        This method returns the stator current and the electromagnetic torque per pole
        pair as functions of the stator flux linkage and, optionally, the rotor angle.
        The allows a unified interface for the system model.

        Parameters
        ----------
        psi_s_dq : complex | ndarray
            Stator flux linkage (Vs) in rotor coordinates.
        exp_j_theta_m : complex | ndarray, optional
            Complex exponential of the electrical rotor angle.

        Returns
        -------
        Tuple [complex | ndarray, float | ndarray]
            Stator current (A) in rotor coordinates and electromagnetic torque (Nm) per
            pole pair.

        """
        i_s_dq = self.i_s_dq(psi_s_dq, exp_j_theta_m)
        tau_m = 1.5 * np.imag(i_s_dq * np.conj(psi_s_dq))
        return i_s_dq, tau_m

    def i_s_dq(
        self,
        psi_s_dq: complex | np.ndarray,
        exp_j_theta_m: complex | np.ndarray | None = None,
    ) -> complex | np.ndarray:
        """
        Current as a function of flux linkage and electrical rotor angle.

        Parameters
        ----------
        psi_s_dq : complex | ndarray
            Stator flux linkage (Vs) in rotor coordinates.
        exp_j_theta_m : complex | ndarray, optional
            Complex exponential of the electrical rotor angle.

        Returns
        -------
        complex | ndarray
            Stator current (A) in rotor coordinates.

        """
        ...

    def psi_s_dq(
        self,
        i_s_dq: complex | np.ndarray,
        exp_j_theta_m: complex | np.ndarray | None = None,
    ) -> complex | np.ndarray:
        """
        Flux linkage as a function of current and electrical rotor angle.

        Parameters
        ----------
        i_s_dq : complex | ndarray
            Stator current (A) in rotor coordinates.
        exp_j_theta_m : complex | ndarray, optional
            Complex exponential of the electrical rotor angle.

        Returns
        -------
        complex | ndarray
            Stator flux linkage in rotor coordinates (Vs).

        """
        ...

    def incr_ind_mat(
        self,
        i_s_dq: complex | np.ndarray,
        exp_j_theta_m: complex | np.ndarray | None = None,
    ) -> np.ndarray:
        """
        Incremental inductance matrix.

        Parameters
        ----------
        i_s_dq : complex | ndarray
            Stator current (A) in rotor coordinates.
        exp_j_theta_m : complex | ndarray, optional
            Complex exponential of the electrical rotor angle.

        Returns
        -------
        ndarray
            Incremental inductance matrix (H).

        """
        ...

    def aux_flux(
        self,
        i_s_dq: complex | np.ndarray,
        exp_j_theta_m: complex | np.ndarray | None = None,
    ) -> complex | np.ndarray:
        """
        Auxiliary flux linkage as a function of current.

        Parameters
        ----------
        i_s_dq : complex | ndarray
            Stator current (A) in rotor coordinates.
        exp_j_theta_m : complex | np.ndarray, optional
            Complex exponential of the electrical rotor angle.

        Returns
        -------
        complex | ndarray
            Auxiliary flux linkage (Vs).

        """
        # This form is valid in the saturated case as well
        L_s = self.incr_ind_mat(i_s_dq, exp_j_theta_m)
        L_dd = L_s[0, 0]
        L_dq = L_s[0, 1]
        L_qq = L_s[1, 1]
        psi_s_dq = complex(self.psi_s_dq(i_s_dq, exp_j_theta_m))
        return (
            psi_s_dq
            - L_qq * np.real(i_s_dq)
            - 1j * L_dd * np.imag(i_s_dq)
            + 1j * L_dq * np.conj(i_s_dq)
        )

    def aux_current(
        self,
        i_s_dq: complex | np.ndarray,
        exp_j_theta_m: complex | np.ndarray | None = None,
    ) -> complex | np.ndarray:
        """
        Auxiliary current as a function of current.

        Parameters
        ----------
        i_s_dq : complex | ndarray
            Stator current (A) in rotor coordinates.
        exp_j_theta_m : complex | ndarray, optional
            Complex exponential of the electrical rotor angle.

        Returns
        -------
        complex | ndarray
            Auxiliary current (A).

        """
        # This form is valid in the saturated case as well
        L_s = self.incr_ind_mat(i_s_dq, exp_j_theta_m)
        inv_L_s = np.linalg.inv(L_s)
        G_dd = inv_L_s[0, 0]
        G_dq = inv_L_s[0, 1]
        G_qq = inv_L_s[1, 1]
        psi_s_dq = complex(self.psi_s_dq(i_s_dq, exp_j_theta_m))
        return (
            (G_qq * np.real(psi_s_dq) + 1j * G_dd * np.imag(psi_s_dq))
            - 1j * G_dq * np.conj(psi_s_dq)
            - i_s_dq
        )

    def iterate_i_s_dq(self, psi_s_dq: complex) -> complex:
        """Solve for the current given the flux linkage using root finding."""
        ...


# %%
@dataclass
class SynchronousMachinePars(BaseSynchronousMachinePars):
    """
    Synchronous machine parameters, without saturation.

    Parameters
    ----------
    n_p : int
        Number of pole pairs.
    R_s : float
        Stator resistance (Ω).
    L_d : float
        d-axis inductance (H).
    L_q : float
        q-axis inductance (H).
    psi_f : float
        Permanent-magnet flux linkage (Vs).

    """

    n_p: int
    R_s: float
    L_d: float
    L_q: float
    psi_f: float

    def i_s_dq(
        self, psi_s_dq: complex | np.ndarray, exp_j_theta_m=None
    ) -> complex | np.ndarray:
        """Current (A) as a function of the flux linkage (Vs)."""
        i_d = (np.real(psi_s_dq) - self.psi_f) / self.L_d
        i_q = np.imag(psi_s_dq) / self.L_q
        i_s_dq = i_d + 1j * i_q
        return i_s_dq

    def psi_s_dq(
        self, i_s_dq: complex | np.ndarray, exp_j_theta_m=None
    ) -> complex | np.ndarray:
        """Flux linkage (Vs) as a function of the stator current (A)."""
        psi_d = self.L_d * np.real(i_s_dq) + self.psi_f
        psi_q = self.L_q * np.imag(i_s_dq)
        psi_s_dq = psi_d + 1j * psi_q
        return psi_s_dq

    def incr_ind_mat(
        self, i_s_dq: complex | np.ndarray, exp_j_theta_m=None
    ) -> np.ndarray:
        """Incremental inductance matrix (H)."""
        return np.array([[self.L_d, 0], [0, self.L_q]])

    def iterate_i_s_dq(self, psi_s_dq: complex) -> complex:
        """Compute the current from the flux linkage using root finding."""
        return complex(self.i_s_dq(psi_s_dq))


# %%
@dataclass
class SaturatedSynchronousMachinePars(BaseSynchronousMachinePars):
    """
    Parameters of a saturated synchronous machine.

    The saturation model is specified as a current map (current as a function of the
    flux linkage). Optionally, to be used only in control systems, a flux map (flux
    linkage as a function of the current) can be provided. For convenience, this class
    also provides the incremental inductance matrix, which can be used in control and
    optimal reference generation.

    Parameters
    ----------
    n_p : int
        Number of pole pairs.
    R_s : float
        Stator resistance (Ω).
    i_s_dq_fcn : Callable[[complex], complex], optional
        Stator current (A) as a function of the stator flux linkage (Vs). Needed in the
        system model and in some control methods.
    psi_s_dq_fcn : Callable[[complex], complex], optional
        Stator flux linkage (Vs) as a function of the stator current (A). This function
        should be differentiable, if incremental inductances are used. Needed only for
        control methods and optimal reference loci, not used in the system model.

    """

    n_p: int
    R_s: float
    i_s_dq_fcn: Callable[[complex | np.ndarray], complex | np.ndarray] | None = None
    psi_s_dq_fcn: Callable[[complex | np.ndarray], complex | np.ndarray] | None = None
    psi_f: float = field(init=False, default=0.0)
    L_d0: float = field(init=False)
    L_q0: float = field(init=False)

    def __post_init__(self) -> None:
        if self.i_s_dq_fcn is not None:
            self.psi_f = root_scalar(
                lambda psi_d: np.real(self.i_s_dq(psi_d)), x0=0, method="newton"
            ).root
        elif self.psi_s_dq_fcn is not None:
            self.psi_f = complex(self.psi_s_dq_fcn(0j)).real
            # Following are needed only for iterative current computation, if used
            self.L_d0 = self.incr_ind_mat(0j)[0, 0]
            self.L_q0 = self.incr_ind_mat(0j)[1, 1]
        else:
            raise ValueError("Either i_s_dq_fcn or psi_s_dq_fcn must be provided")
        if self.psi_f < EPS:  # No permanent magnets
            self.psi_f = 0.0

    def i_s_dq(
        self, psi_s_dq: complex | np.ndarray, exp_j_theta_m=None
    ) -> complex | np.ndarray:
        """Current (A) as a function of the flux linkage (Vs)."""
        if self.i_s_dq_fcn is not None:
            return self.i_s_dq_fcn(psi_s_dq)
        else:
            raise ValueError("i_s_dq_fcn must be provided")

    def psi_s_dq(
        self, i_s_dq: complex | np.ndarray, exp_j_theta_m=None
    ) -> complex | np.ndarray:
        """Flux linkage as a function of the stator current."""
        if self.psi_s_dq_fcn is not None:
            return self.psi_s_dq_fcn(i_s_dq)
        else:
            raise ValueError("psi_s_dq_fcn must be provided")

    def incr_ind_mat(
        self, i_s_dq: complex | np.ndarray, exp_j_theta_m=None
    ) -> np.ndarray:
        """Incremental inductance matrix at given current."""
        psi_dev_d = self.psi_s_dq(i_s_dq + EPS) - self.psi_s_dq(i_s_dq - EPS)
        psi_dev_q = self.psi_s_dq(i_s_dq + 1j * EPS) - self.psi_s_dq(i_s_dq - 1j * EPS)
        L_dd = np.real(psi_dev_d) / (2 * EPS)
        L_qq = np.imag(psi_dev_q) / (2 * EPS)
        L_dq = np.real(psi_dev_q) / (2 * EPS)
        return np.array([[L_dd, L_dq], [L_dq, L_qq]])

    def iterate_i_s_dq(self, psi_s_dq: complex) -> complex:
        """
        Compute the current from the flux linkage using root finding.

        The current is computed iteratively from the flux map using a root-finding
        algorithm. This is less efficient, but may be convenient in some special cases.

        """
        if self.i_s_dq_fcn is not None:
            return complex(self.i_s_dq(psi_s_dq))

        def error(x: list[float]) -> list[float]:
            i_s = x[0] + 1j * x[1]
            err = complex(self.psi_s_dq(i_s)) - psi_s_dq
            return [err.real, err.imag]

        i_s0 = (psi_s_dq.real - self.psi_f) / self.L_d0 + 1j * psi_s_dq.imag / self.L_q0
        sol = root(error, [i_s0.real, i_s0.imag], method="hybr", options={"maxfev": 50})
        return sol.x[0] + 1j * sol.x[1]


# %%
@dataclass
class SpatialSaturatedSynchronousMachinePars(BaseSynchronousMachinePars):
    """
    Parameters of a saturated synchronous machine with spatial harmonics.

    This saturation model contains spatial harmonics in addition to the saturation
    effects. This version is intended to parametrize high-fidelity machine models for
    simulation purposes, while control methods do not support spatial harmonics.

    Parameters
    ----------
    n_p : int
        Number of pole pairs.
    R_s : float
        Stator resistance (Ω).
    magnetic_map_fcn : Callable[[complex, complex], Tuple[complex, float]]
        Stator current (A) and electromagnetic torque (Nm) per pole pair as functions of
        the stator flux linkage (Vs) and the complex exponential of the electrical rotor
        angle.

    """

    n_p: int
    R_s: float
    magnetic_map_fcn: Callable[
        [complex | np.ndarray, complex | np.ndarray],
        Tuple[complex | np.ndarray, float | np.ndarray],
    ]
    psi_f: float = field(init=False, default=0.0)

    def __post_init__(self) -> None:
        self.psi_f = root_scalar(
            lambda psi_d: np.real(self.i_s_dq(psi_d, 1.0)), x0=0, method="newton"
        ).root
        if self.psi_f < EPS:  # No permanent magnets
            self.psi_f = 0.0

    def magnetic_map(
        self,
        psi_s_dq: complex | np.ndarray,
        exp_j_theta_m: complex | np.ndarray | None = None,
    ) -> Tuple[complex | np.ndarray, float | np.ndarray]:
        """
        Magnetic map.

        Parameters
        ----------
        psi_s_dq : complex | ndarray
            Stator flux linkage (Vs) in rotor coordinates.
        exp_j_theta_m : complex | ndarray
            Complex exponential of electrical rotor angle.

        Returns
        -------
        Tuple [complex | ndarray, float | ndarray]
            Stator current (A) in rotor coordinates and electromagnetic torque (Nm) per
            pole pair.

        """
        if exp_j_theta_m is None:
            raise ValueError("exp_j_theta_m must be provided")
        if self.magnetic_map_fcn is None:
            raise ValueError("magnetic_map_fcn must be provided")
        return self.magnetic_map_fcn(psi_s_dq, exp_j_theta_m)

    def i_s_dq(
        self,
        psi_s_dq: complex | np.ndarray,
        exp_j_theta_m: complex | np.ndarray | None = None,
    ) -> complex | np.ndarray:
        """Current (A) as a function of flux linkage (Vs) and rotor angle."""
        if exp_j_theta_m is None:
            raise ValueError("exp_j_theta_m must be provided")
        return self.magnetic_map_fcn(psi_s_dq, exp_j_theta_m)[0]

    def psi_s_dq(
        self, i_s_dq: complex | np.ndarray, exp_j_theta_m=None
    ) -> complex | np.ndarray:
        raise NotImplementedError("Flux map is not implemented")

    def incr_ind_mat(
        self, i_s_dq: complex | np.ndarray, exp_j_theta_m=None
    ) -> np.ndarray:
        raise NotImplementedError("Incremental inductance matrix is not implemented")

    def iterate_i_s_dq(self, psi_s_dq: complex) -> complex:
        raise NotImplementedError("This method is not implemented")


# %%
@dataclass
class InductionMachinePars:
    """
    Γ-model parameters of an induction machine.

    This contains Γ-model parameters of an induction machine. The main-flux saturation
    saturation can also be modeled by providing a callable `L_s` parameter. For
    convenience, the class also provides the corresponding inverse-Γ model parameters,
    which can be used in control systems. If the saturation is modeled, these inverse-Γ
    parameters depend on the stator flux linkage magnitude `psi_s` that should be
    updated by calling the `update_psi_s` method.

    Parameters
    ----------
    n_p : int
        Number of pole pairs.
    R_s : float
        Stator resistance (Ω).
    R_r : float
        Rotor resistance (Ω).
    L_ell : float
        Leakage inductance (H).
    L_s : float | Callable[[float], float]
        Stator inductance (H). If callable, it should be a function of the stator flux
        linkage magnitude (Vs).

    Attributes
    ----------
    gamma : float
        Magnetic coupling factor.
    L_M : float
        Inverse-Γ magnetizing inductance (H).
    L_sgm : float
        Inverse-Γ leakage inductance (H).
    R_R : float
        Inverse-Γ rotor resistance (Ω).
    R_sgm : float
        Inverse-Γ total resistance `R_s` plus `R_R` (Ω).
    alpha : float
        Inverse rotor time constant (rad/s).
    w_rb : float
        Breakdown slip angular frequency (rad/s).

    """

    n_p: int
    R_s: float
    R_r: float
    L_ell: float
    L_s: float | Callable[[float], float]
    psi_s: float = 0.0

    def update_psi_s(self, psi_s: float) -> None:
        """Update the stator flux linkage magnitude state."""
        self.psi_s = psi_s

    @property
    def gamma(self) -> float:
        if callable(self.L_s):
            return self.L_s(self.psi_s) / (self.L_s(self.psi_s) + self.L_ell)
        return self.L_s / (self.L_s + self.L_ell)

    @property
    def L_M(self) -> float:
        if callable(self.L_s):
            return self.gamma * self.L_s(self.psi_s)
        return self.gamma * self.L_s

    @property
    def L_sgm(self) -> float:
        return self.gamma * self.L_ell

    @property
    def R_R(self) -> float:
        return self.gamma**2 * self.R_r

    @property
    def R_sgm(self) -> float:
        return self.R_s + self.R_R

    @property
    def alpha(self) -> float:
        return self.R_R / self.L_M

    @property
    def w_rb(self) -> float:
        return self.R_r / self.L_ell if self.L_ell > 0 else 0.0

    @classmethod
    def from_inv_gamma_pars(
        cls, par: "InductionMachineInvGammaPars"
    ) -> "InductionMachinePars":
        """
        Compute Γ-model parameters from inverse-Γ model parameters.

        This transformation assumes that the parameters are constant.

        Parameters
        ----------
        par : InductionMachineInvGammaPars
            Inverse-Γ model parameters.

        Returns
        -------
        InductionMachinePars
            Γ model parameters.

        """
        g = par.L_M / (par.L_M + par.L_sgm)
        R_r = par.R_R / g**2
        L_ell = par.L_sgm / g
        L_s = par.L_M + par.L_sgm
        return cls(R_s=par.R_s, R_r=R_r, L_ell=L_ell, L_s=L_s, n_p=par.n_p)


# %%
@dataclass
class InductionMachineInvGammaPars:
    """
    Constant inverse-Γ model parameters of an induction machine.

    This contains constant inverse-Γ model parameters of an induction machine. To model
    the main-flux saturation, use the `InductionMachinePars` class instead.

    Parameters
    ----------
    n_p : int
        Number of pole pairs.
    R_s : float
        Stator resistance (Ω).
    R_R : float
        Rotor resistance (Ω).
    L_sgm : float
        Leakage inductance (H).
    L_M : float
        Magnetizing inductance (H).

    Attributes
    ----------
    R_sgm : float
        Inverse-Γ total resistance `R_s` plus `R_R` (Ω).
    alpha : float
        Inverse rotor time constant (rad/s).
    w_rb : float
        Breakdown slip angular frequency (rad/s).


    """

    n_p: int
    R_s: float
    R_R: float
    L_sgm: float
    L_M: float

    def update_psi_s(self, psi_s: float) -> None:
        """Update the stator flux linkage magnitude state."""
        # This method has no effect for constant parameters (no saturation)
        pass

    @property
    def R_sgm(self) -> float:
        return self.R_s + self.R_R

    @property
    def alpha(self) -> float:
        return self.R_R / self.L_M

    @property
    def w_rb(self) -> float:
        return self.R_R * (1 / self.L_sgm + 1 / self.L_M) if self.L_sgm > 0 else 0.0

    @classmethod
    def from_gamma_pars(
        cls, par: "InductionMachinePars"
    ) -> "InductionMachineInvGammaPars":
        """
        Compute inverse-Γ model parameters from Γ model parameters.

        This transformation assumes that the parameters are constant.

        Parameters
        ----------
        par : InductionMachinePars
            Γ-model parameters.

        Returns
        -------
        InductionMachineInvGammaPars
            Inverse-Γ model parameters.

        """
        if callable(par.L_s):
            raise ValueError("L_s must be constant for this transformation")
        g = par.L_s / (par.L_s + par.L_ell)
        R_R = g**2 * par.R_r
        L_sgm = g * par.L_ell
        L_M = g * par.L_s
        return cls(R_s=par.R_s, R_R=R_R, L_sgm=L_sgm, L_M=L_M, n_p=par.n_p)
