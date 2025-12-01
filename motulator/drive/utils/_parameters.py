"""Common dataclasses usable in models and control of machine drives."""

from dataclasses import dataclass, field
from typing import Callable, Literal, Protocol

import numpy as np
from scipy.optimize import root, root_scalar

EPS: float = 1e-6  # Finite-difference step for numerical derivatives


# %%
class BaseSynchronousMachinePars(Protocol):
    """Base class for synchronous machine parameters."""

    n_p: int
    R_s: float

    def i_s_dq(
        self,
        psi_s_dq: complex | np.ndarray,
        exp_j_theta_m: complex | np.ndarray | None = None,
    ) -> complex | np.ndarray:
        """
        Current as a function of flux linkage and rotor angle.

        Parameters
        ----------
        psi_s_dq : complex | ndarray
            Stator flux linkage (Vs) in rotor coordinates.
        exp_j_theta_m : complex | ndarray, optional
            Complex exponential of the electrical rotor angle.

        Returns
        -------
        complex | ndarray
            Stator current in rotor coordinates (A).

        """
        ...

    def tau_M(
        self,
        psi_s_dq: complex | np.ndarray,
        i_s_dq: complex | np.ndarray,
        exp_j_theta_m: complex | np.ndarray | None = None,
    ) -> float | np.ndarray:
        """
        Electromagnetic torque.

        Parameters
        ----------
        psi_s_dq : complex | ndarray
            Stator flux linkage (Vs) in rotor coordinates.
        i_s_dq : complex | ndarray
            Stator current (A) in rotor coordinates.
        exp_j_theta_m : complex | ndarray, optional
            Complex exponential of the electrical rotor angle.

        Returns
        -------
        float | ndarray
            Electromagnetic torque (Nm).

        Notes
        -----
        The default implementation assumes no spatial harmonics. This method can be
        overridden in subclasses to model machines with spatial harmonics.

        """
        tau_M = 1.5 * self.n_p * np.imag(i_s_dq * np.conj(psi_s_dq))
        return tau_M

    def inv_incr_ind_mat(
        self,
        psi_s_dq: complex | np.ndarray,
        exp_j_theta_m: complex | np.ndarray | None = None,
    ) -> np.ndarray:
        """
        Inverse of the incremental inductance matrix.

        Parameters
        ----------
        psi_s_dq : complex | ndarray
            Stator flux linkage (Vs) in rotor coordinates.
        exp_j_theta_m : complex | ndarray, optional
            Complex exponential of the electrical rotor angle.

        Returns
        -------
        ndarray
            Inverse of the incremental inductance matrix (1/H).

        """
        ...

    def aux_current(
        self,
        psi_s_dq: complex | np.ndarray,
        exp_j_theta_m: complex | np.ndarray | None = None,
    ) -> complex | np.ndarray:
        """
        Auxiliary current as a function of flux linkage.

        Parameters
        ----------
        psi_s_dq : complex | ndarray
            Stator flux linkage (Vs) in rotor coordinates.
        exp_j_theta_m : complex | ndarray, optional
            Complex exponential of the electrical rotor angle.

        Returns
        -------
        complex | ndarray
            Auxiliary current (A).

        """
        # This form is valid in the saturated case as well
        inv_L_s = self.inv_incr_ind_mat(psi_s_dq, exp_j_theta_m)
        G_dd = inv_L_s[0, 0]
        G_dq = inv_L_s[0, 1]
        G_qq = inv_L_s[1, 1]
        return complex(
            (G_qq * np.real(psi_s_dq) + 1j * G_dd * np.imag(psi_s_dq))
            - 1j * G_dq * np.conj(psi_s_dq)
            - self.i_s_dq(psi_s_dq)
        )

    def psi_s_dq(
        self,
        i_s_dq: complex | np.ndarray,
        exp_j_theta_m: complex | np.ndarray | None = None,
    ) -> complex | np.ndarray:
        """
        Flux linkage as a function of the current and rotor angle.

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
        return complex(
            self.psi_s_dq(i_s_dq)
            - L_qq * np.real(i_s_dq)
            - 1j * L_dd * np.imag(i_s_dq)
            + 1j * L_dq * np.conj(i_s_dq)
        )


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
    kind : str, optional
        Machine type, defaults to "pm". Allowed values are "pm" (permanent magnet) and
        "rel" (reluctance).

    """

    n_p: int
    R_s: float
    L_d: float
    L_q: float
    psi_f: float
    kind: Literal["pm", "rel"] = "pm"

    def i_s_dq(
        self, psi_s_dq: complex | np.ndarray, exp_j_theta_m=None
    ) -> complex | np.ndarray:
        """Current (A) as a function of the flux linkage (Vs)."""
        i_s_dq = (np.real(psi_s_dq) - self.psi_f) / self.L_d + 1j * np.imag(
            psi_s_dq
        ) / self.L_q
        return i_s_dq

    def psi_s_dq(
        self, i_s_dq: complex | np.ndarray, exp_j_theta_m=None
    ) -> complex | np.ndarray:
        """Flux linkage (Vs) as a function of the stator current (A)."""
        psi_s_dq = (
            self.L_d * np.real(i_s_dq) + 1j * self.L_q * np.imag(i_s_dq) + self.psi_f
        )
        return psi_s_dq

    def inv_incr_ind_mat(
        self, psi_s_dq: complex | np.ndarray, exp_j_theta_m=None
    ) -> np.ndarray:
        """Inverse of the incremental inductance matrix (1/H)."""
        return np.array([[1 / self.L_d, 0], [0, 1 / self.L_q]])

    def incr_ind_mat(
        self, i_s_dq: complex | np.ndarray, exp_j_theta_m=None
    ) -> np.ndarray:
        """Incremental inductance matrix (H)."""
        return np.array([[self.L_d, 0], [0, self.L_q]])


# %%
@dataclass
class SaturatedSynchronousMachinePars(BaseSynchronousMachinePars):
    """
    Parameters of a saturated synchronous machine.

    The saturation model is specified as a current map (current as a function of the
    flux linkage). Optionally, to be used only in control systems, a flux map (flux
    linkage as a function of the current) can be provided. For convenience, this class
    also provides the incremental inductance matrix and its inverse, which can be used
    in control systems and optimal reference generation.

    Parameters
    ----------
    n_p : int
        Number of pole pairs.
    R_s : float
        Stator resistance (Ω).
    i_s_dq_fcn : Callable[[complex], complex], optional
        Stator current (A) as a function of the stator flux linkage (Vs). This function
        should be differentiable, if inverse incremental inductances are used. Needed
        in the system model and in some control methods.
    psi_s_dq_fcn : Callable[[complex], complex], optional
        Stator flux linkage (Vs) as a function of the stator current (A). This function
        should be differentiable, if incremental inductances are used. Needed only for
        some control methods, not in the system model.
    kind : str, optional
        Machine type, defaults to "pm". Allowed values are "pm" (permanent magnet) and
        "rel" (reluctance).
    max_iter : int, optional
        Maximum number of iterations, defaults to None. Value around 20 typically
        suffices. Note that the iterative method is intended for development purposes.

    Notes
    -----
    The class allows providing either `i_s_dq_fcn` or `psi_s_dq_fcn`. If only one of
    them is provided and `max_iter` is given, the other one is computed iteratively.
    This feature is intended for development purposes. It can be used in control
    systems, but iteration increases the simulations time and may not be computationally
    practical in real-time control.

    """

    n_p: int
    R_s: float
    i_s_dq_fcn: Callable[[complex | np.ndarray], complex | np.ndarray] | None = None
    psi_s_dq_fcn: Callable[[complex | np.ndarray], complex | np.ndarray] | None = None
    kind: Literal["pm", "rel"] = "pm"
    psi_f: float = field(init=False, default=0.0)
    max_iter: int | None = None

    def __post_init__(self) -> None:
        if self.i_s_dq_fcn is not None:
            psi_f = root_scalar(
                lambda psi_d: np.real(self.i_s_dq(psi_d)), x0=0, method="newton"
            ).root
            self.psi_f = float(psi_f)
        elif self.psi_s_dq_fcn is not None:
            self.psi_f = complex(self.psi_s_dq_fcn(0j)).real
        else:
            raise ValueError("Either i_s_dq_fcn or psi_s_dq_fcn must be provided")

    def i_s_dq(
        self, psi_s_dq: complex | np.ndarray, exp_j_theta_m=None
    ) -> complex | np.ndarray:
        """Current (A) as a function of the flux linkage (Vs)."""
        if self.i_s_dq_fcn is not None:
            return self.i_s_dq_fcn(psi_s_dq)
        elif self.psi_s_dq_fcn is not None and self.max_iter is not None:
            return self._solve_inverse(psi_s_dq, self._solve_current_single)
        else:
            raise ValueError("Either i_s_dq_fcn or psi_s_dq_fcn must be provided")

    def psi_s_dq(
        self, i_s_dq: complex | np.ndarray, exp_j_theta_m=None
    ) -> complex | np.ndarray:
        """Flux linkage as a function of the stator current."""
        if self.psi_s_dq_fcn is not None:
            return self.psi_s_dq_fcn(i_s_dq)
        elif self.i_s_dq_fcn is not None and self.max_iter is not None:
            return self._solve_inverse(i_s_dq, self._solve_flux_single)
        else:
            raise ValueError("Either i_s_dq_fcn or psi_s_dq_fcn must be provided")

    def inv_incr_ind_mat(
        self, psi_s_dq: complex | np.ndarray, exp_j_theta_m=None
    ) -> np.ndarray:
        """Inverse incremental inductance matrix vs. flux linkage."""
        G_dd = (
            np.real(self.i_s_dq(psi_s_dq + EPS)) - np.real(self.i_s_dq(psi_s_dq - EPS))
        ) / (2 * EPS)
        G_qq = (
            np.imag(self.i_s_dq(psi_s_dq + 1j * EPS))
            - np.imag(self.i_s_dq(psi_s_dq - 1j * EPS))
        ) / (2 * EPS)
        G_dq = (
            np.real(self.i_s_dq(psi_s_dq + 1j * EPS))
            - np.real(self.i_s_dq(psi_s_dq - 1j * EPS))
        ) / (2 * EPS)

        return np.array([[G_dd, G_dq], [G_dq, G_qq]])

    def incr_ind_mat(
        self, i_s_dq: complex | np.ndarray, exp_j_theta_m=None
    ) -> np.ndarray:
        """Incremental inductance matrix vs. current."""
        L_dd = (
            np.real(self.psi_s_dq(i_s_dq + EPS)) - np.real(self.psi_s_dq(i_s_dq - EPS))
        ) / (2 * EPS)
        L_qq = (
            np.imag(self.psi_s_dq(i_s_dq + 1j * EPS))
            - np.imag(self.psi_s_dq(i_s_dq - 1j * EPS))
        ) / (2 * EPS)
        L_dq = (
            np.real(self.psi_s_dq(i_s_dq + 1j * EPS))
            - np.real(self.psi_s_dq(i_s_dq - 1j * EPS))
        ) / (2 * EPS)
        return np.array([[L_dd, L_dq], [L_dq, L_qq]])

    def _solve_inverse(
        self,
        input_val: complex | np.ndarray,
        solve_single: Callable[[complex], complex],
    ) -> complex | np.ndarray:
        """Handle both array and scalar inputs for iterative solving."""
        if isinstance(input_val, np.ndarray):
            result = np.zeros_like(input_val, dtype=complex)
            for idx in np.ndindex(input_val.shape):
                result[idx] = solve_single(complex(input_val[idx]))
            return result
        else:
            return solve_single(input_val)

    def _solve_current_single(self, psi_s_dq: complex) -> complex:
        """Solve for current given flux linkage."""
        if self.psi_s_dq_fcn is None:
            raise ValueError("psi_s_dq_fcn must be provided")
        # Initial guess using incremental inductance at zero current
        L = self.incr_ind_mat(0j)
        G_d = 1 / L[0, 0]
        G_q = 1 / L[1, 1]
        i_s_dq_init = G_d * (psi_s_dq.real - self.psi_f) + 1j * G_q * psi_s_dq.imag
        return self._solve_x(psi_s_dq, self.psi_s_dq_fcn, i_s_dq_init)

    def _solve_flux_single(self, i_s_dq: complex) -> complex:
        """Solve for flux linkage given current."""
        if self.i_s_dq_fcn is None:
            raise ValueError("i_s_dq_fcn must be provided")
        # Initial guess using inverse incremental inductance at psi_f
        G = self.inv_incr_ind_mat(self.psi_f)
        L_d = 1 / G[0, 0]
        L_q = 1 / G[1, 1]
        psi_s_dq_init = L_d * i_s_dq.real + 1j * L_q * i_s_dq.imag + self.psi_f
        return self._solve_x(i_s_dq, self.i_s_dq_fcn, psi_s_dq_init)

    def _solve_x(
        self,
        y: complex,
        f: Callable[[complex | np.ndarray], complex | np.ndarray],
        x_init: complex,
    ) -> complex:
        """
        Solve for x in y = f(x).

        Parameters
        ----------
        y : complex
            Target quantity.
        f : Callable[[complex], complex]
            Function that maps x to y.
        x_init : complex
            Initial guess.

        Returns
        -------
        complex
            x that produces y.

        """

        def error_fcn(x) -> list[float]:
            x = complex(x[0], x[1])
            y_eval = complex(f(x))
            return [y_eval.real - y.real, y_eval.imag - y.imag]

        x0 = [x_init.real, x_init.imag]
        result = root(error_fcn, x0, method="hybr", options={"maxfev": self.max_iter})

        return complex(result.x[0], result.x[1])


# %%
@dataclass
class SpatialSaturatedSynchronousMachinePars(BaseSynchronousMachinePars):
    """
    Parameters of a saturated synchronous machine with spatial harmonics.

    This saturation model contains spatial harmonics in addition to the saturation
    effects. This version is intended as a high-fidelity machine model for simulation
    purposes, while most control methods do not support spatial harmonics.

    Parameters
    ----------
    n_p : int
        Number of pole pairs.
    R_s : float
        Stator resistance (Ω).
    i_s_dq_fcn : Callable[[complex], complex]
        Stator current (A) as a function of the stator flux linkage (Vs) and the complex
        exponential of the rotor angle.
    tau_M_ripple_fcn : Callable[[complex], float]
        Torque ripple (Nm) as a function of the stator flux linkage (Vs) and the complex
        exponential of the rotor angle.
    kind : str, optional
        Machine type, defaults to "pm". Allowed values are "pm" (permanent magnet) and
        "rel" (reluctance).

    """

    n_p: int
    R_s: float
    i_s_dq_fcn: Callable[
        [complex | np.ndarray, complex | np.ndarray], complex | np.ndarray
    ]
    tau_M_ripple_fcn: Callable[
        [complex | np.ndarray, complex | np.ndarray], float | np.ndarray
    ]
    kind: Literal["pm", "rel"] = "pm"
    psi_f: float = field(init=False, default=0.0)

    def __post_init__(self) -> None:
        psi_f = root_scalar(
            lambda psi_d: np.real(self.i_s_dq(psi_d, 1.0)), x0=0, method="newton"
        ).root
        self.psi_f = float(psi_f)

    def i_s_dq(
        self,
        psi_s_dq: complex | np.ndarray,
        exp_j_theta_m: complex | np.ndarray | None = None,
    ) -> complex | np.ndarray:
        """
        Current as a function of flux linkage and rotor angle.

        Parameters
        ----------
        psi_s_dq : complex | ndarray
            Stator flux linkage (Vs) in rotor coordinates.
        exp_j_theta_m : complex | ndarray
            Complex exponential of electrical rotor angle.

        """
        if exp_j_theta_m is None:
            raise ValueError("exp_j_theta_m must be provided")
        if self.i_s_dq_fcn is None:
            raise ValueError("i_s_dq_fcn must be provided")
        return self.i_s_dq_fcn(psi_s_dq, exp_j_theta_m)

    def tau_M(
        self,
        psi_s_dq: complex | np.ndarray,
        i_s_dq: complex | np.ndarray,
        exp_j_theta_m: complex | np.ndarray | None = None,
    ) -> float | np.ndarray:
        """
        Torque as a function of flux linkage and rotor angle.

        Parameters
        ----------
        psi_s_dq : complex | ndarray
            Stator flux linkage (Vs) in rotor coordinates.
        i_s_dq : complex | ndarray
            Stator current (A) in rotor coordinates.
        exp_j_theta_m : complex | ndarray
            Complex exponential of electrical rotor angle.

        """
        if exp_j_theta_m is None:
            raise ValueError("exp_j_theta_m must be provided")
        if self.tau_M_ripple_fcn is None:
            raise ValueError("tau_M_ripple_fcn must be provided")

        tau_M_ripple = 1.5 * self.n_p * self.tau_M_ripple_fcn(psi_s_dq, exp_j_theta_m)
        return 1.5 * self.n_p * np.imag(i_s_dq * np.conj(psi_s_dq)) - tau_M_ripple

    def psi_s_dq(
        self, i_s_dq: complex | np.ndarray, exp_j_theta_m=None
    ) -> complex | np.ndarray:
        raise NotImplementedError("Flux map is not implemented")

    def inv_incr_ind_mat(
        self, psi_s_dq: complex | np.ndarray, exp_j_theta_m=None
    ) -> np.ndarray:
        raise NotImplementedError(
            "Inverse incremental inductance matrix is not implemented"
        )

    def incr_ind_mat(
        self, i_s_dq: complex | np.ndarray, exp_j_theta_m=None
    ) -> np.ndarray:
        raise NotImplementedError("Incremental inductance matrix is not implemented")


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
