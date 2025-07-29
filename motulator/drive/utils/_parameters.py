"""Common dataclasses usable in models and control of machine drives."""

from dataclasses import dataclass, field
from typing import Callable, Literal, Protocol

import numpy as np
from scipy.optimize import root, root_scalar


# %%
class BaseSynchronousMachinePars(Protocol):
    """Base class for synchronous machine parameters."""

    n_p: int
    R_s: float

    def i_s_dq(self, psi_s_dq: complex | np.ndarray) -> complex | np.ndarray:
        """Current (A) as a function of the flux linkage (Vs)."""
        ...

    def psi_s_dq(self, i_s_dq: complex | np.ndarray) -> complex | np.ndarray:
        """Flux linkage (Vs) as a function of the current (A)."""
        ...

    def inv_incr_ind_mat(self, psi_s_dq: complex | np.ndarray) -> np.ndarray:
        """Inverse of the incremental inductance matrix (1/H)."""
        ...

    def incr_ind_mat(self, i_s_dq: complex | np.ndarray) -> np.ndarray:
        """Incremental inductance matrix (H)."""
        ...

    def aux_current(self, psi_s_dq: complex | np.ndarray) -> complex | np.ndarray:
        """Auxiliary current (A) as function of the flux linkage (Vs)."""
        # This form is valid in the saturated case as well
        inv_L_s = self.inv_incr_ind_mat(psi_s_dq)
        G_dd = inv_L_s[0, 0]
        G_dq = inv_L_s[0, 1]
        G_qq = inv_L_s[1, 1]
        return complex(
            (G_qq * np.real(psi_s_dq) + 1j * G_dd * np.imag(psi_s_dq))
            - 1j * G_dq * np.conj(psi_s_dq)
            - self.i_s_dq(psi_s_dq)
        )

    def aux_flux(self, i_s_dq: complex | np.ndarray) -> complex | np.ndarray:
        """Auxiliary flux linkage (Vs) as function of the current (A)."""
        # This form is valid in the saturated case as well
        L_s = self.incr_ind_mat(i_s_dq)
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

    def i_s_dq(self, psi_s_dq: complex | np.ndarray) -> complex | np.ndarray:
        """Current (A) as a function of the flux linkage (Vs)."""
        i_s_dq = (np.real(psi_s_dq) - self.psi_f) / self.L_d + 1j * np.imag(
            psi_s_dq
        ) / self.L_q
        return i_s_dq

    def psi_s_dq(self, i_s_dq: complex | np.ndarray) -> complex | np.ndarray:
        """Flux linkage (Vs) as a function of the stator current (A)."""
        psi_s_dq = (
            self.L_d * np.real(i_s_dq) + 1j * self.L_q * np.imag(i_s_dq) + self.psi_f
        )
        return psi_s_dq

    def inv_incr_ind_mat(self, psi_s_dq: complex | np.ndarray) -> np.ndarray:
        """Inverse of the incremental inductance matrix (1/H)."""
        return np.array([[1 / self.L_d, 0], [0, 1 / self.L_q]])

    def incr_ind_mat(self, i_s_dq: complex | np.ndarray) -> np.ndarray:
        """Incremental inductance matrix (H)."""
        return np.array([[self.L_d, 0], [0, self.L_q]])


# %%
@dataclass
class SaturatedSynchronousMachinePars(BaseSynchronousMachinePars):
    """
    Parameters of a saturated synchronous machine.

    The saturation model is specified as as a current map (current as a function of the
    flux linkage). Optionally, to be used only in control systems, a flux map (flux
    linkage as a function of the current) can be provided. For convenience, this class
    also provides the incremental inductance matrix and its inverse, which can be used
    for the system model and optimal reference generation.

    Parameters
    ----------
    n_p : int
        Number of pole pairs.
    R_s : float
        Stator resistance (Ω).
    i_s_dq_fcn : Callable[[complex], complex]
        Stator current (A) as a function of the stator flux linkage (Vs). This function
        should be differentiable, if inverse incremental inductances are used.
    psi_s_dq_fcn : Callable[[complex], complex], optional
        Stator flux linkage (Vs) as a function of the stator current (A). This function
        should be differentiable, if incremental inductances are used. Needed only for
        some control methods, not in the system model. If not given, the modified
        Powell's method is used to iteratively compute the flux linkage.
    max_iter : int, optional
        Maximum number of iterations for the modified Powell's method, defaults to 20.
        This is needed only for some control methods (not for the system model) in such
        a case that `psi_s_dq_fcn` is not given.
    kind : str, optional
        Machine type, defaults to "pm". Allowed values are "pm" (permanent magnet) and
        "rel" (reluctance).

    """

    n_p: int
    R_s: float
    i_s_dq_fcn: Callable[[complex | np.ndarray], complex | np.ndarray]
    psi_s_dq_fcn: Callable[[complex | np.ndarray], complex | np.ndarray] | None = None
    kind: Literal["pm", "rel"] = "pm"
    max_iter: int = 20
    psi_f: float = field(init=False, default=0.0)

    def __post_init__(self) -> None:
        psi_f = root_scalar(
            lambda psi_d: np.real(self.i_s_dq(psi_d)), x0=0, method="newton"
        ).root
        self.psi_f = float(psi_f)

    def i_s_dq(self, psi_s_dq: complex | np.ndarray) -> complex | np.ndarray:
        """Current as a function of the flux linkage."""
        return self.i_s_dq_fcn(psi_s_dq)

    def psi_s_dq(self, i_s_dq: complex | np.ndarray) -> complex | np.ndarray:
        """Flux linkage as a function of the stator current."""
        if self.psi_s_dq_fcn is None:
            # For arrays, apply the solver to each element
            if isinstance(i_s_dq, np.ndarray):
                # Initialize result array with same shape as input
                result = np.zeros_like(i_s_dq, dtype=complex)

                # Inductances for initial guesses
                G = self.inv_incr_ind_mat(self.psi_f)
                L_d = 1 / G[0, 0]
                L_q = 1 / G[1, 1]

                # Apply solver to each element
                for idx in np.ndindex(i_s_dq.shape):
                    i_s = complex(i_s_dq[idx])
                    # Initial guess
                    psi_s_dq_init = (
                        L_d * np.real(i_s) + 1j * L_q * np.imag(i_s) + self.psi_f
                    )
                    # Solve for actual flux linkage and store in result array
                    result[idx] = self.solve_psi_s_dq(
                        i_s, psi_s_dq_init, max_iter=self.max_iter
                    )
                return result

            # For a single complex value
            G = self.inv_incr_ind_mat(self.psi_f)
            L_d = 1 / G[0, 0]
            L_q = 1 / G[1, 1]
            psi_s_dq_init = L_d * i_s_dq.real + 1j * L_q * i_s_dq.imag + self.psi_f
            return self.solve_psi_s_dq(i_s_dq, psi_s_dq_init, max_iter=self.max_iter)

        # Use the provided function if available
        return self.psi_s_dq_fcn(i_s_dq)

    def inv_incr_ind_mat(self, psi_s_dq: complex | np.ndarray) -> np.ndarray:
        """Inverse incremental inductance matrix vs. flux linkage."""
        eps = float(np.finfo(np.float16).eps)
        G_dd = (
            np.real(self.i_s_dq(psi_s_dq + eps)) - np.real(self.i_s_dq(psi_s_dq - eps))
        ) / (2 * eps)
        G_qq = (
            np.imag(self.i_s_dq(psi_s_dq + 1j * eps))
            - np.imag(self.i_s_dq(psi_s_dq - 1j * eps))
        ) / (2 * eps)
        G_dq = (
            np.real(self.i_s_dq(psi_s_dq + 1j * eps))
            - np.real(self.i_s_dq(psi_s_dq - 1j * eps))
        ) / (2 * eps)

        return np.array([[G_dd, G_dq], [G_dq, G_qq]])

    def incr_ind_mat(self, i_s_dq: complex | np.ndarray) -> np.ndarray:
        """Incremental inductance matrix vs. current."""
        eps = float(np.finfo(np.float16).eps)
        L_dd = (
            np.real(self.psi_s_dq(i_s_dq + eps)) - np.real(self.psi_s_dq(i_s_dq - eps))
        ) / (2 * eps)
        L_qq = (
            np.imag(self.psi_s_dq(i_s_dq + 1j * eps))
            - np.imag(self.psi_s_dq(i_s_dq - 1j * eps))
        ) / (2 * eps)
        L_dq = (
            np.real(self.psi_s_dq(i_s_dq + 1j * eps))
            - np.real(self.psi_s_dq(i_s_dq - 1j * eps))
        ) / (2 * eps)
        return np.array([[L_dd, L_dq], [L_dq, L_qq]])

    def solve_psi_s_dq(
        self, i_s_dq_target: complex, psi_s_dq_init: complex, max_iter: int
    ) -> complex:
        """
        Solve for flux linkage given target current, accounting for cross-saturation.

        Parameters
        ----------
        i_s_dq_target : complex
            Target stator current (A)
        psi_s_dq_init : complex
            Initial guess for flux linkage (Vs).
        max_iter : int
            Maximum number of iterations.

        Returns
        -------
        complex
            Stator flux linkage (Vs) that produces the target current.

        """

        def error_fcn(x) -> list[float]:
            # x = [psi_d, psi_q]
            psi_s = complex(x[0], x[1])
            i_s = complex(self.i_s_dq(psi_s))
            return [i_s.real - i_s_dq_target.real, i_s.imag - i_s_dq_target.imag]

        x0 = [psi_s_dq_init.real, psi_s_dq_init.imag]
        result = root(error_fcn, x0, method="hybr", options={"maxfev": max_iter})

        return complex(result.x[0], result.x[1])


# %%
@dataclass
class InductionMachinePars:
    """
    Γ-model parameters of an induction machine.

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

    """

    n_p: int
    R_s: float
    R_r: float
    L_ell: float
    L_s: float | Callable[[float], float]

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
    Inverse-Γ model parameters of an induction machine.

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

    """

    n_p: int
    R_s: float
    R_R: float
    L_sgm: float
    L_M: float

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
            raise ValueError
        g = par.L_s / (par.L_s + par.L_ell)
        R_R = g**2 * par.R_r
        L_sgm = g * par.L_ell
        L_M = g * par.L_s
        return cls(R_s=par.R_s, R_R=R_R, L_sgm=L_sgm, L_M=L_M, n_p=par.n_p)
