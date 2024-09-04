"""Common dataclasses usable in models and control of machine drives."""

from abc import ABC
from dataclasses import dataclass

# Note: Union can be replaced by "|" in Python 3.10
from typing import Callable, Union

import numpy as np


# %%
@dataclass
class NominalValues:
    """
    Nominal values.

    Parameters
    ----------
    U : float
        Voltage (V, rms, line-line).
    I : float
        Current (A, rms).
    f : float
        Frequency (Hz).
    P : float
        Power (W).
    tau : float
        Torque (Nm).

    """
    U: float
    I: float  # noqa: E741
    f: float
    P: float
    tau: float


# %%
@dataclass
class BaseValues:
    # pylint: disable=too-many-instance-attributes
    """
    Base values.

    Parameters
    ----------
    u : float
        Voltage (V, peak, line-neutral).
    i : float
        Current (A, peak).
    w : float
        Angular frequency (rad/s).
    psi : float
        Flux linkage (Vs).
    p : float
        Power (W).
    Z : float
        Impedance (Ω).
    L : float
        Inductance (H).
    tau : float
        Torque (Nm).
    n_p : int
        Number of pole pairs.

    """
    u: float
    i: float
    w: float
    psi: float
    p: float
    Z: float
    L: float
    tau: float
    n_p: int

    @classmethod
    def from_nominal(cls, nom, n_p):
        """
        Compute base values from nominal values.

        Parameters
        ----------
        nom : NominalValues
            Nominal values containing the following fields:

                U : float
                    Voltage (V, rms, line-line).
                I : float
                    Current (A, rms).
                f : float
                    Frequency (Hz).

        n_p : int
            Number of pole pairs.

        Returns
        -------
        BaseValues
            Base values.

        Notes
        -----
        Notice that the nominal torque is larger than the base torque due to
        the power factor and efficiency being less than unity.

        """
        u = np.sqrt(2/3)*nom.U
        i = np.sqrt(2)*nom.I
        w = 2*np.pi*nom.f
        psi = u/w
        p = 1.5*u*i
        Z = u/i
        L = Z/w
        tau = n_p*p/w

        return cls(u=u, i=i, w=w, psi=psi, p=p, Z=Z, L=L, tau=tau, n_p=n_p)


# %%
@dataclass
class MachinePars(ABC):
    """
    Base class for electrical parameters of an electric machine.

    Parameters
    ----------
    n_p : int
        Number of pole pairs.
    R_s : float
        Stator resistance (Ω).

    """

    n_p: int = None
    R_s: float = None


# %%
@dataclass
class SynchronousMachinePars(MachinePars):
    """
    Synchronous machine parameters.

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

    L_d: float = None
    L_q: float = None
    psi_f: float = None


# %%
@dataclass
class InductionMachinePars(MachinePars):
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
    L_s : float | callable
        Stator inductance (H).

    """

    R_r: float = None
    L_ell: float = None
    # L_s: float | Callable = None  # Can be used in Python 3.10 and later
    L_s: Union[float, Callable] = None

    @classmethod
    def from_inv_gamma_model_pars(cls, par):
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
        g = par.L_M/(par.L_M + par.L_sgm)
        R_r, L_ell, L_s = par.R_R/g**2, par.L_sgm/g, par.L_M + par.L_sgm

        return cls(R_s=par.R_s, R_r=R_r, L_ell=L_ell, L_s=L_s, n_p=par.n_p)


# %%
@dataclass
class InductionMachineInvGammaPars(MachinePars):
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

    R_R: float = None
    L_sgm: float = None
    L_M: float = None

    @classmethod
    def from_gamma_model_pars(cls, par):
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
        g = par.L_s/(par.L_s + par.L_ell)
        R_R, L_sgm, L_M = g**2*par.R_r, g*par.L_ell, g*par.L_s

        return cls(R_s=par.R_s, R_R=R_R, L_sgm=L_sgm, L_M=L_M, n_p=par.n_p)


# %%
@dataclass
class TwoMassMechanicalSystemPars:
    """
    Two-mass mechanical system parameters.

    Parameters
    ----------
    J_M : float
        Motor moment of inertia (kgm²).
    J_L : float
        Load moment of inertia (kgm²).
    K_S : float
        Shaft torsional stiffness (Nm/rad).
    C_S : float
        Shaft torsional damping (Nm/(rad/s)).
    B_L : float | callable
        Friction coefficient (Nm/(rad/s)) that can be constant, corresponding
        to viscous friction, or an arbitrary function of the load speed. For
        example, choosing ``B_L = lambda w_L: k*abs(w_M)`` leads to the
        quadratic load torque ``k*w_L**2``. The default is ``B_L = 0``.

    """

    J_M: float = None
    J_L: float = None
    K_S: float = None
    C_S: float = None
    B_L: Union[float, Callable] = 0
