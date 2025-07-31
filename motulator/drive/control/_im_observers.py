"""Common control functions and classes for for induction machine drives."""

from cmath import exp
from dataclasses import dataclass
from math import inf, pi
from typing import Callable

from motulator.common.utils._utils import wrap
from motulator.drive.utils._parameters import (
    InductionMachineInvGammaPars,
    InductionMachinePars,
)


# %%
@dataclass
class ObserverStates:
    """State estimates."""

    psi_s: complex = 0j
    theta_s: float = 0.0
    w_m: float = 0.0
    tau_L: float = 0.0


@dataclass
class ObserverWorkspace:
    """Workspace variables."""

    d_psi_s: complex = 0j
    d_theta_s: float = 0.0
    d_w_m: float = 0.0
    d_tau_L: float = 0.0
    old_i_s: complex = 0j
    eps: float = 0.0
    T_s: float = 0.0


@dataclass
class ObserverOutputs:
    """Feedback signals for the control system."""

    u_dc: float = 0.0
    i_s: complex = 0j
    u_s: complex = 0j
    psi_s: complex = 0j
    psi_R: complex = 0j
    tau_M: float = 0.0
    theta_s: float = 0.0
    w_s: float = 0.0
    w_r: float = 0.0
    w_m: float = 0.0
    w_M: float = 0.0


class FluxObserver:
    """
    Reduced-order flux observer.

    This class implements a reduced-order flux observer for induction machines. The
    observer structure is similar to [#Hin2010]_. The observer operates in synchronous
    coordinates (but not locked to any particular vector). The main-flux saturation can
    be taken into account by providing the saturation model via `InductionMachinePars`.

    Parameters
    ----------
    par : InductionMachineInvGammaPars | InductionMachinePars
        Machine model parameters.
    k_o1, k_o2 : Callable[[float], complex]
        Observer gains as functions of the electrical angular speed of the rotor.

    Notes
    -----
    The pure voltage model corresponds to ``k_o1 = lambda w_m: 0`` and `k_o2 = lambda
    w_m: 0``, resulting in the marginally stable estimation-error dynamics. The current
    model is obtained by setting ``k_o1 = lambda w_m: 1`` and `k_o2 = lambda w_m: 0``.

    References
    ----------
    .. [#Hin2010] Hinkkanen, Harnefors, Luomi, "Reduced-order flux observers with
       stator-resistance adaptation for speed-sensorless induction motor drives," IEEE
       Trans. Power Electron., 2010, https://doi.org/10.1109/TPEL.2009.2039650

    """

    def __init__(
        self,
        par: InductionMachinePars | InductionMachineInvGammaPars,
        k_o1: Callable[[float], complex],
        k_o2: Callable[[float], complex],
    ) -> None:
        self.par = par
        self.k_o1 = k_o1
        self.k_o2 = k_o2
        self.state = ObserverStates()
        self._work = ObserverWorkspace()

    def compute_output(
        self, u_s_ab: complex, i_s_ab: complex, w_M: float | None
    ) -> ObserverOutputs:
        """
        Compute the feedback signals for the control system.

        Parameters
        ----------
        u_s_ab : complex
            Stator voltage (V) in stator coordinates.
        i_s_ab : complex
            Stator current (A) in stator coordinates.
        w_M : float, optional
            Rotor speed (mechanical rad/s), either measured or estimated.

        Returns
        -------
        out : ObserverOutputs
            Estimated feedback signals for the control system.

        """
        # Unpack
        par = self.par

        # Initialize the output signals
        out = ObserverOutputs(psi_s=self.state.psi_s, theta_s=self.state.theta_s)

        # Current and voltage vectors in estimated rotor coordinates
        out.i_s = exp(-1j * out.theta_s) * i_s_ab
        out.u_s = exp(-1j * out.theta_s) * u_s_ab

        # Rotor flux estimate
        out.psi_R = out.psi_s - par.L_sgm * self._work.old_i_s

        # Derivative of the stator current
        T_s = self._work.T_s
        d_i_s = (out.i_s - self._work.old_i_s) / T_s if T_s > 0 else 0.0
        self._work.old_i_s = out.i_s  # Update the old value, safe to do here

        # Mechanical and electrical angular speeds of the rotor
        if w_M is None:
            raise ValueError
        out.w_M = w_M
        out.w_m = par.n_p * w_M

        # Slip angular frequency
        prod = out.psi_s * out.psi_R.conjugate()
        out.w_r = par.w_rb * prod.imag / prod.real if prod.real > 0 else 0.0

        # Synchronous angular frequency
        out.w_s = out.w_m + out.w_r

        # Estimation error
        e_o = (
            par.L_sgm * d_i_s
            - out.u_s
            + (par.R_sgm + 1j * out.w_s * par.L_sgm) * out.i_s
            - (par.alpha - 1j * out.w_m) * out.psi_R
        )

        # Observer gains
        k_o1 = self.k_o1(out.w_m)
        proj = out.psi_R / out.psi_R.conjugate() if abs(out.psi_R) > 0 else 0.0
        k_o2 = self.k_o2(out.w_m) * proj  # Inherently sensorless observer gain

        # Derivative of the stator flux
        v_err = k_o1 * e_o + k_o2 * e_o.conjugate()
        d_psi_s = out.u_s - par.R_s * out.i_s - 1j * out.w_s * out.psi_s + v_err

        # Torque estimate
        if par.L_sgm > 0:
            out.tau_M = 1.5 * par.n_p * (out.i_s * out.psi_s.conjugate()).imag
        else:  # Disable torque estimation in pure open-loop V/Hz control mode
            out.tau_M = 0

        # Speed estimation error for the speed observer
        self._work.eps = -(e_o / out.psi_R).imag if abs(out.psi_R) > 0 else 0.0

        # Store the derivatives for the update method
        self._work.d_psi_s = d_psi_s
        self._work.d_theta_s = out.w_s

        return out

    def update(self, T_s: float) -> None:
        """Update the state estimates."""
        self.state.psi_s += T_s * self._work.d_psi_s
        self.state.theta_s = wrap(self.state.theta_s + T_s * self._work.d_theta_s)
        self._work.T_s = T_s
        # Update the saturation model for the next sampling period
        self.par.update_psi_s(abs(self.state.psi_s))


# %%
class SpeedFluxObserver(FluxObserver):
    """
    Observer with speed estimation.

    This class implements a reduced-order flux observer for induction machines with
    speed estimation. If the inertia of the mechanical system is provided, the observer
    also estimates the load torque, to avoid the lag in the speed estimate.

    Parameters
    ----------
    par : InductionMachineInvGammaPars | InductionMachinePars
        Machine model parameters.
    k_o1, k_o2 : Callable[[float], complex]
        Observer gains as functions of the electrical angular speed of the rotor.
    alpha_o : float
        Speed estimation pole (rad/s).
    J : float, optional
        Inertia of the mechanical system (kgm²). Defaults to infinity, which means the
        mechanical system model is not used.

    """

    def __init__(
        self,
        par: InductionMachineInvGammaPars | InductionMachinePars,
        k_o1: Callable[[float], complex],
        k_o2: Callable[[float], complex],
        alpha_o: float,
        J: float | None = None,
    ) -> None:
        # Critically damped dynamics
        self.J = J
        if self.J is None:
            self.k_w = alpha_o
            self.k_tau = 0.0
        else:
            self.k_w = 2 * alpha_o
            self.k_tau = self.J * alpha_o**2

        super().__init__(par, k_o1, k_o2)

    def compute_output(
        self, u_s_ab: complex, i_s_ab: complex, w_M=None
    ) -> ObserverOutputs:
        """
        Compute feedback signals with speed estimation.

        Parameters
        ----------
        u_s_ab : complex
            Stator voltage (V) in stator coordinates.
        i_s_ab : complex
            Stator current (A) in stator coordinates.

        Returns
        -------
        out : ObserverOutputs
            Estimated feedback signals for the control system, including speed estimate.

        """
        w_M_est = self.state.w_m / self.par.n_p
        out = super().compute_output(u_s_ab, i_s_ab, w_M_est)

        if self.J is None:
            self._work.d_w_m = self.k_w * self._work.eps
            self._work.d_tau_L = 0.0
        else:
            self._work.d_w_m = (
                self.par.n_p * (out.tau_M - self.state.tau_L) / self.J
                + self.k_w * self._work.eps
            )
            self._work.d_tau_L = -self.k_tau * self._work.eps / self.par.n_p

        return out

    def update(self, T_s: float) -> None:
        """Extend the update method to include the speed estimate."""
        super().update(T_s)
        self.state.w_m += T_s * self._work.d_w_m
        self.state.tau_L += T_s * self._work.d_tau_L


# %%
def create_sensored_observer(
    par: InductionMachineInvGammaPars | InductionMachinePars,
    k_o: Callable[[float], complex] | None = None,
) -> FluxObserver:
    """
    Create a sensored observer.

    The observer gains are ``k_o1 = k_o`` and ``k_o2 = 0``.

    Parameters
    ----------
    par : InductionMachineInvGammaPars | InductionMachinePars
        Machine model parameters.
    k_o : Callable[[float], complex], optional
        Observer gain as a function of the electrical angular speed of the rotor,
        defaults to ``lambda w_m: 1 + 0.2*abs(w_m)/(R_R/L_M - 1j*w_m)``.

    """

    def default_k_o(w_m: float) -> complex:
        return 1.0 + 0.2 * abs(w_m) / (par.alpha - 1j * w_m)

    if k_o is None:
        k_o = default_k_o
    return FluxObserver(par, k_o, lambda w_m: 0)


def create_sensorless_observer(
    par: InductionMachineInvGammaPars | InductionMachinePars,
    alpha_o: float = 2 * pi * 40,
    k_o: Callable[[float], complex] | None = None,
    J: float | None = None,
) -> SpeedFluxObserver:
    """
    Create a sensorless observer with speed estimation.

    The observer gains are ``k_o1 = k_o`` and ``k_o2 = k_o``.

    Parameters
    ----------
    par : InductionMachineInvGammaPars | InductionMachinePars
        Machine model parameters.
    alpha_o : float
        Speed estimation pole (rad/s), defaults to 2*pi*40.
    k_o : Callable[[float], complex], optional
        Observer gain as a function of the electrical angular speed of the rotor,
        defaults to ``lambda w_m: (0.5*R_R/L_M + 0.2*abs(w_m))/(R_R/L_M - 1j*w_m)``.
    J : float, optional
        Inertia of the mechanical system (kgm²). Defaults to None, which means the
        mechanical system model is not used.

    """

    def default_k_o(w_m: float) -> complex:
        return (0.5 * par.alpha + 0.2 * abs(w_m)) / (par.alpha - 1j * w_m)

    if k_o is None:
        k_o = default_k_o

    return SpeedFluxObserver(par, k_o, k_o, alpha_o, J)


def create_vhz_observer(
    par: InductionMachineInvGammaPars | InductionMachinePars,
    k_o: Callable[[float], complex] | None = None,
) -> FluxObserver:
    """
    Create a sensorless flux observer without speed estimation.

    The observer gains are ``k_o1 = k_o`` and ``k_o2 = k_o``. However, if ``L_M = inf``,
    then ``k_o1 = 1`` and ``k_o2 = 0``, allowing to parametrize observer-based V/Hz
    control as pure open loop V/Hz control.

    Parameters
    ----------
    par : InductionMachineInvGammaPars | InductionMachinePars
        Machine model parameters.
    k_o : Callable[[float], complex], optional
        Observer gain as a function of the electrical angular speed of the rotor,
        defaults to ``lambda w_m: (0.5*R_R/L_M + 0.2*abs(w_m))/(R_R/L_M - 1j*w_m)``
        (except for the case ``L_M = inf``, where ``k_o1 = 1`` and ``k_o2 = 0``).

    """
    if par.L_M == inf:  # Pure open-loop V/Hz control
        return FluxObserver(par, lambda w_m: 1, lambda w_m: 0)

    def default_k_o(w_m: float) -> complex:
        return (0.5 * par.alpha + 0.2 * abs(w_m)) / (par.alpha - 1j * w_m)

    if k_o is None:
        k_o = default_k_o

    return FluxObserver(par, k_o, k_o)
