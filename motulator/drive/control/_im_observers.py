"""Common control functions and classes for for induction machine drives."""

from cmath import exp
from dataclasses import dataclass
from math import inf, pi
from typing import Callable

from motulator.common.utils import wrap
from motulator.drive.control._base import Measurements
from motulator.drive.utils._parameters import InductionMachineInvGammaPars


# %%
@dataclass
class ObserverStates:
    """State estimates."""

    psi_R: float = 0.0
    theta_s: float = 0.0
    w_m: float = 0.0


@dataclass
class ObserverWorkspace:
    """Workspace variables."""

    d_psi_R: float = 0.0
    d_theta_s: float = 0.0
    old_i_s: complex = 0j
    w_r: float = 0.0
    T_s: float = 0.0


@dataclass
class ObserverOutputs:
    """Feedback signals for the control system."""

    u_dc: float = 0.0
    i_s: complex = 0j
    u_s: complex = 0j
    w_s: float = 0.0
    psi_s: complex = 0j
    psi_R: float = 0.0
    tau_M: float = 0.0
    theta_s: float = 0.0
    w_r: float = 0.0
    w_m: float = 0.0
    w_M: float = 0.0


class FluxObserver:
    """
    Reduced-order flux observer operating in estimated rotor flux coordinates.

    This class implements a reduced-order flux observer for induction machines. The
    observer structure is similar to [#Hin2010]_. The observer operates in estimated
    rotor flux coordinates.

    Parameters
    ----------
    par : InductionMachineInvGammaPars
        Machine model parameters.
    k_o1, k_o2 : Callable[[float], complex]
        Observer gains as functions of the rotor angular speed.

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
        par: InductionMachineInvGammaPars,
        k_o1: Callable[[float], complex],
        k_o2: Callable[[float], complex],
    ) -> None:
        self.par = par
        self.k_o1 = k_o1
        self.k_o2 = k_o2
        self.state = ObserverStates()
        self._work = ObserverWorkspace()

    def compute_output(
        self, meas: Measurements, u_s_ab: complex, w_M: float | None
    ) -> ObserverOutputs:
        """
        Compute the feedback signals for the control system.

        Parameters
        ----------
        meas : Measurements
            Measured signals.
        u_s_ab : complex
            Stator voltage (V) in stator coordinates.
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
        out = ObserverOutputs(psi_R=self.state.psi_R, theta_s=self.state.theta_s)

        # Current and voltage vectors in estimated rotor flux coordinates
        out.i_s = exp(-1j * out.theta_s) * meas.i_s_ab
        out.u_s = exp(-1j * out.theta_s) * u_s_ab

        # Stator flux estimate
        out.psi_s = par.L_sgm * out.i_s + out.psi_R

        # Derivative of the stator current
        T_s = self._work.T_s
        d_i_s = (out.i_s - self._work.old_i_s) / T_s if T_s > 0 else 0
        self._work.old_i_s = out.i_s  # Update the old value, safe to do here

        # Induced voltage from the stator quantities (without the w_s term that is taken
        # into account separately to avoid an algebraic loop)
        v_s = out.u_s - par.R_s * out.i_s - par.L_sgm * d_i_s

        # Induced voltage from the rotor quantities
        if w_M is None:
            raise ValueError
        w_m = par.n_p * w_M
        v_r = par.R_R * out.i_s - (par.R_R / par.L_M - 1j * w_m) * out.psi_R

        # Observer gains
        k_o1 = self.k_o1(w_m)
        k_o2 = self.k_o2(w_m)

        # Angular frequencies
        den = (
            out.psi_R
            + par.L_sgm * ((1 - k_o1) * out.i_s + k_o2 * out.i_s.conjugate()).real
        )
        num = (v_s + k_o1 * (v_r - v_s) + k_o2 * (v_r - v_s).conjugate()).imag
        out.w_s = num / den if den > 0 else w_m
        out.w_r = self.par.R_R * out.i_s.imag / out.psi_R if out.psi_R > 0 else 0
        out.w_m = w_m
        out.w_M = w_m / par.n_p

        # Torque estimate
        if par.L_sgm > 0:
            out.tau_M = 1.5 * par.n_p * (out.i_s * out.psi_s.conjugate()).imag
        else:  # Disable torque estimation in pure open-loop V/Hz control mode
            out.tau_M = 0

        # Compute and store the derivatives for the update method
        v = v_s - 1j * out.w_s * par.L_sgm * out.i_s
        self._work.d_psi_R = (v + k_o1 * (v_r - v) + k_o2 * (v_r - v).conjugate()).real
        self._work.d_theta_s = out.w_s

        return out

    def update(self, T_s: float) -> None:
        """Update the state estimates."""
        self.state.psi_R += T_s * self._work.d_psi_R
        self.state.theta_s = wrap(self.state.theta_s + T_s * self._work.d_theta_s)
        self._work.T_s = T_s


class SpeedFluxObserver(FluxObserver):
    """
    Observer with speed estimation.

    This class implements a reduced-order flux observer for induction machines with
    speed estimation. The observer structure is similar to [#Hin2010]_. The observer
    operates in estimated rotor flux coordinates.

    Parameters
    ----------
    par : InductionMachineInvGammaPars
        Machine model parameters.
    k_o1, k_o2 : Callable[[float], complex]
        Observer gains as functions of the rotor angular speed.
    alpha_o : float
        Speed estimation bandwidth (rad/s).

    """

    def __init__(
        self,
        par: InductionMachineInvGammaPars,
        k_o1: Callable[[float], complex],
        k_o2: Callable[[float], complex],
        alpha_o: float,
    ) -> None:
        super().__init__(par, k_o1, k_o2)
        self.alpha_o = alpha_o

    def compute_output(
        self, meas: Measurements, u_s_ab: complex, w_M=None
    ) -> ObserverOutputs:
        """Compute feedback signals with speed estimation."""
        w_M = self.state.w_m / self.par.n_p
        out = super().compute_output(meas, u_s_ab, w_M)
        self._work.w_r = out.w_r
        return out

    def update(self, T_s: float) -> None:
        """Extend the update method to include the speed estimate."""
        super().update(T_s)
        w_s = self._work.d_theta_s
        d_w_m = self.alpha_o * (w_s - self._work.w_r - self.state.w_m)
        self.state.w_m += T_s * d_w_m


# %%
def create_sensored_observer(
    par: InductionMachineInvGammaPars, k_o: Callable[[float], complex] | None = None
) -> FluxObserver:
    """
    Create a sensored observer.

    The observer gains are ``k_o1 = k_o`` and ``k_o2 = 0``.

    Parameters
    ----------
    par : InductionMachineInvGammaPars
        Machine model parameters.
    k_o : Callable[[float], complex], optional
        Observer gain as a function of the rotor angular speed, default to ``lambda w_m:
        1 + 0.2*abs(w_m)/(R_R/L_M - 1j*w_m)``.

    """
    alpha = par.R_R / par.L_M

    def default_k_o(w_m: float) -> complex:
        return 1 + 0.2 * abs(w_m) / (alpha - 1j * w_m)

    if k_o is None:
        k_o = default_k_o
    return FluxObserver(par, k_o, lambda w_m: 0)


def create_sensorless_observer(
    par: InductionMachineInvGammaPars,
    alpha_o: float = 2 * pi * 50,
    k_o: Callable[[float], complex] | None = None,
) -> SpeedFluxObserver:
    """
    Create a sensorless observer with speed estimation using

    The observer gains are ``k_o1 = k_o`` and ``k_o2 = k_o``.

    Parameters
    ----------
    par : InductionMachineInvGammaPars
        Machine model parameters.
    alpha_o : float
        Speed estimation bandwidth (rad/s), defaults to 2*pi*50.
    k_o : Callable[[float], complex], optional
        Observer gain as a function of the rotor angular speed, defaults to ``lambda
        w_m: (0.5*R_R/L_M + 0.2*abs(w_m))/(R_R/L_M - 1j*w_m)``.

    """
    alpha = par.R_R / par.L_M

    def default_k_o(w_m: float) -> complex:
        return (0.5 * alpha + 0.2 * abs(w_m)) / (alpha - 1j * w_m)

    if k_o is None:
        k_o = default_k_o

    return SpeedFluxObserver(par, k_o, k_o, alpha_o)


def create_vhz_observer(
    par: InductionMachineInvGammaPars, k_o: Callable[[float], complex] | None = None
) -> FluxObserver:
    """
    Create a sensorless flux observer without speed estimation.

    The observer gains are ``k_o1 = k_o`` and ``k_o2 = k_o``. However, if ``L_M = inf``,
    then ``k_o1 = 1`` and ``k_o2 = 0``, allowing to parametrize observer-based V/Hz
    control as pure open loop V/Hz control.

    Parameters
    ----------
    par : InductionMachineInvGammaPars
        Machine model parameters.
    k_o : Callable[[float], complex], optional
        Observer gain as a function of the rotor angular speed, defaults to ``lambda
        w_m: (0.5*R_R/L_M + 0.2*abs(w_m))/(R_R/L_M - 1j*w_m)`` (except for the case
        ``L_M = inf``, where ``k_o1 = 1`` and ``k_o2 = 0``).

    """
    if par.L_M == inf:  # Pure open-loop V/Hz control
        return FluxObserver(par, lambda w_m: 1, lambda w_m: 0)

    alpha = par.R_R / par.L_M

    def default_k_o(w_m: float) -> complex:
        return (0.5 * alpha + 0.2 * abs(w_m)) / (alpha - 1j * w_m)

    if k_o is None:
        k_o = default_k_o

    return FluxObserver(par, k_o, k_o)
