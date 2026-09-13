"""Common control functions and classes for for induction machine drives."""

from cmath import exp
from dataclasses import dataclass
from math import inf, pi
from typing import Callable

from motulator.common.utils._utils import wrap
from motulator.drive.control._common import SpeedObserver
from motulator.drive.utils._parameters import (
    InductionMachineInvGammaPars,
    InductionMachinePars,
)


# %%
@dataclass
class ObserverOutputs:
    """Feedback signals for the control system."""

    u_dc: float = 0.0  # DC-bus voltage
    i_s: complex = 0j  # Stator current
    u_s: complex = 0j  # Stator voltage
    psi_s: complex = 0j  # Stator flux estimate
    psi_R: complex = 0j  # Rotor flux estimate
    tau_M: float = 0.0  # Electromagnetic torque estimate
    tau_L: float = 0.0  # Load torque estimate
    w_c: float = 0.0  # Angular speed of the coordinate system
    w_s: float = 0.0  # Synchronous angular frequency, w_s = w_m + w_r
    w_r: float = 0.0  # Slip angular frequency estimate
    w_m: float = 0.0  # Electrical angular rotor speed estimate
    w_M: float = 0.0  # Mechanical angular rotor speed estimate
    theta_c: float = 0.0  # Coordinate system angle
    e_o: complex = 0j  # Estimation error
    eps: float = 0.0  # Mechanical rotor speed estimation error signal
    h: float = 0.0  # Weight of the external speed error signal


# %%
class FluxObserver:
    """
    Reduced-order flux observer.

    This class implements a reduced-order flux observer for induction machines. The
    observer structure is similar to [#Hin2010]_. The observer operates in synchronous
    coordinates rotating at `w_c` (but not locked to any particular vector). The rotor
    speed error signal is either computed internally from the flux estimation error or
    supplied externally, e.g., from the speed measurement. The weight `h` of the
    external signal, given to `compute_output`, selects between these: `h = 0` gives
    purely model-based (sensorless) operation, `h = 1` relies on the external signal
    alone, and intermediate values blend the two. The conjugate-error observer gain
    follows the same weight. The main-flux saturation can be taken into account by
    providing the saturation model via `InductionMachinePars`.

    Parameters
    ----------
    par : InductionMachineInvGammaPars | InductionMachinePars
        Machine model parameters.
    k_o : Callable[[float], complex]
        Observer gain as a function of the electrical angular speed of the rotor.

    Notes
    -----
    The pure voltage model corresponds to ``k_o = lambda w_m: 0``, resulting in the
    marginally stable estimation-error dynamics. The current model is obtained by
    setting ``k_o = lambda w_m: 1`` together with `h = 1`.

    References
    ----------
    .. [#Hin2010] Hinkkanen, Harnefors, Luomi, "Reduced-order flux observers with
       stator-resistance adaptation for speed-sensorless induction motor drives," IEEE
       Trans. Power Electron., 2010, https://doi.org/10.1109/TPEL.2009.2039650

    """

    def __init__(
        self,
        par: InductionMachinePars | InductionMachineInvGammaPars,
        k_o: Callable[[float], complex],
    ) -> None:
        self.par = par
        self.k_o = k_o
        # States
        self.psi_s: complex = 0j
        self.theta_c: float = 0.0
        # Other memory variables
        self._T_s: float = 0.0
        self._old_i_s: complex = 0j

    def compute_output(
        self,
        u_s_ab: complex,
        i_s_ab: complex,
        w_M: float,
        eps_ext: float = 0.0,
        h: float = 0.0,
    ) -> ObserverOutputs:
        """
        Compute the feedback signals for the control system.

        Parameters
        ----------
        u_s_ab : complex
            Stator voltage (V) in stator coordinates.
        i_s_ab : complex
            Stator current (A) in stator coordinates.
        w_M : float
            Rotor speed (mechanical rad/s), typically from the speed observer.
        eps_ext : float, optional
            External rotor speed error signal (mechanical rad/s), defaults to 0.
        h : float, optional
            Weight of `eps_ext` in the range [0, 1], defaults to 0, i.e., the model-
            based error signal is used exclusively.

        Returns
        -------
        out : ObserverOutputs
            Estimated feedback signals for the control system.

        """
        # Unpack
        par = self.par

        # Initialize the output signals
        out = ObserverOutputs(psi_s=self.psi_s, theta_c=self.theta_c, h=h)

        # Current and voltage vectors in estimated rotor coordinates
        out.i_s = exp(-1j * out.theta_c) * i_s_ab
        out.u_s = exp(-1j * out.theta_c) * u_s_ab

        # Mechanical and electrical angular speeds of the rotor
        out.w_M = w_M
        out.w_m = par.n_p * w_M

        # Rotor flux estimate
        out.psi_R = out.psi_s - par.L_sgm * self._old_i_s

        # Slip angular frequency
        prod = out.psi_s * out.psi_R.conjugate()
        out.w_r = par.w_rb * prod.imag / prod.real if prod.real > 0 else 0.0

        # Angular speed of the coordinate system equals synchronous angular frequency
        out.w_c = out.w_s = out.w_m + out.w_r
        # out.w_c = out.w_m  # Use rotor coordinates instead of synchronous coordinates

        # Estimation error
        d_i_s = (out.i_s - self._old_i_s) / self._T_s if self._T_s > 0 else 0.0
        out.e_o = (
            par.L_sgm * d_i_s
            - out.u_s
            + (par.R_sgm + 1j * out.w_c * par.L_sgm) * out.i_s
            - (par.alpha - 1j * out.w_m) * out.psi_R
        )

        # Torque estimate
        if par.L_sgm > 0:
            out.tau_M = 1.5 * par.n_p * (out.i_s * out.psi_s.conjugate()).imag
        else:  # Disable torque estimation in pure open-loop V/Hz control mode
            out.tau_M = 0

        # Rotor speed error signal for the speed observer. The model-based part is faded
        # out as the external error signal takes over.
        eps_o = -(out.e_o / out.psi_R).imag / par.n_p if abs(out.psi_R) > 0 else 0.0
        out.eps = h * eps_ext + (1 - h) * eps_o

        return out

    def update(self, T_s: float, out: ObserverOutputs) -> None:
        """Update the state estimates."""
        par = self.par

        # Observer gains. The conjugate-error gain decouples the speed estimation error,
        # so it is faded out together with the model-based error signal.
        k_o1 = self.k_o(out.w_m)
        proj = out.psi_R / out.psi_R.conjugate() if abs(out.psi_R) > 0.0 else 0.0
        k_o2 = (1 - out.h) * self.k_o(out.w_m) * proj

        # Update the states
        v_err = k_o1 * out.e_o + k_o2 * out.e_o.conjugate()
        d_psi_s = out.u_s - par.R_s * out.i_s - 1j * out.w_c * out.psi_s + v_err
        self.psi_s += T_s * d_psi_s
        self.theta_c = wrap(self.theta_c + T_s * out.w_c)

        # Update the saturation model for the next sampling period
        self.par.update_psi_s(abs(self.psi_s))

        # Update the sampling period and the old current value
        self._T_s = T_s
        self._old_i_s = out.i_s


# %%
class SpeedFluxObserver:
    """
    Flux observer with speed estimation.

    This class implements a reduced-order flux observer for induction machines with
    speed estimation. If the inertia of the mechanical system is provided, the observer
    also estimates the load torque, to avoid the lag in the speed estimate. The rotor
    speed error signal is defined in {class}`FluxObserver`.

    Parameters
    ----------
    par : InductionMachineInvGammaPars | InductionMachinePars
        Machine model parameters.
    alpha_o : float
        Speed estimation pole (rad/s).
    k_o : Callable[[float], complex]
        Observer gain as a function of the electrical angular speed of the rotor.
    J : float, optional
        Inertia of the mechanical system (kgm²). Defaults to None, which means the
        mechanical system model is not used.

    """

    def __init__(
        self,
        par: InductionMachineInvGammaPars | InductionMachinePars,
        alpha_o: float,
        k_o: Callable[[float], complex],
        J: float | None = None,
    ) -> None:
        # Configure observer gains for critically damped dynamics
        if J is None:
            k_w = alpha_o
            k_tau = 0.0
        else:
            k_w = 2 * alpha_o
            k_tau = J * alpha_o**2
        # Create component observers
        self.speed_observer = SpeedObserver(k_w, k_tau, J)
        self.flux_observer = FluxObserver(par, k_o)

    def compute_output(
        self, u_s_ab: complex, i_s_ab: complex, eps_ext: float = 0.0, h: float = 0.0
    ) -> ObserverOutputs:
        """
        Compute feedback signals with speed estimation.

        Parameters
        ----------
        u_s_ab : complex
            Stator voltage (V) in stator coordinates.
        i_s_ab : complex
            Stator current (A) in stator coordinates.
        eps_ext : float, optional
            External rotor speed error signal (mechanical rad/s), defaults to 0.
        h : float, optional
            Weight of `eps_ext` in the range [0, 1], defaults to 0, i.e., the model-
            based error signal is used exclusively.

        Returns
        -------
        out : ObserverOutputs
            Estimated feedback signals for the control system.

        """
        w_M, tau_L = self.speed_observer.compute_output()
        out = self.flux_observer.compute_output(u_s_ab, i_s_ab, w_M, eps_ext, h)
        out.tau_L = tau_L
        return out

    def update(self, T_s: float, out: ObserverOutputs) -> None:
        """Update state observers."""
        self.speed_observer.update(T_s, out.eps, out.tau_M)
        self.flux_observer.update(T_s, out)


# %%
def create_speed_flux_observer(
    par: InductionMachineInvGammaPars | InductionMachinePars,
    alpha_o: float | None = None,
    k_o: Callable[[float], complex] | None = None,
    sensorless: bool = True,
    J: float | None = None,
) -> SpeedFluxObserver:
    """
    Create a flux observer with speed estimation.

    In sensored mode, the measured rotor speed is filtered. In sensorless mode, the
    rotor speed is estimated based on the stator voltage and current. The sensored mode
    corresponds to the weight `h = 1` and the sensorless mode to `h = 0`, see
    {class}`FluxObserver`.

    Parameters
    ----------
    par : InductionMachineInvGammaPars | InductionMachinePars
        Machine model parameters.
    alpha_o : float, optional
        Speed estimation pole (rad/s). If `None`, it defaults to 2*pi*40 in sensorless
        mode and 2*pi*400 in sensored mode.
    k_o : Callable[[float], complex], optional
        Observer gain as a function of the electrical angular rotor speed. If `None`, it
        defaults to ``lambda w_m: (0.5*par.alpha + 0.2*abs(w_m))/(par.alpha - 1j*w_m)``
        in sensorless mode, and to ``lambda w_m: 1.0 + 0.2*abs(w_m)/(par.alpha
        - 1j*w_m)`` in sensored mode.
    sensorless : bool, optional
        If True, sensorless control is used, defaults to True.
    J : float, optional
        Inertia of the mechanical system (kgm²). Defaults to None, which means the
        mechanical system model is not used.

    Returns
    -------
    SpeedFluxObserver
        Flux observer with speed estimation.

    """
    if alpha_o is None:
        alpha_o = 2 * pi * 40 if sensorless else 2 * pi * 400

    def default_k_o_sensorless(w_m: float) -> complex:
        return (0.5 * par.alpha + 0.2 * abs(w_m)) / (par.alpha - 1j * w_m)

    def default_k_o_sensored(w_m: float) -> complex:
        return 1.0 + 0.2 * abs(w_m) / (par.alpha - 1j * w_m)

    if sensorless:
        k_o = default_k_o_sensorless if k_o is None else k_o
    else:
        k_o = default_k_o_sensored if k_o is None else k_o

    return SpeedFluxObserver(par, alpha_o, k_o, J)


def create_vhz_observer(
    par: InductionMachineInvGammaPars | InductionMachinePars,
    k_o: Callable[[float], complex] | None = None,
) -> FluxObserver:
    """
    Create a sensorless flux observer without speed estimation.

    The observer gain is `k_o`. However, if ``L_M = inf``, then ``k_o = 1`` is used
    together with the weight `h = 1`, which disables the model-based correction and
    allows to parametrize observer-based V/Hz control as pure open loop V/Hz control.

    Parameters
    ----------
    par : InductionMachineInvGammaPars | InductionMachinePars
        Machine model parameters.
    k_o : Callable[[float], complex], optional
        Observer gain as a function of the electrical angular speed of the rotor,
        defaults to ``lambda w_m: (0.5*R_R/L_M + 0.2*abs(w_m))/(R_R/L_M - 1j*w_m)``
        (except for the case ``L_M = inf``, where ``k_o1 = 1`` and ``k_o2 = 0``).

    Returns
    -------
    FluxObserver
        Sensorless flux observer without speed estimation.

    """
    if par.L_M == inf:  # Pure open-loop V/Hz control, used with h = 1
        return FluxObserver(par, lambda w_m: 1)

    def default_k_o(w_m: float) -> complex:
        return (0.5 * par.alpha + 0.2 * abs(w_m)) / (par.alpha - 1j * w_m)

    if k_o is None:
        k_o = default_k_o

    return FluxObserver(par, k_o)
