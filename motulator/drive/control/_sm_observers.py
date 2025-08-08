"""Observers for synchronous machine drives."""

from cmath import exp
from dataclasses import dataclass
from math import pi
from typing import Callable

from motulator.common.utils._utils import wrap
from motulator.drive.utils._parameters import (
    SaturatedSynchronousMachinePars,
    SynchronousMachinePars,
)


@dataclass
class ObserverStates:
    """State estimates."""

    theta_m: float = 0.0
    w_m: float = 0.0
    psi_s: complex = 0j
    tau_L: float = 0.0


@dataclass
class ObserverWorkspace:
    """Workspace variables."""

    d_psi_s: complex = 0j
    d_theta_m: float = 0.0
    d_psi_f: float = 0.0
    d_w_m: float = 0.0
    d_tau_L: float = 0.0
    eps_m: float = 0.0


@dataclass
class ObserverOutputs:
    """Feedback signals for the control system."""

    u_dc: float = 0.0
    i_s: complex = 0j
    u_s: complex = 0j
    psi_s: complex = 0j
    tau_M: float = 0.0  # Electromagnetic torque estimate
    tau_L: float = 0.0  # Load torque estimate
    w_c: float = 0.0  # Angular speed of the coordinate system
    theta_c: float = 0.0  # Coordinate system angle
    theta_m: float = 0.0  # Electrical rotor angle
    w_m: float = 0.0  # Electrical angular rotor speed
    w_M: float = 0.0  # Mechanical angular rotor speed
    psi_f: float = 0.0  # PM-flux linkage estimate


# %%
class FluxObserver:
    """
    Observer for synchronous machines in estimated rotor coordinates.

    This observer estimates the stator flux linkage, the rotor angle, and (optionally)
    the PM-flux linkage. The design is based on [#Hin2018]_ and [#Tuo2018]. The observer
    gain decouples the electrical and mechanical dynamics and allows placing the poles
    of the corresponding linearized estimation error dynamics. The PM-flux linkage can
    also be estimated [#Tuo2018]_. The observer can also be used in sensored mode, in
    which case the control system is fixed to the measured rotor angle. The magnetic
    saturation is taken into account.

    Parameters
    ----------
    par : SynchronousMachinePars | SaturatedSynchronousMachinePars
        Machine model parameters.
    k_theta : float
        Rotor angle estimation gain (rad/s).
    k_o : Callable[[float], float]
        Observer gain as a function of the rotor angular speed.
    k_f : Callable[[float], float], optional
        PM-flux estimation gain (V) as a function of the rotor angular speed.
    sensorless : bool
        If True, sensorless mode is used.

    References
    ----------
    .. [#Hin2018] Hinkkanen, Saarakkala, Awan, Mölsä, Tuovinen, "Observers for
       sensorless synchronous motor drives: Framework for design and analysis," IEEE
       Trans. Ind. Appl., 2018, https://doi.org/10.1109/TIA.2018.2858753

    .. [#Tuo2018] Tuovinen, Awan, Kukkola, Saarakkala, Hinkkanen, "Permanent-magnet flux
       adaptation for sensorless synchronous motor drives," Proc. IEEE SLED, 2018,
       https://doi.org/10.1109/SLED.2018.8485899

    """

    def __init__(
        self,
        par: SynchronousMachinePars | SaturatedSynchronousMachinePars,
        k_theta: float,
        k_o: Callable[[float], float],
        k_f: Callable[[float], float],
        sensorless: bool,
    ) -> None:
        self.par = par
        self.k_theta = k_theta
        self.k_o = k_o
        self.k_f = k_f
        self.sensorless = sensorless
        self.state = ObserverStates(theta_m=0.0, w_m=0.0, psi_s=complex(par.psi_f))
        self._work = ObserverWorkspace()

    def compute_output(
        self,
        u_s_ab: complex,
        i_s_ab: complex,
        w_M: float | None,
        theta_M_meas: float | None = None,
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
            Rotor speed (mechanical rad/s), either measured or estimated.
        theta_M_meas : float, optional
            Measured rotor angle (mechanical rad), used only in sensored mode.

        Returns
        -------
        out : ObserverOutputs
            Estimated feedback signals for the control system.

        """
        # Unpack and initialize the output signals
        par = self.par
        out = ObserverOutputs(psi_s=self.state.psi_s, psi_f=par.psi_f)

        # Get the rotor speed
        if w_M is None:
            raise ValueError
        w_m = par.n_p * w_M

        # Coordinate system angle equals the electrical rotor angle (or its estimate)
        if self.sensorless or theta_M_meas is None:
            out.theta_c = out.theta_m = self.state.theta_m
        else:
            out.theta_c = out.theta_m = par.n_p * theta_M_meas

        # Current and voltage vectors in (estimated) rotor coordinates
        out.i_s = exp(-1j * out.theta_m) * i_s_ab
        out.u_s = exp(-1j * out.theta_m) * u_s_ab

        # Error term
        e = complex(par.psi_s_dq(out.i_s)) - out.psi_s

        # Observer gains and error terms
        if self.sensorless:
            # Auxiliary flux
            psi_a = complex(par.aux_flux(out.i_s))

            # Observer gains
            k_o1 = self.k_o(w_m)
            k_o2 = k_o1 * psi_a / psi_a.conjugate() if psi_a != 0 else k_o1

            # Error term for the rotor angle estimation
            eps_m = -(e / psi_a).imag if psi_a != 0 else 0

            # Angular speed of the coordinate system
            out.w_c = w_m + self.k_theta * eps_m
            out.w_m = w_m

            # Error term for the PM-flux estimation
            eps_f = -(e / psi_a).real if psi_a != 0 else 0
        else:
            # Sensored mode assumes measured rotor coordinates
            k_o1, k_o2 = self.k_o(w_m), 0
            out.w_c = w_m
            out.w_m = w_m
            eps_m = 0
            eps_f = 0

        # Torque estimate
        out.tau_M = 1.5 * par.n_p * (out.i_s * out.psi_s.conjugate()).imag
        out.w_M = out.w_m / par.n_p

        # Compute and store the time derivatives for the update method
        v = out.u_s - par.R_s * out.i_s - 1j * out.w_c * out.psi_s
        self._work.d_psi_s = v + k_o1 * e + k_o2 * e.conjugate()
        self._work.d_psi_f = self.k_f(w_m) * eps_f
        self._work.d_theta_m = out.w_c
        self._work.eps_m = eps_m

        return out

    def update(self, T_s: float) -> None:
        """Update the state estimates."""
        # Update the state estimates
        self.state.psi_s += T_s * self._work.d_psi_s
        self.state.theta_m = wrap(self.state.theta_m + T_s * self._work.d_theta_m)
        # Update also the PM-fux parameter estimate
        self.par.psi_f += T_s * self._work.d_psi_f


# %%
class SpeedFluxObserver(FluxObserver):
    """
    Observer with load torque and speed estimation.

    This observer estimates the rotor speed and the rotor angle. The observer gain
    decouples the electrical and mechanical dynamics and allows placing the poles of the
    corresponding linearized estimation error dynamics. If the inertia of the
    mechanical system is provided, the observer also estimates the load torque, which
    avoids lag in the speed estimate during accelerations [#Lor1991]_.

    Parameters
    ----------
    par : SynchronousMachinePars | SaturatedSynchronousMachinePars
        Machine model parameters.
    alpha_o : float, optional
        Speed-estimation pole location (rad/s).
    k_o : Callable[[float], float], optional
        Observer gain as a function of the rotor angular speed.
    k_f : Callable[[float], float], optional
        PM-flux estimation gain (V) as a function of the rotor angular speed.
    J : float, optional
        Inertia of the mechanical system (kgm²). Defaults to None, which means the
        mechanical system model is not used.

    References
    ----------
    .. [#Lor1991] Lorenz, Van Patten, "High-resolution velocity estimation for
       all-digital, AC servo drives," IEEE Trans. Ind. Appl., 1991,
       https://doi.org/10.1109/28.85485

    """

    def __init__(
        self,
        par: SynchronousMachinePars | SaturatedSynchronousMachinePars,
        alpha_o: float,
        k_o: Callable[[float], float],
        k_f: Callable[[float], float],
        J: float | None = None,
    ) -> None:
        # Critically damped dynamics
        self.J = J
        if self.J is None:
            k_theta = 2 * alpha_o
            self.k_w = alpha_o**2
            self.k_tau = 0.0
        else:
            k_theta = 3 * alpha_o
            self.k_w = 3 * alpha_o**2
            self.k_tau = self.J * alpha_o**3

        super().__init__(par, k_theta, k_o, k_f, True)

    def compute_output(
        self,
        u_s_ab: complex,
        i_s_ab: complex,
        w_M: float | None = None,
        theta_M_meas: float | None = None,
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
            Estimated feedback signals for the control system.

        """
        w_M_est = self.state.w_m / self.par.n_p
        out = super().compute_output(u_s_ab, i_s_ab, w_M_est)
        out.tau_L = self.state.tau_L

        if self.J is None:
            self._work.d_w_m = self.k_w * self._work.eps_m
            self._work.d_tau_L = 0.0
        else:
            self._work.d_w_m = (
                self.par.n_p * (out.tau_M - out.tau_L) / self.J
                + self.k_w * self._work.eps_m
            )
            self._work.d_tau_L = -self.k_tau * self._work.eps_m / self.par.n_p

        return out

    def update(self, T_s: float) -> None:
        """Extend the update method to include the speed estimate."""
        super().update(T_s)
        self.state.w_m += T_s * self._work.d_w_m
        self.state.tau_L += T_s * self._work.d_tau_L


# %%
def create_sensored_observer(
    par: SynchronousMachinePars | SaturatedSynchronousMachinePars,
    k_theta: float = 2 * pi * 200,
    k_o: Callable[[float], float] | None = None,
    k_f: Callable[[float], float] | None = None,
) -> FluxObserver:
    """
    Create a flux observer for a drive equipped with a motion sensor.

    Parameters
    ----------
    par : SynchronousMachinePars | SaturatedSynchronousMachinePars
        Machine model parameters.
    k_theta : float, optional
        Rotor-angle estimation gain (rad/s), defaults to 2*pi*200.
    k_o : Callable[[float], float], optional
        Observer gain as a function of the rotor angular speed, defaults to ``lambda
        w_m: 0.25*(R_s*(L_d + L_q)/(L_d*L_q) + 0.2*abs(w_m))`` if `sensorless` else
        ``lambda w_m: 2*pi*15``.
    k_f : Callable[[float], float], optional
        PM-flux estimation gain (V) as a function of the rotor angular speed, defaults
        to zero, ``lambda w_m: 0``. A typical nonzero gain is of the form ``lambda w_m:
        max(k*(abs(w_m) - w_min), 0)``, i.e., zero below the speed `w_min` (rad/s) and
        linearly increasing above that with the slope `k` (Vs).

    Returns
    -------
    FluxObserver
        Flux observer for a sensored drive.

    """
    k_o = (lambda w_m: 2 * pi * 15) if k_o is None else k_o
    k_f = (lambda w_m: 0) if k_f is None else k_f

    return FluxObserver(par, k_theta, k_o, k_f, False)


def create_sensorless_observer(
    par: SynchronousMachinePars | SaturatedSynchronousMachinePars,
    alpha_o: float = 2 * pi * 100,
    k_o: Callable[[float], float] | None = None,
    k_f: Callable[[float], float] | None = None,
    J: float | None = None,
) -> SpeedFluxObserver:
    """
    Create a sensorless observer with speed estimation.

    Parameters
    ----------
    par : SynchronousMachinePars | SaturatedSynchronousMachinePars
        Machine model parameters.
    alpha_o : float, optional
        Speed estimation pole (rad/s), defaults to 2*pi*100.
    k_o : Callable[[float], float], optional
        Observer gain as a function of the rotor angular speed, defaults to ``lambda
        w_m: 0.25*(R_s*(L_d + L_q)/(L_d*L_q) + 0.2*abs(w_m))`` if `sensorless` else
        ``lambda w_m: 2*pi*15``.
    k_f : Callable[[float], float], optional
        PM-flux estimation gain (V) as a function of the rotor angular speed, defaults
        to zero, ``lambda w_m: 0``. A typical nonzero gain is of the form ``lambda w_m:
        max(k*(abs(w_m) - w_min), 0)``, i.e., zero below the speed `w_min` (rad/s) and
        linearly increasing above that with the slope `k` (Vs).
    J : float, optional
        Inertia of the mechanical system (kgm²). Defaults to None, which means the
        mechanical system model is not used.

    Returns
    -------
    SpeedFluxObserver
        Sensorless observer with speed estimation.

    """
    # Poles at zero speed are located s = 0 and s = -2*sigma0
    inv_L_s0 = par.inv_incr_ind_mat(par.psi_f)
    sigma0 = 0.25 * par.R_s * (inv_L_s0[0, 0] + inv_L_s0[1, 1])

    k_o = (lambda w_m: sigma0 + 0.2 * abs(w_m)) if k_o is None else k_o
    k_f = (lambda w_m: 0) if k_f is None else k_f

    return SpeedFluxObserver(par, alpha_o, k_o, k_f, J)


def create_vhz_observer(
    par: SynchronousMachinePars | SaturatedSynchronousMachinePars,
    k_theta: float = 2 * pi * 200,
    k_o: Callable[[float], float] | None = None,
) -> FluxObserver:
    """
    Create a sensorless flux observer without speed estimation.

    Parameters
    ----------
    par : SynchronousMachinePars | SaturatedSynchronousMachinePars
        Machine model parameters.
    k_theta : float, optional
        Angle estimation gain (rad/s), defaults to 2*pi*200.
    k_o : Callable[[float], float], optional
        Observer gain as a function of the rotor angular speed, defaults to ``lambda
        w_m: 0.25*(R_s*(L_d + L_q)/(L_d*L_q) + 0.2*abs(w_m))`` if `sensorless` else
        ``lambda w_m: 2*pi*15``.

    Returns
    -------
    FluxObserver
        Sensorless observer without speed estimation.

    """
    inv_L_s0 = par.inv_incr_ind_mat(par.psi_f)
    sigma0 = 0.25 * par.R_s * (inv_L_s0[0, 0] + inv_L_s0[1, 1])

    k_o = (lambda w_m: sigma0 + 0.2 * abs(w_m)) if k_o is None else k_o

    return FluxObserver(par, k_theta, k_o, lambda w_m: 0, True)
