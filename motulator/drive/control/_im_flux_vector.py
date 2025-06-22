"""Flux-vector control of induction machine drives."""

from cmath import exp
from dataclasses import dataclass, field
from math import inf, pi, sqrt
from typing import Callable, Sequence

from motulator.common.control._pwm import PWM
from motulator.common.utils._utils import sign
from motulator.drive.control._base import Measurements
from motulator.drive.control._im_observers import (
    ObserverOutputs,
    create_sensored_observer,
    create_sensorless_observer,
    create_vhz_observer,
)
from motulator.drive.utils._parameters import InductionMachineInvGammaPars


# %%
@dataclass
class References:
    """Reference signals for flux-vector control."""

    T_s: float = 0.0
    d_abc: Sequence[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    tau_M: float = 0.0
    psi_s: float = 0.0
    u_s: complex = 0j
    i_s: complex = 0j


# %%
@dataclass
class Gains:
    """Control gains for flux and torque controller."""

    alpha_psi: float
    alpha_tau: float
    alpha_i: float
    alpha_c: float


class FluxTorqueController:
    """
    Flux and torque controller.

    Parameters
    ----------
    par : InductionMachineInvGammaPars
        Machine model parameters.
    alpha_psi : float
        Flux-control bandwidth (rad/s).
    alpha_tau : float
        Torque-control bandwidth (rad/s).
    alpha_i : float, optional
        Integral action bandwidth (rad/s), defaults to 0.
    alpha_c : float, optional
        Transparent current-control bandwidth (rad/s), defaults to `alpha_tau`.
    i_s_max : float, optional
        Stator current limit (A), defaults to `inf`.

    """

    def __init__(
        self,
        par: InductionMachineInvGammaPars,
        alpha_psi: float,
        alpha_tau: float,
        alpha_i: float = 0,
        alpha_c: float | None = None,
        i_s_max: float = inf,
    ) -> None:
        self.par = par
        alpha_c = alpha_tau if alpha_c is None else alpha_c
        self.i_s_max = i_s_max
        self.gain = Gains(alpha_psi, alpha_tau, alpha_i, alpha_c)
        # Integral states
        self.x_psi: complex = 0j
        self.x_tau: complex = 0j
        # Workspace variables
        self._i_a: complex = 0j
        self._v: complex = 0j

    def compute_output(
        self, psi_s_ref: float, tau_M_ref: float, fbk: ObserverOutputs
    ) -> tuple[complex, complex]:
        """
        Calculate the voltage reference, with transparent current limit.

        Parameters
        ----------
        psi_s_ref : float
            Stator flux reference (Vs).
        tau_M_ref : float
            Torque reference (Nm).
        fbk : ObserverOutputs
            Feedback signals.

        Returns
        -------
        tuple[complex, complex]
            Voltage reference (V) and stator current reference (A).

        """
        par = self.par
        gain = self.gain

        # Auxiliary current and torque-production factor
        if par.L_sgm > 0:  # Check to enable open-loop V/Hz control
            i_a = fbk.psi_R / par.L_sgm
            c_tau = 1.5 * par.n_p * (i_a * fbk.psi_s.conjugate()).real
            # Directions
            t_psi = 1.5 * par.n_p * abs(fbk.psi_s) * i_a / c_tau if c_tau > 0 else 1
            t_tau = 1j * fbk.psi_s / c_tau if c_tau > 0 else 0
        else:
            i_a = 0j
            t_psi = 1
            t_tau = 0

        # Error signals
        e_psi = psi_s_ref - abs(fbk.psi_s)
        e_tau = tau_M_ref - fbk.tau_M
        e_u = gain.alpha_psi * e_psi * t_psi + gain.alpha_tau * e_tau * t_tau
        u_i = self.x_psi * t_psi + self.x_tau * t_tau
        e_v = u_i - gain.alpha_i * (abs(fbk.psi_s) * t_psi + fbk.tau_M * t_tau)

        # Internal current reference for state feedback
        if gain.alpha_c * par.L_sgm > 0:
            i_s_ref = fbk.i_s + e_u / (gain.alpha_c * par.L_sgm)
            if abs(i_s_ref) > self.i_s_max:
                i_s_ref = self.i_s_max * i_s_ref / abs(i_s_ref)
            e_up = gain.alpha_c * par.L_sgm * (i_s_ref - fbk.i_s)
        else:
            i_s_ref = fbk.i_s
            e_up = e_u

        # Voltage reference
        v = par.R_s * fbk.i_s + 1j * (fbk.w_m + fbk.w_r) * fbk.psi_s + e_v
        u_s_ref = v + e_up

        # Workspace variables for the update method
        self._i_a = i_a
        self._v = v

        return u_s_ref, i_s_ref

    def update(self, T_s: float, fbk: ObserverOutputs) -> None:
        """Update the integral states."""
        par, gain = self.par, self.gain
        # Error signal and gains
        e = fbk.u_s - self._v
        k_psi = gain.alpha_i / abs(fbk.psi_s) if abs(fbk.psi_s) > 0 else 0
        k_tau = 1.5 * par.n_p * gain.alpha_i
        # Update the integral states
        self.x_psi += T_s * k_psi * (fbk.psi_s * e.conjugate()).real
        self.x_tau += T_s * k_tau * (1j * self._i_a * e.conjugate()).real


# %%
class ReferenceGenerator:
    """
    Reference generator for flux-vector control.

    Parameters
    ----------
    par : InductionMachineInvGammaPars
        Machine model parameters.
    psi_s_nom : float
        Nominal stator flux linkage (Vs).
    i_s_max : float
        Maximum stator current (A).
    tau_M_max : float, optional
        Maximum torque reference (Nm), defaults to `inf`.
    k_u : float, optional
        Voltage utilization factor, defaults to 1.
    k_b : float, optional
        Breakdown torque margin, defaults to 1.

    """

    def __init__(
        self,
        par: InductionMachineInvGammaPars,
        psi_s_nom: float,
        i_s_max: float,
        tau_M_max: float = inf,
        k_u: float = 1.0,
        k_b: float = 1.0,
    ) -> None:
        self.par = par
        self.psi_s_nom = psi_s_nom
        self.i_s_max = i_s_max
        self.tau_M_max = tau_M_max
        self.k_u = k_u
        self.k_b = k_b

    def compute_output(
        self, tau_M_ref: float, w_s: float, u_dc: float
    ) -> tuple[float, float]:
        """Simple field-weakening strategy."""
        par = self.par
        # Field-weakening
        u_s_max = self.k_u * u_dc / sqrt(3)
        psi_s_max = u_s_max / abs(w_s) if w_s != 0 else inf
        psi_s_ref = min(psi_s_max, self.psi_s_nom)
        # Torque limit
        if par.L_sgm > 0:  # Check to enable open-loop V/Hz control
            k = self.k_b * par.L_M / (par.L_M + par.L_sgm)
            tau_b = 0.75 * par.n_p * k * psi_s_ref**2 / par.L_sgm
            tau_M_ref = min(abs(tau_M_ref), self.tau_M_max, tau_b) * sign(tau_M_ref)
        return psi_s_ref, tau_M_ref


# %%
@dataclass
class FluxVectorControllerCfg:
    """
    Flux-vector controller configuration.

    Parameters
    ----------
    psi_s_nom : float
        Nominal stator flux linkage (Vs).
    i_s_max : float
        Maximum stator current (A).
    alpha_tau : float, optional
        Torque-control bandwidth (rad/s), defaults to 2*pi*100.
    alpha_psi : float, optional
        Flux-control bandwidth (rad/s), defaults to `alpha_tau`.
    alpha_i : float, optional
        Integral action bandwidth (rad/s), defaults to `alpha_tau`.
    alpha_o : float, optional
        Speed estimation bandwidth (rad/s), defaults to 2*pi*50.
    k_o : Callable[[float], complex], optional
        Observer gain as a function of the rotor angular speed.
    alpha_c : float, optional
        Transparent current-control bandwidth (rad/s), defaults to `alpha_tau`.
    tau_M_max : float
        Maximum torque reference (Nm).
    k_u : float, optional
        Voltage utilization factor, defaults to 0.9.
    k_b : float, optional
        Breakdown torque margin, defaults to 0.9.

    """

    psi_s_nom: float
    i_s_max: float
    alpha_tau: float = 2 * pi * 100
    alpha_psi: float | None = None
    alpha_i: float | None = None
    alpha_o: float = 2 * pi * 50
    k_o: Callable[[float], complex] | None = None
    alpha_c: float | None = None
    tau_M_max: float = inf
    k_u: float = 0.9
    k_b: float = 0.9


# %%
class FluxVectorController:
    """
    Flux-vector controller for induction machine drives.

    This class implements a variant of flux-vector control. Rotor coordinates and
    decoupling between the stator flux and torque channels are used [#Tii2025b]_.


    Parameters
    ----------
    par : InductionMachineInvGammaPars
        Machine model parameters.
    cfg : FluxVectorControllerCfg
        Flux-vector control configuration.
    sensorless : bool, optional
        If True, sensorless control is used, defaults to True.
    T_s : float, optional
        Sampling period (s), defaults to 125e-6.

    References
    ----------
    .. [#Tii2025b] Tiitinen, Hinkkanen, Harnefors, "Sensorless flux-vector control
       framework: An extension for induction machines," IEEE Trans. Ind. Electron.,
       2025, https://doi.org/10.1109/TIE.2025.3559958

    """

    def __init__(
        self,
        par: InductionMachineInvGammaPars,
        cfg: FluxVectorControllerCfg,
        sensorless: bool = True,
        T_s: float = 125e-6,
    ) -> None:
        self.pwm = PWM()
        self.reference_gen = ReferenceGenerator(
            par, cfg.psi_s_nom, cfg.i_s_max, cfg.tau_M_max, cfg.k_u, cfg.k_b
        )
        alpha_psi = cfg.alpha_tau if cfg.alpha_psi is None else cfg.alpha_psi
        alpha_i = cfg.alpha_tau if cfg.alpha_i is None else cfg.alpha_i
        self.flux_torque_ctrl = FluxTorqueController(
            par, alpha_psi, cfg.alpha_tau, alpha_i, cfg.alpha_c, cfg.i_s_max
        )
        if sensorless:
            self.observer = create_sensorless_observer(par, cfg.alpha_o, cfg.k_o)
        else:
            self.observer = create_sensored_observer(par, cfg.k_o)
        self.sensorless = sensorless
        self.T_s = T_s

    def get_feedback(self, meas: Measurements) -> ObserverOutputs:
        """Get feedback signals."""
        u_c_ab = self.pwm.get_realized_voltage()
        if self.sensorless:
            fbk = self.observer.compute_output(u_c_ab, meas.i_c_ab)
        else:
            fbk = self.observer.compute_output(u_c_ab, meas.i_c_ab, meas.w_M)
        fbk.u_dc = meas.u_dc
        return fbk

    def compute_output(self, tau_M_ref: float, fbk: ObserverOutputs) -> References:
        """Compute references."""
        ref = References(T_s=self.T_s, tau_M=tau_M_ref)
        ref.psi_s, ref.tau_M = self.reference_gen.compute_output(
            ref.tau_M, fbk.w_s, fbk.u_dc
        )
        ref.u_s, ref.i_s = self.flux_torque_ctrl.compute_output(
            ref.psi_s, ref.tau_M, fbk
        )
        u_s_ref_ab = exp(1j * fbk.theta_s) * ref.u_s
        ref.d_abc = self.pwm(ref.T_s, u_s_ref_ab, fbk.u_dc, fbk.w_s)
        return ref

    def update(self, ref: References, fbk: ObserverOutputs) -> None:
        """Update states."""
        self.observer.update(ref.T_s)
        self.flux_torque_ctrl.update(ref.T_s, fbk)


# %%
@dataclass
class ObserverBasedVHzControllerCfg:
    """
    Observer-based V/Hz controller configuration.

    Parameters
    ----------
    psi_s_nom : float
        Nominal stator flux linkage (Vs).
    i_s_max : float
        Maximum stator current (A).
    alpha_psi : float, optional
        Flux-control bandwidth (rad/s), defaults to 2*pi*100.
    alpha_tau : float, optional
        Torque-control bandwidth (rad/s), defaults to 2*pi*20.
    alpha_f : float, optional
        Low-pass-filter bandwidth (rad/s), defaults to 2*pi*1.
    k_o : Callable[[float], complex], optional
        Observer gain as a function of the rotor angular speed.
    k_u : float, optional
        Voltage utilization factor, defaults to 0.9.
    k_b : float, optional
        Breakdown torque margin, defaults to 0.9.

    """

    psi_s_nom: float
    i_s_max: float
    alpha_psi: float = 2 * pi * 100
    alpha_tau: float = 2 * pi * 20
    alpha_f: float = 2 * pi * 1
    k_o: Callable[[float], complex] | None = None
    k_u: float = 0.9
    k_b: float = 0.9


class ObserverBasedVHzController:
    """
    Observer-based V/Hz controller for induction machine drives.

    This class implements sensorless observer-based V/Hz control. Rotor coordinates and
    decoupling between the stator flux and torque channels are used [#Tii2025b]_.

    Parameters
    ----------
    par : InductionMachineInvGammaPars
        Machine model parameters.
    cfg : ObserverBasedVHzControllerCfg
        Observer-based V/Hz controller configuration.
    T_s : float, optional
        Sampling period (s), defaults to 250e-6.

    """

    def __init__(
        self,
        par: InductionMachineInvGammaPars,
        cfg: ObserverBasedVHzControllerCfg,
        T_s: float = 250e-6,
    ) -> None:
        self.pwm = PWM()
        self.reference_gen = ReferenceGenerator(
            par, cfg.psi_s_nom, cfg.i_s_max, inf, cfg.k_u, cfg.k_b
        )
        self.flux_torque_ctrl = FluxTorqueController(
            par, cfg.alpha_psi, cfg.alpha_tau, 0, cfg.alpha_tau, cfg.i_s_max
        )
        self.observer = create_vhz_observer(par, cfg.k_o)
        self.alpha_f: float = cfg.alpha_f
        self.tau_M_lpf: float = 0.0  # Low-pass-filtered torque estimate
        self.T_s = T_s
        # Configurations for pure open-loop V/Hz control
        if par.L_M == inf:
            self.observer.state.psi_R = cfg.psi_s_nom
            self.pwm = PWM(overmodulation="MME")
            self.reference_gen.k_u = inf

    def get_feedback(self, w_M_ref: float, meas: Measurements) -> ObserverOutputs:
        """Get feedback signals."""
        u_c_ab = self.pwm.get_realized_voltage()
        fbk = self.observer.compute_output(u_c_ab, meas.i_c_ab, w_M_ref)
        fbk.u_dc = meas.u_dc
        return fbk

    def compute_output(self, fbk: ObserverOutputs) -> References:
        """Compute references."""
        ref = References(T_s=self.T_s)
        w_s = fbk.w_m + fbk.w_r
        ref.psi_s, ref.tau_M = self.reference_gen.compute_output(
            self.tau_M_lpf, w_s, fbk.u_dc
        )
        ref.u_s, ref.i_s = self.flux_torque_ctrl.compute_output(
            ref.psi_s, ref.tau_M, fbk
        )
        u_s_ref_ab = exp(1j * fbk.theta_s) * ref.u_s
        ref.d_abc = self.pwm(ref.T_s, u_s_ref_ab, fbk.u_dc, w_s)
        return ref

    def update(self, ref: References, fbk: ObserverOutputs) -> None:
        """Update states."""
        self.tau_M_lpf += ref.T_s * self.alpha_f * (fbk.tau_M - self.tau_M_lpf)
        self.observer.update(ref.T_s)
