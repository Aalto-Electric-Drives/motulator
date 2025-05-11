"""Flux-vector control of synchronous machine drives."""

from cmath import exp
from dataclasses import dataclass, field
from math import inf, pi
from typing import Callable, Sequence

from motulator.common.control._pwm import PWM
from motulator.drive.control._base import Measurements
from motulator.drive.control._sm_observers import (
    ObserverOutputs,
    create_sensored_observer,
    create_sensorless_observer,
    create_vhz_observer,
)
from motulator.drive.control._sm_reference_gen import ReferenceGenerator
from motulator.drive.utils._parameters import (
    SaturatedSynchronousMachinePars,
    SynchronousMachinePars,
)


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
    w_M: float | None = None


# %%
@dataclass
class Gains:
    """Control gains for flux and torque controller."""

    alpha_psi: float
    alpha_tau: float
    alpha_i: float


class FluxTorqueController:
    """
    Flux and torque controller.

    Parameters
    ----------
    par : SynchronousMachinePars | SaturatedSynchronousMachinePars
        Machine model parameters.
    alpha_psi : float
        Flux-control bandwidth (rad/s).
    alpha_tau : float
        Torque-control bandwidth (rad/s).
    alpha_i : float, optional
        Integral action bandwidth (rad/s), defaults to 0.

    """

    def __init__(
        self,
        par: SynchronousMachinePars | SaturatedSynchronousMachinePars,
        alpha_psi: float,
        alpha_tau: float,
        alpha_i: float = 0.0,
    ) -> None:
        self.par = par
        # Integral states
        self.x_psi: complex = alpha_i * par.psi_f
        self.x_tau: complex = 0j
        # Workspace variables
        self._i_a: complex = 0j
        self._v: complex = 0j
        # Gains
        self.gain = Gains(alpha_psi, alpha_tau, alpha_i)

    def compute_output(
        self, psi_s_ref: float, tau_M_ref: float, fbk: ObserverOutputs
    ) -> complex:
        """Calculate the voltage reference."""
        par = self.par
        gain = self.gain

        # Auxiliary current and torque-production factor
        i_a = complex(par.aux_current(fbk.psi_s))
        c_tau = 1.5 * par.n_p * (i_a * fbk.psi_s.conjugate()).real

        # Directions
        t_psi = 1.5 * par.n_p * abs(fbk.psi_s) * i_a / c_tau if c_tau > 0 else 1
        t_tau = 1j * fbk.psi_s / c_tau if c_tau > 0 else 0

        # Error signals
        e_psi = psi_s_ref - abs(fbk.psi_s)
        e_tau = tau_M_ref - fbk.tau_M
        e_u = gain.alpha_psi * e_psi * t_psi + gain.alpha_tau * e_tau * t_tau
        u_i = self.x_psi * t_psi + self.x_tau * t_tau
        e_v = u_i - gain.alpha_i * (abs(fbk.psi_s) * t_psi + fbk.tau_M * t_tau)

        # Voltage reference
        v = par.R_s * fbk.i_s + 1j * fbk.w_m * fbk.psi_s + e_v
        u_s_ref = v + e_u

        # Workspace variables for the update method
        self._i_a = i_a
        self._v = v

        return u_s_ref

    def update(self, T_s: float, fbk: ObserverOutputs) -> None:
        """Update the integral state."""
        par = self.par
        gain = self.gain
        # Error signal and gains
        e = fbk.u_s - self._v
        k_psi = gain.alpha_i / abs(fbk.psi_s) if abs(fbk.psi_s) > 0 else 0
        k_tau = 1.5 * par.n_p * gain.alpha_i
        # Update the integral states
        self.x_psi += T_s * k_psi * (fbk.psi_s * e.conjugate()).real
        self.x_tau += T_s * k_tau * (1j * self._i_a * e.conjugate()).real


# %%
@dataclass
class FluxVectorControllerCfg:
    """
    Flux-vector controller configuration.

    Parameters
    ----------
    i_s_max : float
        Maximum stator current (A).
    alpha_tau : float, optional
        Torque-control bandwidth (rad/s), defaults to 2*pi*100.
    alpha_psi : float, optional
        Flux-control bandwidth (rad/s), defaults to `alpha_tau`.
    alpha_i : float, optional
        Integral-action bandwidth (rad/s), defaults to `alpha_tau`.
    alpha_o : float, optional
        Speed estimation bandwidth (rad/s), defaults to 2*pi*50.
    k_o : Callable[[float], float], optional
        Observer gain as a function of the rotor angular speed.
    k_f : Callable[[float], float], optional
        PM-flux estimation gain as a function of the rotor angular speed.
    psi_s_min : float, optional
        Minimum stator flux (Vs), defaults to `par.psi_f`.
    psi_s_max : float, optional
        Maximum stator flux (Vs), defaults to `inf`.
    k_u : float, optional
        Voltage utilization factor, defaults to 0.9.
    k_mtpv : float, optional
        MTPV margin, defaults to 0.9.

    """

    i_s_max: float
    alpha_tau: float = 2 * pi * 100
    alpha_psi: float | None = None
    alpha_i: float | None = None
    alpha_o: float = 2 * pi * 50
    k_o: Callable[[float], float] | None = None
    k_f: Callable[[float], float] | None = None
    psi_s_min: float | None = None
    psi_s_max: float = inf
    k_u: float = 0.9
    k_mtpv: float = 0.9


# %%
class FluxVectorController:
    """
    Flux-vector controller of synchronous machine drives.

    This class implements a variant of flux-vector control. Rotor coordinates and
    decoupling between the stator flux and torque channels are used according to
    [#Awa2019b]_. Here, the stator flux magnitude and the electromagnetic torque are
    selected as controllable variables [#Tii2025a]_. The magnetic saturation is taken
    into account [#Var2022]_.

    Parameters
    ----------
    par : SynchronousMachinePars | SaturatedSynchronousMachinePars
        Machine model parameters.
    cfg : FluxVectorControllerCfg
        Flux-vector control configuration.
    sensorless : bool, optional
        If True, sensorless control is used, defaults to True.
    T_s : float, optional
        Sampling period (s), defaults to 125e-6.

    References
    ----------
    .. [#Awa2019b] Awan, Hinkkanen, Bojoi, Pellegrino, "Stator-flux-oriented control of
       synchronous motors: A systematic design procedure," IEEE Trans. Ind. Appl., 2019,
       https://doi.org/10.1109/TIA.2019.2927316

    .. [#Tii2025a] Tiitinen, Hinkkanen, Harnefors, "Design framework for sensorless
       control of synchronous machine drives," IEEE Trans. Ind. Electron., 2025,
       https://doi.org/10.1109/TIE.2024.3429650

    .. [#Var2022] Varatharajan, Pellegrino, Armando, "Direct flux vector control of
       synchronous motor drives: Accurate decoupled control with online adaptive maximum
       torque per ampere and maximum torque per volts evaluation," IEEE Trans. Ind.
       Electron., 2022, https://doi.org/10.1109/TIE.2021.3060665

    """

    def __init__(
        self,
        par: SynchronousMachinePars | SaturatedSynchronousMachinePars,
        cfg: FluxVectorControllerCfg,
        sensorless: bool = True,
        T_s: float = 125e-6,
    ) -> None:
        self.pwm = PWM()
        self.reference_gen = ReferenceGenerator(
            par, cfg.i_s_max, cfg.psi_s_min, cfg.psi_s_max, cfg.k_u, cfg.k_mtpv
        )
        alpha_psi = cfg.alpha_tau if cfg.alpha_psi is None else cfg.alpha_psi
        alpha_i = cfg.alpha_tau if cfg.alpha_i is None else cfg.alpha_i
        self.flux_torque_ctrl = FluxTorqueController(
            par, alpha_psi, cfg.alpha_tau, alpha_i
        )
        if sensorless:
            self.observer = create_sensorless_observer(
                par, cfg.alpha_o, cfg.k_o, cfg.k_f
            )
        else:
            self.observer = create_sensored_observer(
                par, 2 * cfg.alpha_o, cfg.k_o, cfg.k_f
            )
        self.sensorless = sensorless
        self.T_s = T_s

    def get_feedback(self, meas: Measurements) -> ObserverOutputs:
        """Get feedback signals."""
        u_s_ab = self.pwm.get_realized_voltage()
        if self.observer.sensorless:
            fbk = self.observer.compute_output(meas, u_s_ab)
        else:
            fbk = self.observer.compute_output(meas, u_s_ab, meas.w_M)
        fbk.u_dc = meas.u_dc
        return fbk

    def compute_output(self, tau_M_ref: float, fbk: ObserverOutputs) -> References:
        """Compute references."""
        ref = References(T_s=self.T_s, tau_M=tau_M_ref)
        ref.psi_s, ref.tau_M = self.reference_gen.compute_flux_and_torque_refs(
            ref.tau_M, fbk.w_m, fbk.u_dc
        )
        # Current references are not used, but they are computed for plotting
        ref.i_s = self.reference_gen.compute_current_ref(ref.psi_s, ref.tau_M)
        ref.u_s = self.flux_torque_ctrl.compute_output(ref.psi_s, ref.tau_M, fbk)
        u_s_ref_ab = exp(1j * fbk.theta_m) * ref.u_s
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
    i_s_max : float
        Maximum stator current (A).
    alpha_psi : float, optional
        Flux-control bandwidth (rad/s), defaults to 2*pi*100.
    alpha_tau : float, optional
        Torque-control bandwidth (rad/s), defaults to 2*pi*20.
    alpha_d : float, optional
        Rotor-angle estimation bandwidth (rad/s), defaults to 2*pi*200.
    alpha_f : float, optional
        Filter bandwidth (rad/s), defaults to 2*pi*1.
    k_o : Callable[[float], complex], optional
        Observer gain as a function of the rotor angular speed.
    k_u : float, optional
        Voltage utilization factor, defaults to 0.9.
    k_mtpv : float, optional
        MTPV margin, defaults to 0.9.
    psi_s_min : float, optional
        Minimum stator flux (Vs), defaults to `par.psi_f`.
    psi_s_max : float, optional
        Maximum stator flux (Vs), defaults to `inf`.

    """

    i_s_max: float
    alpha_psi: float = 2 * pi * 100
    alpha_tau: float = 2 * pi * 20
    alpha_d: float = 2 * pi * 200
    alpha_f: float = 2 * pi * 1
    k_o: Callable[[float], float] | None = None
    k_u: float = 0.9
    k_mtpv: float = 0.9
    psi_s_min: float | None = None
    psi_s_max: float = inf


class ObserverBasedVHzController:
    """
    Observer-based V/Hz controller for synchronous machine drives.

    This class implements sensorless observer-based V/Hz control. Rotor coordinates and
    decoupling between the stator flux and torque channels are used [#Tii2025a]_.

    Parameters
    ----------
    par : SynchronousMachinePars | SaturatedSynchronousMachinePars
        Machine model parameters.
    cfg : ObserverBasedVHzControllerCfg
        Observer-based V/Hz control configuration.
    T_s : float, optional
        Sampling period (s), defaults to 250e-6.

    """

    def __init__(
        self,
        par: SynchronousMachinePars | SaturatedSynchronousMachinePars,
        cfg: ObserverBasedVHzControllerCfg,
        T_s: float = 250e-6,
    ) -> None:
        self.pwm = PWM()
        self.reference_gen = ReferenceGenerator(
            par, cfg.i_s_max, cfg.psi_s_min, cfg.psi_s_max, cfg.k_u, cfg.k_mtpv
        )
        self.flux_torque_ctrl = FluxTorqueController(par, cfg.alpha_psi, cfg.alpha_tau)
        self.observer = create_vhz_observer(par, cfg.alpha_d, cfg.k_o)
        self.alpha_f: float = cfg.alpha_f
        self.tau_M_lpf: float = 0.0  # Low-pass-filtered torque estimate
        self.n_p = par.n_p
        self.T_s = T_s

    def get_feedback(self, w_M_ref: float, meas: Measurements) -> ObserverOutputs:
        """Get feedback signals."""
        u_s_ab = self.pwm.get_realized_voltage()
        fbk = self.observer.compute_output(meas, u_s_ab, w_M_ref)
        fbk.u_dc = meas.u_dc
        return fbk

    def compute_output(self, fbk: ObserverOutputs) -> References:
        """Calculate references."""
        ref = References(T_s=self.T_s)
        ref.psi_s, ref.tau_M = self.reference_gen.compute_flux_and_torque_refs(
            self.tau_M_lpf, fbk.w_m, fbk.u_dc
        )
        # Current references are not used, but they are computed for plotting
        ref.i_s = self.reference_gen.compute_current_ref(ref.psi_s, ref.tau_M)
        ref.u_s = self.flux_torque_ctrl.compute_output(ref.psi_s, ref.tau_M, fbk)
        u_s_ref_ab = exp(1j * fbk.theta_m) * ref.u_s
        ref.d_abc = self.pwm(ref.T_s, u_s_ref_ab, fbk.u_dc, fbk.w_s)
        return ref

    def update(self, ref: References, fbk: ObserverOutputs) -> None:
        """Update states."""
        self.tau_M_lpf += ref.T_s * self.alpha_f * (fbk.tau_M - self.tau_M_lpf)
        self.observer.update(ref.T_s)
