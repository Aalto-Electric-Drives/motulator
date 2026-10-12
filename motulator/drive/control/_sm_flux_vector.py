"""Flux-vector control of synchronous machine drives."""

from collections.abc import Callable
from dataclasses import dataclass
from math import inf, pi

from motulator.common.control._base import TimeSeries
from motulator.common.control._pwm import PWM
from motulator.drive.control._common import (
    FluxTorqueControllerBase,
    References,
    matched_pole,
)
from motulator.drive.control._sm_observers import (
    ObserverOutputs,
    compute_feedback,
    create_speed_flux_observer,
    create_vhz_observer,
)
from motulator.drive.control._sm_reference_gen import (
    ReferenceGenerator,
    ReferenceGeneratorOnline,
)
from motulator.drive.utils._parameters import (
    SaturatedSynchronousMachinePars,
    SynchronousMachinePars,
)


# %%
class FluxTorqueController(FluxTorqueControllerBase):
    """
    Flux and torque controller for synchronous machines.

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
        super().__init__(par, alpha_psi, alpha_tau, alpha_i)
        self.par = par
        self.x_psi = alpha_i * par.psi_f

    def _aux_current(self, psi_s: complex, i_s: complex) -> complex:
        """Auxiliary current vector."""
        L_s = self.par.incr_ind_mat(i_s)
        L_dd, L_dq, L_qq = L_s[0, 0], L_s[0, 1], L_s[1, 1]
        det_L = L_dd * L_qq - L_dq**2
        return (
            L_dd * psi_s.real + 1j * L_qq * psi_s.imag + 1j * L_dq * psi_s.conjugate()
        ) / det_L - i_s

    def compute_output(
        self, psi_s_ref: float, tau_M_ref: float, fbk: ObserverOutputs
    ) -> complex:
        """Calculate the voltage reference."""
        i_a = self._aux_current(fbk.psi_s, fbk.i_s)
        # If the torque-production factor is negative, the torque cannot be controlled.
        # In a PM machine with weak PMs, this occurs at a high flux and a small load
        # angle (psi_s.real > 0), where the flux is reduced until the torque becomes
        # controllable again. Beyond the MTPV limit (psi_s.real < 0 for PM machines),
        # reducing the flux would not help.
        c_tau_negative = (i_a * fbk.psi_s.conjugate()).real < 0
        if c_tau_negative and self.par.psi_f > 0 and fbk.psi_s.real > 0:
            psi_s_ref = 0.0
        return self._compute_voltage_ref(psi_s_ref, tau_M_ref, fbk, i_a, fbk.w_m)


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
    alpha_psi : float | None, optional
        Flux-control bandwidth (rad/s), defaults to `alpha_tau`.
    alpha_i : float | None, optional
        Integral-action bandwidth (rad/s), defaults to `alpha_tau`.
    alpha_o : float | None, optional
        Speed estimation poles (rad/s). Defaults to 2*pi*50 if `sensorless` is True,
        otherwise 2*pi*400. If `J` is given, the default is divided by sqrt(3), which
        keeps the speed estimation gain approximately the same.
    alpha_ref : float, optional
        Reference generation bandwidth (rad/s), defaults to 2*pi*100.
    k_o : Callable[[float], float] | None, optional
        Observer gain as a function of the electrical rotor angular speed. If None,
        defaults to ``lambda w_m: 0.25*R_s*(1/L_d + 1/L_q) + 0.2*abs(w_m)`` if
        `sensorless` is True (incremental inductances at zero current), otherwise
        ``lambda w_m: 2*pi*15``.
    k_f : Callable[[float], float] | None, optional
        PM-flux estimation gain as a function of the electrical rotor angular speed. If
        None, the gain is zero, i.e., the PM-flux linkage is not estimated.
    psi_s_min : float | None, optional
        Minimum stator flux (Vs). If None, defaults to `par.psi_f` elsewhere. It must be
        positive for machines without PMs.
    psi_s_max : float, optional
        Maximum stator flux (Vs), defaults to `inf`.
    k_u : float, optional
        Voltage utilization factor, defaults to 0.9.
    k_mtpv : float, optional
        MTPV margin, defaults to 0.85.
    J : float | None, optional
        Inertia (kgm²). Defaults to None, meaning the mechanical system model is not
        used in speed estimation.
    sensorless : bool, optional
        If True, sensorless control is used, defaults to True.
    online_ref : bool, optional
        If True, the online reference generation is used, defaults to False.
    T_s : float, optional
        Sampling period (s), defaults to 125e-6.

    """

    i_s_max: float
    alpha_tau: float = 2 * pi * 100
    alpha_psi: float | None = None
    alpha_i: float | None = None
    alpha_o: float | None = None
    alpha_ref: float = 2 * pi * 100
    k_o: Callable[[float], float] | None = None
    k_f: Callable[[float], float] | None = None
    psi_s_min: float | None = None
    psi_s_max: float = inf
    k_u: float = 0.9
    k_mtpv: float = 0.85
    J: float | None = None
    sensorless: bool = True
    online_ref: bool = False
    T_s: float = 125e-6


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
    ) -> None:
        reference_generator = (
            ReferenceGeneratorOnline if cfg.online_ref else ReferenceGenerator
        )
        self.reference_gen = reference_generator(
            par,
            cfg.i_s_max,
            cfg.psi_s_min,
            cfg.psi_s_max,
            cfg.k_u,
            cfg.k_mtpv,
            cfg.alpha_ref,
        )
        alpha_psi = cfg.alpha_tau if cfg.alpha_psi is None else cfg.alpha_psi
        alpha_i = cfg.alpha_tau if cfg.alpha_i is None else cfg.alpha_i
        self.flux_torque_ctrl = FluxTorqueController(
            par, alpha_psi, cfg.alpha_tau, alpha_i
        )
        self.observer = create_speed_flux_observer(
            par, cfg.alpha_o, cfg.T_s, cfg.k_o, cfg.k_f, cfg.sensorless, cfg.J
        )
        self.cfg = cfg
        self.par = par
        self.sensorless = cfg.sensorless
        self.online_ref = cfg.online_ref

    def default_pwm(self) -> PWM:
        """Get the default pulse-width modulator."""
        return PWM()

    def get_feedback(
        self,
        u_s_ab: complex,
        i_s_ab: complex,
        w_M_meas: float | None,  # Not used, needed for the interface
        theta_M_meas: float | None,
    ) -> ObserverOutputs:
        """Get the feedback signals."""
        return compute_feedback(
            self.observer, self.sensorless, u_s_ab, i_s_ab, theta_M_meas
        )

    def compute_output(self, tau_M_ref: float, fbk: ObserverOutputs) -> References:
        """Compute references."""
        ref = References(T_s=self.cfg.T_s, tau_M=tau_M_ref)
        ref.psi_s, ref.tau_M = self.reference_gen.compute_flux_and_torque_refs(
            ref.tau_M, fbk.w_m, fbk.u_dc
        )
        ref.u_s = self.flux_torque_ctrl.compute_output(ref.psi_s, ref.tau_M, fbk)
        return ref

    def update(self, ref: References, fbk: ObserverOutputs) -> None:
        """Update states."""
        self.observer.update(ref.T_s, fbk)
        self.flux_torque_ctrl.update(ref.T_s, fbk)
        if self.online_ref:
            self.reference_gen.update(ref.T_s)

    def post_process(self, ts: TimeSeries) -> None:
        """Post-process controller time series."""


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
    alpha_f : float, optional
        Filter bandwidth (rad/s), defaults to 2*pi*1.
    alpha_o : float, optional
        Angle estimation pole (rad/s), defaults to 2*pi*200.
    k_o : Callable[[float], float] | None, optional
        Observer gain as a function of the electrical rotor angular speed. If None,
        defaults to ``lambda w_m: 0.25*R_s*(1/L_d + 1/L_q) + 0.2*abs(w_m)``
        (incremental inductances at zero current).
    k_u : float, optional
        Voltage utilization factor, defaults to 0.9.
    k_mtpv : float, optional
        MTPV margin, defaults to 0.9.
    psi_s_min : float | None, optional
        Minimum stator flux (Vs), defaults to `par.psi_f` elsewhere. It must be
        positive for machines without PMs.
    psi_s_max : float, optional
        Maximum stator flux (Vs), defaults to `inf`.
    T_s : float, optional
        Sampling period (s), defaults to 250e-6.

    """

    i_s_max: float
    alpha_psi: float = 2 * pi * 100
    alpha_tau: float = 2 * pi * 20
    alpha_f: float = 2 * pi * 1
    alpha_o: float = 2 * pi * 200
    k_o: Callable[[float], float] | None = None
    k_u: float = 0.9
    k_mtpv: float = 0.9
    psi_s_min: float | None = None
    psi_s_max: float = inf
    T_s: float = 250e-6


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

    """

    def __init__(
        self,
        par: SynchronousMachinePars | SaturatedSynchronousMachinePars,
        cfg: ObserverBasedVHzControllerCfg,
    ) -> None:
        self.cfg = cfg
        self.reference_gen = ReferenceGenerator(
            par, cfg.i_s_max, cfg.psi_s_min, cfg.psi_s_max, cfg.k_u, cfg.k_mtpv, 0.0
        )  # The current reference is not used, so its tracking bandwidth is zero
        self.flux_torque_ctrl = FluxTorqueController(par, cfg.alpha_psi, cfg.alpha_tau)
        k_theta = matched_pole(cfg.alpha_o, cfg.T_s)  # First-order angle estimation
        self.observer = create_vhz_observer(par, k_theta, cfg.k_o)
        self.alpha_f: float = cfg.alpha_f
        self.tau_M_lpf: float = 0.0  # Low-pass-filtered torque estimate
        self.T_s = cfg.T_s

    def default_pwm(self) -> PWM:
        """Get the default pulse-width modulator."""
        return PWM(overmodulation="MME")

    def get_feedback(
        self, u_s_ab: complex, i_s_ab: complex, w_M_ref: float
    ) -> ObserverOutputs:
        """Get feedback signals."""
        return self.observer.compute_output(u_s_ab, i_s_ab, w_M_ref)

    def compute_output(self, fbk: ObserverOutputs) -> References:
        """Calculate references."""
        ref = References(T_s=self.cfg.T_s)
        ref.psi_s, ref.tau_M = self.reference_gen.compute_flux_and_torque_refs(
            self.tau_M_lpf, fbk.w_m, fbk.u_dc
        )
        ref.u_s = self.flux_torque_ctrl.compute_output(ref.psi_s, ref.tau_M, fbk)
        return ref

    def update(self, ref: References, fbk: ObserverOutputs) -> None:
        """Update states."""
        self.tau_M_lpf += ref.T_s * self.alpha_f * (fbk.tau_M - self.tau_M_lpf)
        self.observer.update(ref.T_s, fbk)

    def post_process(self, ts: TimeSeries) -> None:
        """Post-process controller time series."""
