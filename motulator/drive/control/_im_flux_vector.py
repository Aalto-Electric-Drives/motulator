"""Flux-vector control of induction machine drives."""

from collections.abc import Callable
from dataclasses import dataclass
from math import inf, pi, sqrt

from motulator.common.control._base import TimeSeries
from motulator.common.utils._utils import sign
from motulator.drive.control._common import FluxTorqueControllerBase, References
from motulator.drive.control._im_observers import (
    FluxObserver,
    ObserverOutputs,
    compute_feedback,
    create_speed_flux_observer,
    sensorless_k_o,
    to_rotor_flux_coordinates,
)
from motulator.drive.utils._parameters import (
    BaseInductionMachineControlPars,
    InductionMachineInvGammaPars,
)


# %%
class FluxTorqueController(FluxTorqueControllerBase):
    """
    Flux and torque controller for induction machines.

    Parameters
    ----------
    par : BaseInductionMachineControlPars
        Machine model parameters.
    alpha_psi : float
        Flux-control bandwidth (rad/s).
    alpha_tau : float
        Torque-control bandwidth (rad/s).
    alpha_i : float, optional
        Integral-action bandwidth (rad/s), defaults to 0.

    """

    def compute_output(
        self,
        psi_s_ref: float,
        tau_M_ref: float,
        fbk: ObserverOutputs,
        op_par: InductionMachineInvGammaPars,
    ) -> complex:
        """
        Calculate the voltage reference.

        Parameters
        ----------
        psi_s_ref : float
            Stator flux reference (Vs).
        tau_M_ref : float
            Torque reference (Nm).
        fbk : ObserverOutputs
            Feedback signals.
        op_par : InductionMachineInvGammaPars
            Inverse-Γ model parameters at the operating point.

        Returns
        -------
        complex
            Voltage reference (V).

        """
        i_a = fbk.psi_R / op_par.L_sgm
        return self._compute_voltage_ref(psi_s_ref, tau_M_ref, fbk, i_a, fbk.w_s)


# %%
class ReferenceGenerator:
    """
    Reference generator for flux-vector control and observer-based V/Hz control.

    Parameters
    ----------
    psi_s_nom : float
        Nominal stator flux linkage (Vs).
    i_s_max : float
        Maximum stator current (A).
    tau_M_max : float
        Maximum torque reference (Nm).
    k_u : float
        Voltage utilization factor.
    k_b : float
        Breakdown torque margin.

    """

    def __init__(
        self, psi_s_nom: float, i_s_max: float, tau_M_max: float, k_u: float, k_b: float
    ) -> None:
        self.psi_s_nom = psi_s_nom
        self.i_s_max = i_s_max
        self.tau_M_max = tau_M_max
        self.k_u = k_u
        self.k_b = k_b

    def compute_output(
        self,
        tau_M_ref: float,
        w_s: float,
        psi_R: float,
        u_dc: float,
        op_par: InductionMachineInvGammaPars,
    ) -> tuple[float, float]:
        """
        Compute the stator flux and limited torque references.

        Parameters
        ----------
        tau_M_ref : float
            Torque reference (Nm).
        w_s : float
            Stator angular frequency estimate (rad/s).
        psi_R : float
            Rotor flux magnitude estimate (Vs).
        u_dc : float
            DC-bus voltage (V).
        op_par : InductionMachineInvGammaPars
            Inverse-Γ model parameters at the operating point.

        Returns
        -------
        tuple[float, float]
            Stator flux magnitude reference (Vs) and limited torque reference (Nm).

        """
        # Field-weakening
        u_s_max = self.k_u * u_dc / sqrt(3)
        psi_s_max = u_s_max / abs(w_s) if w_s != 0 else inf
        # Current limit
        psi_s_cl = op_par.L_sgm * self.i_s_max + psi_R
        # Stator flux reference
        psi_s_ref = min(psi_s_max, self.psi_s_nom, psi_s_cl)

        # Torque reference limiting
        # Breakdown torque limit
        k = self.k_b * op_par.L_M / (op_par.L_M + op_par.L_sgm)
        tau_b = 0.75 * op_par.n_p * k * psi_s_ref**2 / op_par.L_sgm
        # Current limit: the stator flux on the circle psi_s_ref stays within the circle
        # L_sgm*i_s_max around the rotor flux, which gives the maximum q-component of
        # the stator flux in rotor flux coordinates
        tau_M_cl = 0.0
        if psi_R > 0:
            psi_sd = (psi_s_ref**2 + psi_R**2 - (op_par.L_sgm * self.i_s_max) ** 2) / (
                2 * psi_R
            )
            psi_sq = sqrt(max(psi_s_ref**2 - max(psi_sd, 0) ** 2, 0))
            tau_M_cl = 1.5 * op_par.n_p * psi_R * psi_sq / op_par.L_sgm
        # Limited torque reference
        tau_M_ref = min(abs(tau_M_ref), self.tau_M_max, tau_b, tau_M_cl) * sign(
            tau_M_ref
        )
        return psi_s_ref, tau_M_ref


# %%
@dataclass(kw_only=True)
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
    alpha_psi : float | None, optional
        Flux-control bandwidth (rad/s), defaults to `alpha_tau`.
    alpha_i : float | None, optional
        Integral-action bandwidth (rad/s), defaults to `alpha_tau`.
    alpha_o : float | None, optional
        Speed estimation poles (rad/s). Defaults to 2*pi*60 if `sensorless` is True,
        otherwise 2*pi*400. If `J` is given, the default is halved, which keeps the
        speed estimation gain approximately the same.
    k_o : Callable[[float, InductionMachineInvGammaPars], complex] | None, optional
        Observer gain as a function of the electrical rotor angular speed and the
        inverse-Γ model parameters at the operating point. If None, defaults to
        ``lambda w_m, par: (0.5*par.alpha + 0.2*abs(w_m))/(par.alpha - 1j*w_m)`` if
        `sensorless` is True, otherwise ``lambda w_m, par: 1 + 0.2*abs(w_m)/(par.alpha
        - 1j*w_m)``.
    tau_M_max : float, optional
        Maximum torque reference (Nm), defaults to `inf`.
    k_u : float, optional
        Voltage utilization factor, defaults to 0.9.
    k_b : float, optional
        Breakdown torque margin, defaults to 0.9.
    J : float | None, optional
        Inertia (kgm²). Defaults to None, meaning the mechanical system model is not
        used in speed estimation.
    sensorless : bool, optional
        If True, sensorless control is used, defaults to True.
    T_s : float, optional
        Sampling period (s), defaults to 125e-6.

    """

    psi_s_nom: float
    i_s_max: float
    alpha_tau: float = 2 * pi * 100
    alpha_psi: float | None = None
    alpha_i: float | None = None
    alpha_o: float | None = None
    k_o: Callable[[float, InductionMachineInvGammaPars], complex] | None = None
    tau_M_max: float = inf
    k_u: float = 0.9
    k_b: float = 0.9
    J: float | None = None
    sensorless: bool = True
    T_s: float = 125e-6


# %%
class FluxVectorController:
    """
    Flux-vector controller for induction machine drives.

    This class implements a variant of flux-vector control. Decoupling between the
    stator flux and torque channels is used [#Tii2025b]_.

    Parameters
    ----------
    par : BaseInductionMachineControlPars
        Machine model parameters.
    cfg : FluxVectorControllerCfg
        Flux-vector controller configuration.

    References
    ----------
    .. [#Tii2025b] Tiitinen, Hinkkanen, Harnefors, "Sensorless flux-vector control
       framework: An extension for induction machines," IEEE Trans. Ind. Electron.,
       2025, https://doi.org/10.1109/TIE.2025.3559958

    """

    k_comp = 1.5

    def __init__(
        self, par: BaseInductionMachineControlPars, cfg: FluxVectorControllerCfg
    ) -> None:
        self.par = par
        self.cfg = cfg
        self.reference_gen = ReferenceGenerator(
            cfg.psi_s_nom, cfg.i_s_max, cfg.tau_M_max, cfg.k_u, cfg.k_b
        )
        alpha_psi = cfg.alpha_tau if cfg.alpha_psi is None else cfg.alpha_psi
        alpha_i = cfg.alpha_tau if cfg.alpha_i is None else cfg.alpha_i
        self.flux_torque_ctrl = FluxTorqueController(
            par, alpha_psi, cfg.alpha_tau, alpha_i
        )
        self.observer = create_speed_flux_observer(
            par, cfg.alpha_o, cfg.T_s, cfg.k_o, cfg.sensorless, cfg.J
        )
        self.sensorless = cfg.sensorless
        self.T_s = cfg.T_s

    def get_feedback(
        self,
        u_s_ab: complex | None,
        i_s_ab: complex,
        w_M_meas: float | None,
        theta_M_meas: float | None,
    ) -> ObserverOutputs:
        """Get the feedback signals."""
        return compute_feedback(
            self.observer, self.sensorless, u_s_ab, i_s_ab, w_M_meas
        )

    def compute_output(self, tau_M_ref: float, fbk: ObserverOutputs) -> References:
        """Compute references."""
        op_par = self.observer.op_par
        ref = References(T_s=self.T_s, tau_M=tau_M_ref)
        ref.psi_s, ref.tau_M = self.reference_gen.compute_output(
            ref.tau_M, fbk.w_s, abs(fbk.psi_R), fbk.u_dc, op_par
        )
        ref.u_s = self.flux_torque_ctrl.compute_output(
            ref.psi_s, ref.tau_M, fbk, op_par
        )
        return ref

    def update(self, ref: References, fbk: ObserverOutputs) -> None:
        """Update states."""
        self.observer.update(ref.T_s, fbk)
        self.flux_torque_ctrl.update(ref.T_s, fbk)

    def post_process(self, ts: TimeSeries) -> None:
        """Post-process controller time series."""
        to_rotor_flux_coordinates(ts)


# %%
@dataclass(kw_only=True)
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
        Bandwidth of the low-pass filter of the torque estimate (rad/s), defaults to
        2*pi*1.
    k_o : Callable[[float, InductionMachineInvGammaPars], complex] | None, optional
        Observer gain as a function of the electrical rotor angular speed and the
        inverse-Γ model parameters at the operating point. If None, defaults to
        ``lambda w_m, par: (0.5*par.alpha + 0.2*abs(w_m))/(par.alpha - 1j*w_m)``.
    k_u : float, optional
        Voltage utilization factor, defaults to 0.9.
    k_b : float, optional
        Breakdown torque margin, defaults to 0.9.
    T_s : float, optional
        Sampling period (s), defaults to 250e-6.

    """

    psi_s_nom: float
    i_s_max: float
    alpha_psi: float = 2 * pi * 100
    alpha_tau: float = 2 * pi * 20
    alpha_f: float = 2 * pi * 1
    k_o: Callable[[float, InductionMachineInvGammaPars], complex] | None = None
    k_u: float = 0.9
    k_b: float = 0.9
    T_s: float = 250e-6


class ObserverBasedVHzController:
    """
    Observer-based V/Hz controller for induction machine drives.

    This class implements sensorless observer-based V/Hz control. Decoupling between the
    stator flux and torque channels is used [#Tii2025b]_.

    Parameters
    ----------
    par : BaseInductionMachineControlPars
        Machine model parameters.
    cfg : ObserverBasedVHzControllerCfg
        Observer-based V/Hz controller configuration.

    """

    k_comp = 1.5

    def __init__(
        self, par: BaseInductionMachineControlPars, cfg: ObserverBasedVHzControllerCfg
    ) -> None:
        self.par = par
        self.cfg = cfg
        self.reference_gen = ReferenceGenerator(
            cfg.psi_s_nom, cfg.i_s_max, inf, cfg.k_u, cfg.k_b
        )
        self.flux_torque_ctrl = FluxTorqueController(par, cfg.alpha_psi, cfg.alpha_tau)
        k_o = sensorless_k_o if cfg.k_o is None else cfg.k_o
        self.observer = FluxObserver(par, k_o)
        self.tau_M_lpf: float = 0.0  # Low-pass-filtered torque estimate
        self.T_s = cfg.T_s

    def get_feedback(
        self, u_s_ab: complex | None, i_s_ab: complex, w_M_ref: float
    ) -> ObserverOutputs:
        """Get feedback signals."""
        return self.observer.compute_output(u_s_ab, i_s_ab, w_M_ref)

    def compute_output(self, fbk: ObserverOutputs) -> References:
        """Compute references."""
        op_par = self.observer.op_par
        ref = References(T_s=self.T_s)
        ref.psi_s, ref.tau_M = self.reference_gen.compute_output(
            self.tau_M_lpf, fbk.w_s, abs(fbk.psi_R), fbk.u_dc, op_par
        )
        ref.u_s = self.flux_torque_ctrl.compute_output(
            ref.psi_s, ref.tau_M, fbk, op_par
        )
        return ref

    def compute_standby_output(self, fbk: ObserverOutputs) -> References:
        """Compute references while the gates are blocked."""
        # The flux and torque controller tracks the estimated flux and torque
        ref = References(T_s=self.T_s, tau_M=fbk.tau_M, psi_s=abs(fbk.psi_s))
        ref.u_s = self.flux_torque_ctrl.compute_output(
            ref.psi_s, ref.tau_M, fbk, self.observer.op_par
        )
        return ref

    def update(self, ref: References, fbk: ObserverOutputs) -> None:
        """Update states."""
        self.tau_M_lpf += ref.T_s * self.cfg.alpha_f * (fbk.tau_M - self.tau_M_lpf)
        self.observer.update(ref.T_s, fbk)

    def post_process(self, ts: TimeSeries) -> None:
        """Post-process controller time series."""
        to_rotor_flux_coordinates(ts)
