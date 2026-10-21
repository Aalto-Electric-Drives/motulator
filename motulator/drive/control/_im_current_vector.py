"""Current-vector control methods for induction machine drives."""

from cmath import exp, phase
from collections.abc import Callable
from copy import copy
from dataclasses import dataclass
from math import pi, sqrt

from motulator.common.control import ComplexPIController, DiscreteComplexController
from motulator.common.control._base import TimeSeries
from motulator.common.utils._utils import clip
from motulator.drive.control._common import References
from motulator.drive.control._im_observers import (
    ObserverOutputs,
    compute_feedback,
    create_speed_flux_observer,
    to_rotor_flux_coordinates,
)
from motulator.drive.utils._parameters import (
    InductionMachineInvGammaPars,
    InductionMachinePars,
)


# %%
class CurrentController(ComplexPIController):
    """
    Current controller for induction machines.

    Parameters
    ----------
    par : InductionMachineInvGammaPars | InductionMachinePars
        Machine model parameters.
    alpha_c : float
        Reference-tracking bandwidth (rad/s).
    alpha_i : float, optional
        Integral action bandwidth (rad/s), defaults to `alpha_c`.

    """

    def __init__(
        self,
        par: InductionMachineInvGammaPars | InductionMachinePars,
        alpha_c: float,
        alpha_i: float | None = None,
    ) -> None:
        alpha_i = alpha_c if alpha_i is None else alpha_i
        k_t = alpha_c * par.L_sgm
        k_i = alpha_c * alpha_i * par.L_sgm
        k_p = (alpha_c + alpha_i) * par.L_sgm
        super().__init__(k_p, k_i, k_t)


# %%
class DiscreteCurrentController(DiscreteComplexController):
    """
    Direct discrete-time current controller for induction machines.

    The currents are mapped to the leakage flux linkages, ``psi = L_sgm*i``. The
    resistive voltage drop and the back-emf are disturbances, which the integral action
    compensates for. Since the design includes the delays, the angle of the voltage
    reference is not advanced (`k_comp = 0` in `CurrentVectorController`).

    Parameters
    ----------
    par : InductionMachineInvGammaPars | InductionMachinePars
        Machine model parameters.
    alpha_c : float
        Reference-tracking bandwidth (rad/s).
    T_s : float
        Sampling period (s).

    """

    def __init__(
        self,
        par: InductionMachineInvGammaPars | InductionMachinePars,
        alpha_c: float,
        T_s: float,
    ) -> None:
        self.par = par
        super().__init__(alpha_c, T_s)

    def compute_output(self, i_ref: complex, i: complex) -> complex:
        # Extends the base class method by mapping the currents to the flux linkages
        L_sgm = self.par.L_sgm
        return super().compute_output(L_sgm * i_ref, L_sgm * i)


# %%
class CurrentReferenceGenerator:
    """
    Current reference generator.

    In the base-speed region, the current reference in rotor-flux coordinates is::

        i_s_ref = i_sd_nom + 1j*tau_M_ref/(1.5*n_p*abs(psi_R))

    where `psi_R` is the estimated rotor flux. The nominal flux-producing current
    component is computed from the nominal stator flux in the no-load condition::

        i_sd_nom = psi_s_nom/(L_M + L_sgm)

    If the saturation is modeled, the inductances are evaluated at `psi_s_nom`. In the
    field-weakening operation, the flux-producing current component is::

        i_s_ref.real = (k_fw/s)*(u_s_max - abs(u_s_ref))

    where `1/s` refers to integration, ``u_s_max = k_u*u_dc/sqrt(3)`` is the maximum
    stator voltage in the linear modulation region, `u_s_ref` is the (unlimited) stator
    voltage reference, and `k_fw` is the field-weakening gain. The field-weakening
    method and its tuning corresponds roughly to [#Hin2006]_. Furthermore, the torque-
    producing current component `i_s_ref.imag` is limited based on the maximum stator
    current and the breakdown slip.

    Parameters
    ----------
    par : InductionMachineInvGammaPars | InductionMachinePars
        Machine model parameters.
    psi_s_nom : float
        Nominal stator flux linkage (Vs).
    i_s_max : float
        Maximum stator current (A).
    w_s_nom : float
        Nominal stator angular frequency (rad/s).
    k_u : float
        Voltage utilization factor.
    k_fw : float | None
        Field-weakening gain (1/H). If None, ``2*R_R/(w_s_nom*L_sgm**2)`` is used.

    References
    ----------
    .. [#Hin2006] Hinkkanen, Luomi, "Braking scheme for vector-controlled induction
       motor drives equipped with diode rectifier without braking resistor," IEEE Trans.
       Ind. Appl., 2006, https://doi.org/10.1109/TIA.2006.880852

    """

    def __init__(
        self,
        par: InductionMachineInvGammaPars | InductionMachinePars,
        psi_s_nom: float,
        i_s_max: float,
        w_s_nom: float,
        k_u: float,
        k_fw: float | None,
    ) -> None:
        self.par = par
        self.i_s_max = i_s_max
        self.k_u = k_u
        # Magnetizing current at the nominal flux, taking the saturation into account
        par_nom = copy(par)
        par_nom.update_psi_s(psi_s_nom)
        self.i_sd_nom = psi_s_nom / (par_nom.L_M + par_nom.L_sgm)
        self.k_fw = 2 * par.R_R / (w_s_nom * par.L_sgm**2) if k_fw is None else k_fw
        self.i_sd_ref = self.i_sd_nom  # Integral state

    def compute_output(self, tau_M_ref: float, psi_R: float) -> tuple[complex, float]:
        """
        Compute the stator current reference.

        Parameters
        ----------
        tau_M_ref : float
            Torque reference (Nm).
        psi_R : float
            Estimated rotor flux magnitude (Vs).

        Returns
        -------
        tuple[complex, float]
            Stator current reference (A) and limited torque reference (Nm).

        """
        par = self.par  # Unpack
        i_sd_ref = self.i_sd_ref  # Get the state

        def q_axis_current_limit(i_sd_ref: float, psi_R: float) -> float:
            # Priority given to the d component
            max_i_sq1 = sqrt(self.i_s_max**2 - i_sd_ref**2)
            # Breakdown torque limit
            i_sq_max2 = psi_R / par.L_sgm + i_sd_ref
            # q-axis current limit
            i_sq_max = min(max_i_sq1, i_sq_max2)
            return i_sq_max

        # q-axis current reference
        i_sq_ref = tau_M_ref / (1.5 * par.n_p * psi_R) if psi_R > 0 else 0

        # Limit the current
        i_sq_max = q_axis_current_limit(i_sd_ref, psi_R)
        i_sq_ref = clip(i_sq_ref, -i_sq_max, i_sq_max)

        # Current reference
        i_s_ref = i_sd_ref + 1j * i_sq_ref

        # Limited torque (for the speed controller)
        tau_M_ref = 1.5 * par.n_p * psi_R * i_sq_ref

        return i_s_ref, tau_M_ref

    def update(self, T_s: float, u_s_ref: complex, u_dc: float) -> None:
        """
        Field-weakening based on the unlimited reference voltage.

        Parameters
        ----------
        T_s : float
            Sampling time (s).
        u_s_ref : complex
            Unlimited stator voltage reference (V).
        u_dc : float
            DC-link voltage (V).

        """
        u_s_max = self.k_u * u_dc / sqrt(3)
        self.i_sd_ref += T_s * self.k_fw * (u_s_max - abs(u_s_ref))
        self.i_sd_ref = clip(self.i_sd_ref, -self.i_s_max, self.i_sd_nom)


# %%
@dataclass
class CurrentVectorControllerCfg:
    """
    Current vector controller configuration.

    Parameters
    ----------
    psi_s_nom : float
        Nominal stator flux linkage (Vs).
    i_s_max : float
        Maximum stator current (A).
    alpha_c : float, optional
        Current control reference-tracking bandwidth (rad/s), defaults to 2*pi*200.
    alpha_i : float, optional
        Current control integral-action bandwidth (rad/s), defaults to `alpha_c`. Not
        used if `discrete` is True.
    alpha_o : float | None, optional
        Speed estimation poles (rad/s). Defaults to 2*pi*60 if `sensorless` is True,
        otherwise 2*pi*400. If `J` is given, the default is halved, which keeps the
        speed estimation gain approximately the same.
    k_o : Callable[[float], complex] | None, optional
        Observer gain as a function of the electrical rotor angular speed. If None,
        defaults to ``lambda w_m: (0.5*alpha + 0.2*abs(w_m))/(alpha - 1j*w_m)`` if
        `sensorless` is True, otherwise ``lambda w_m: 1 + 0.2*abs(w_m)/(alpha -
        1j*w_m)``, where ``alpha = R_R/L_M``.
    w_s_nom : float, optional
        Nominal stator angular frequency (rad/s), defaults to 2*pi*50.
    k_u : float, optional
        Voltage utilization factor, defaults to 0.95.
    k_fw : float | None, optional
        Field-weakening gain (1/H), defaults to `2*R_R/(w_s_nom*L_sgm**2)`.
    J : float | None, optional
        Inertia (kgm²). Defaults to None, meaning the mechanical system model is not
        used in speed estimation.
    sensorless : bool, optional
        If True, sensorless control is used, defaults to True.
    T_s : float, optional
        Sampling period (s), defaults to 125e-6.
    discrete : bool, optional
        If True, the direct discrete-time current controller is used instead of the
        continuous-time design, defaults to False.

    """

    psi_s_nom: float
    i_s_max: float
    alpha_c: float = 2 * pi * 200
    alpha_i: float | None = None
    alpha_o: float | None = None
    k_o: Callable[[float], complex] | None = None
    w_s_nom: float = 2 * pi * 50
    k_u: float = 0.95
    k_fw: float | None = None
    J: float | None = None
    sensorless: bool = True
    T_s: float = 125e-6
    discrete: bool = False


class CurrentVectorController:
    """
    Current-vector controller for induction machine drives.

    Parameters
    ----------
    par : InductionMachineInvGammaPars | InductionMachinePars
        Machine model parameters.
    cfg : CurrentVectorControllerCfg
        Current-vector controller configuration.

    """

    def __init__(
        self,
        par: InductionMachineInvGammaPars | InductionMachinePars,
        cfg: CurrentVectorControllerCfg,
    ) -> None:
        par = copy(par)  # The observer updates the saturation state of this copy
        self.cfg = cfg
        self.sensorless = cfg.sensorless
        self.reference_gen = CurrentReferenceGenerator(
            par, cfg.psi_s_nom, cfg.i_s_max, cfg.w_s_nom, cfg.k_u, cfg.k_fw
        )
        self.current_ctrl: CurrentController | DiscreteCurrentController
        if cfg.discrete:
            self.current_ctrl = DiscreteCurrentController(par, cfg.alpha_c, cfg.T_s)
        else:
            self.current_ctrl = CurrentController(par, cfg.alpha_c, cfg.alpha_i)
        self.observer = create_speed_flux_observer(
            par, cfg.alpha_o, cfg.T_s, cfg.k_o, cfg.sensorless, cfg.J
        )
        # The discrete-time current controller compensates for the delays itself
        self.k_comp = 0.0 if cfg.discrete else 1.5
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
        ref = References(T_s=self.T_s, tau_M=tau_M_ref)
        ref.i_s, ref.tau_M = self.reference_gen.compute_output(
            ref.tau_M, abs(fbk.psi_R)
        )
        # Transform the reference to the same coordinates as the feedback
        ref.i_s = exp(1j * phase(fbk.psi_R)) * ref.i_s
        ref.u_s = self.current_ctrl.compute_output(ref.i_s, fbk.i_s)
        return ref

    def update(self, ref: References, fbk: ObserverOutputs) -> None:
        """Update states."""
        self.observer.update(ref.T_s, fbk)
        self.reference_gen.update(ref.T_s, ref.u_s, fbk.u_dc)
        # The discrete-time controller takes its limited output, except with blocked
        # gates, when the open-circuit voltage of the observer is the realized one
        u_s = ref.u_s_lim if self.cfg.discrete and ref.enabled else fbk.u_s
        self.current_ctrl.update(ref.T_s, u_s, fbk.w_c)

    def post_process(self, ts: TimeSeries) -> None:
        """Post-process controller time series."""
        to_rotor_flux_coordinates(ts)
