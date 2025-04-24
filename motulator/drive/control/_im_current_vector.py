"""Current-vector control methods for induction machine drives."""

from cmath import exp
from dataclasses import dataclass, field
from math import pi, sqrt
from typing import Callable, Sequence

from motulator.common.control import ComplexPIController
from motulator.common.control._pwm import PWM
from motulator.common.utils import clip
from motulator.drive.control._base import Measurements
from motulator.drive.control._im_observers import (
    ObserverOutputs,
    create_sensored_observer,
    create_sensorless_observer,
)
from motulator.drive.utils._parameters import InductionMachineInvGammaPars


# %%
@dataclass
class References:
    """Reference signals for current-vector control."""

    T_s: float = 0.0
    d_abc: Sequence[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    tau_M: float = 0.0
    u_s: complex = 0j
    i_s: complex = 0j


# %%
class CurrentController(ComplexPIController):
    """
    Current controller for induction machines.

    Parameters
    ----------
    par : InductionMachineInvGammaPars
        Machine model parameters.
    alpha_c : float, optional
        Reference-tracking bandwidth (rad/s).
    alpha_i : float, optional
        Integral action bandwidth (rad/s), defaults to `alpha_c`.

    """

    def __init__(
        self,
        par: InductionMachineInvGammaPars,
        alpha_c: float,
        alpha_i: float | None = None,
    ) -> None:
        alpha_i = alpha_c if alpha_i is None else alpha_i
        k_t = alpha_c * par.L_sgm
        k_i = alpha_c * alpha_i * par.L_sgm
        k_p = (alpha_c + alpha_i) * par.L_sgm
        super().__init__(k_p, k_i, k_t)


# %%
class CurrentReferenceGenerator:
    """
    Current reference generator.

    In the base-speed region, the current reference in rotor-flux coordinates is::

        i_s_ref = psi_R_ref/L_M + 1j*tau_M_ref/(1.5*n_p*abs(psi_R))

    where `psi_R_ref` is the reference for the rotor flux magnitude and `psi_R` is the
    estimated rotor flux. The field-weakening operation is based adjusting the flux-
    producing current component::

        i_s_ref.real = (k_fw/s)*(u_s_max - abs(u_s_ref))

    where `1/s` refers to integration, ``u_s_max = k_u*u_dc/sqrt(3)`` is the maximum
    stator voltage in the linear modulation region, `u_s_ref` is the (unlimited) stator
    voltage reference, and `k_fw` is the field-weakening gain. The field-weakening
    method and its tuning corresponds roughly to [#Hin2006]_. Furthermore, the torque-
    producing current component `i_s_ref.imag` is limited based on the maximum stator
    current and the breakdown slip.

    The nominal flux-producing current component is computed from the nominal stator
    flux in the no-load condition::

        i_sd_nom = psi_s_nom/(L_M + L_sgm)

    Parameters
    ----------
    machine_pars : InductionMachineInvGammaPars
        Machine model parameters.
    psi_s_nom : float
        Nominal stator flux linkage (Vs).
    i_s_max : float
        Maximum stator current (A).
    w_s_nom : float, optional
        Nominal stator angular frequency (rad/s).
    k_u : float, optional
        Voltage utilization factor, defaults to 1.
    k_fw : float, optional
        Field-weakening gain (1/H), defaults to `2*R_R/(w_s_nom*L_sgm**2)`.

    References
    ----------
    .. [#Hin2006] Hinkkanen, Luomi, "Braking scheme for vector-controlled induction
       motor drives equipped with diode rectifier without braking resistor," IEEE Trans.
       Ind. Appl., 2006, https://doi.org/10.1109/TIA.2006.880852

    """

    def __init__(
        self,
        par: InductionMachineInvGammaPars,
        psi_s_nom: float,
        i_s_max: float,
        w_s_nom: float,
        k_u: float = 1.0,
        k_fw: float = 0.0,
    ) -> None:
        self.par = par
        self.i_s_max = i_s_max
        self.k_u = k_u
        self.i_sd_nom = psi_s_nom / (par.L_M + par.L_sgm)
        if k_fw == 0:
            self.k_fw = 2 * par.R_R / (w_s_nom * par.L_sgm**2)
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

        def q_axis_current_limit(i_sd_ref, psi_R) -> float:
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
            Realized (limited) stator voltage reference (V).
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
        Current control integral-action bandwidth (rad/s), defaults to `alpha_c`.
    alpha_o : float, optional
        Speed estimation bandwidth (rad/s), defaults to 2*pi*100.
    k_o : Callable[[float], complex], optional
        Observer gain as a function of the rotor angular speed.
    w_s_nom : float, optional
        Nominal stator angular frequency (rad/s), defaults to 2*pi*50.
    k_u : float, optional
        Voltage utilization factor, defaults to 0.95.
    k_fw : float, optional
        Field-weakening gain (1/H), defaults to `2*R_R/(w_s_nom*L_sgm**2)`.

    """

    psi_s_nom: float
    i_s_max: float
    alpha_c = 2 * pi * 200
    alpha_i: float | None = None
    alpha_o: float = 2 * pi * 100
    k_o: Callable[[float], complex] | None = None
    w_s_nom: float = 2 * pi * 50
    k_u: float = 0.95
    k_fw: float = 0.0


class CurrentVectorController:
    """
    Current-vector controller for induction machine drives.

    Parameters
    ----------
    par : InductionMachineInvGammaPars
        Machine model parameters.
    cfg : CurrentVectorControllerCfg
        Current-vector controller configuration.
    sensorless : bool, optional
        If True, sensorless control is used, defaults to True.
    T_s : float, optional
        Sampling period (s), defaults to 125e-6.

    """

    def __init__(
        self,
        par: InductionMachineInvGammaPars,
        cfg: CurrentVectorControllerCfg,
        sensorless: bool = True,
        T_s: float = 125e-6,
    ) -> None:
        self.pwm = PWM()
        self.reference_gen = CurrentReferenceGenerator(
            par, cfg.psi_s_nom, cfg.i_s_max, cfg.w_s_nom, cfg.k_u, cfg.k_fw
        )
        self.current_ctrl = CurrentController(par, cfg.alpha_c, cfg.alpha_i)
        if sensorless:
            self.observer = create_sensorless_observer(par, cfg.alpha_o, cfg.k_o)
        else:
            self.observer = create_sensored_observer(par, cfg.k_o)
        self.sensorless = sensorless
        self.T_s = T_s

    def get_feedback(self, meas: Measurements) -> ObserverOutputs:
        """Get the feedback signals."""
        u_s_ab = self.pwm.get_realized_voltage()
        if self.sensorless:
            fbk = self.observer.compute_output(meas, u_s_ab)
        else:
            fbk = self.observer.compute_output(meas, u_s_ab, meas.w_M)
        fbk.u_dc = meas.u_dc
        return fbk

    def compute_output(self, tau_M_ref: float, fbk: ObserverOutputs) -> References:
        """Compute references."""
        ref = References(T_s=self.T_s, tau_M=tau_M_ref)
        ref.i_s, ref.tau_M = self.reference_gen.compute_output(ref.tau_M, fbk.psi_R)
        ref.u_s = self.current_ctrl.compute_output(ref.i_s, fbk.i_s)
        u_s_ref_ab = exp(1j * fbk.theta_s) * ref.u_s
        ref.d_abc = self.pwm(ref.T_s, u_s_ref_ab, fbk.u_dc, fbk.w_s)
        return ref

    def update(self, ref: References, fbk: ObserverOutputs) -> None:
        """Update states."""
        self.observer.update(ref.T_s)
        self.reference_gen.update(ref.T_s, ref.u_s, fbk.u_dc)
        self.current_ctrl.update(ref.T_s, fbk.u_s, fbk.w_s)
