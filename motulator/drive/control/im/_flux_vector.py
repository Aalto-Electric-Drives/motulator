"""Flux-vector control of synchronous machine drives."""

from dataclasses import dataclass

import numpy as np

from motulator.drive.control import DriveControlSystem, SpeedController
from motulator.drive.control.im._common import Observer, ObserverCfg


@dataclass
class FluxVectorControlCfg:
    """
    Flux-vector control configuration.

    Parameters
    ----------
    nom_psi_s : float
        Nominal stator flux linkage (Vs).
    max_i_s : float
        Maximum stator current (A).
    max_tau_M : float
        Maximum torque reference (Nm).
    k_u : float, optional
        Voltage utilization factor. The default is 0.95.

    """

    nom_psi_s: float
    max_i_s: float
    max_tau_M: float
    k_u: float = 0.95


# %%
class FluxVectorControl(DriveControlSystem):
    """
    Flux-vector control of asynchronous machine drives.

    Parameters
    ----------
    par : InductionMachineInvGammaPars
        Machine model parameters.

    alpha_psi : float, optional
        Flux-control bandwidth (rad/s). The default is 2*pi*100.
    alpha_tau : float, optional
        Torque-control bandwidth (rad/s). The default is 2*pi*200.
    alpha_c : float, optional
        Internal current-control bandwidth (rad/s). The default is 2*pi*200.
    alpha_o : float, optional
        Observer bandwidth (rad/s). The default is 2*pi*40.
    J : float, optional
        Moment of inertia (kgm²). Needed only for the speed controller.
    T_s : float
        Sampling period (s). The default is 250e-6.
    sensorless : bool, optional
        If True, sensorless control is used. The default is True.

    """

    def __init__(
        self,
        par,
        cfg: FluxVectorControlCfg,
        alpha_psi=2*np.pi*100,
        alpha_tau=2*np.pi*200,
        alpha_c=2*np.pi*200,
        alpha_o=2*np.pi*40,
        J=None,
        T_s=250e-6,
        sensorless=True,
    ):
        super().__init__(par, T_s, sensorless)
        self.cfg = cfg
        if J is not None:
            self.speed_ctrl = SpeedController(J, 2*np.pi*4)
        else:
            self.speed_ctrl = None
        self.observer = Observer(ObserverCfg(par, T_s, sensorless, alpha_o))
        self.alpha_psi = alpha_psi
        self.alpha_tau = alpha_tau
        self.alpha_c = alpha_c

    def get_flux_reference(self, fbk):
        """Simple field-weakening strategy."""
        # The flux magnitude is reduced inversely proportional to the angular
        # frequency beyond the nominal.
        max_u_s = self.cfg.k_u*fbk.u_dc/np.sqrt(3)
        max_psi_s = max_u_s/np.abs(fbk.w_s) if fbk.w_s != 0 else np.inf
        return np.min([max_psi_s, self.cfg.nom_psi_s])

    def output(self, fbk):
        """Calculate references."""
        par, cfg = self.par, self.cfg

        # Get the references from the outer loop
        ref = super().output(fbk)
        ref = super().get_torque_reference(fbk, ref)
        ref.psi_s = self.get_flux_reference(fbk)

        # Limit the torque reference
        ref.tau_M = np.clip(ref.tau_M, -cfg.max_tau_M, cfg.max_tau_M)

        # Torque estimate
        tau_M = 1.5*par.n_p*np.imag(fbk.i_s*np.conj(fbk.psi_s))

        # Torque-production factor, c_tau = 0 corresponds to the MTPV condition
        c_tau = 1.5*par.n_p*np.real(fbk.psi_R*np.conj(fbk.psi_s))

        # References for the flux and torque controllers
        e_psi = self.alpha_psi*(ref.psi_s - np.abs(fbk.psi_s))
        e_tau = self.alpha_tau*(ref.tau_M - tau_M)
        if c_tau > 0:
            e_s = (
                1.5*par.n_p*np.abs(fbk.psi_s)*fbk.psi_R*e_psi +
                1j*fbk.psi_s*par.L_sgm*e_tau)/c_tau
        else:
            e_s = e_psi

        # Internal current reference for state feedback
        ref.i_s = fbk.i_s + e_s/(self.alpha_c*self.par.L_sgm)
        # Limit the current reference
        if np.abs(ref.i_s) > cfg.max_i_s:
            ref.i_s = cfg.max_i_s*ref.i_s/np.abs(ref.i_s)
        e_sp = self.alpha_c*self.par.L_sgm*(ref.i_s - fbk.i_s)

        # Stator voltage reference
        ref.u_s = par.R_s*fbk.i_s + 1j*(fbk.w_m + fbk.w_r)*fbk.psi_s + e_sp
        u_ss = ref.u_s*np.exp(1j*fbk.theta_s)
        ref.d_abc = self.pwm(ref.T_s, u_ss, fbk.u_dc, fbk.w_s)

        return ref
