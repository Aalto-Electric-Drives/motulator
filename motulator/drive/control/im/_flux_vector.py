"""Flux-vector control of synchronous machine drives."""

from dataclasses import dataclass, InitVar

import numpy as np

from motulator.drive.control import DriveControlSystem, SpeedController
from motulator.drive.control.im._common import Observer, ObserverCfg

class FluxVectorControlCfg:
    """
    Controller configuration.

    Parameters
    ----------
    nom_u_s : float
        Nominal voltage
    nom_w_s : float
        Nominal speed
    tau_max : float
        Maximum torque reference
    k_u : float, optional
        Voltage utilization factor. The default is 0.95.

    """
    def __init__(self, nom_u_s, nom_w_s, nom_tau_M, k_u = 0.95) -> None:
        self.nom_u_s = nom_u_s
        self.nom_w_s = nom_w_s
        self.tau_max = nom_tau_M * 1.5
        self.nom_psi_s = nom_u_s/nom_w_s
        self.k_u = k_u

# %%
class FluxVectorControl(DriveControlSystem):
    """
    Flux-vector control of asynchronous machine drives.

    Parameters
    ----------
    par : InductionMachineInvGammaPars
        Machine model parameters.

    alpha_psi : float, optional
        Bandwidth of the flux controller (rad/s). The default is 2*pi*100.
    alpha_tau : float, optional
        Bandwidth of the torque controller (rad/s). The default is 2*pi*200.
    alpha_o : float, optional
        Observer bandwidth (rad/s). The default is 2*pi*40.
    J : float, optional
        Moment of inertia (kgm²). Needed only for the speed controller. 
    T_s : float
        Sampling period (s). The default is 250e-6.
    sensorless : bool, optional
        If True, sensorless control is used. The default is True.

    References
    ----------

    """

    def __init__(
            self,
            par,
            cfg: FluxVectorControlCfg,
            alpha_psi=2*np.pi*100,
            alpha_tau=2*np.pi*200,
            alpha_o=2*np.pi*40,
            J=None,
            T_s=250e-6,
            sensorless=True):
        super().__init__(par, T_s, sensorless)
        self.cfg = cfg

        if J is not None:
            self.speed_ctrl = SpeedController(J, 2*np.pi*4)
        else:
            self.speed_ctrl = None
        self.observer = Observer(
            ObserverCfg(par, T_s, sensorless, alpha_o))
        # Bandwidths
        self.alpha_psi = alpha_psi
        self.alpha_tau = alpha_tau
        self.k = 0

    def get_flux_reference(self, fbk):
        """Simple field-weakening with flux-magnitude 
        reduced inversely proportional to the speed at speeds beyond the nominal."""

        max_u_s = self.cfg.k_u*fbk.u_dc/np.sqrt(3)
        max_psi_s = max_u_s/np.abs(fbk.w_s) if fbk.w_s != 0 else np.inf
        return np.min([max_psi_s, self.cfg.nom_psi_s])


    def output(self, fbk):
        """Calculate references."""
        par = self.par

        # Get the references from the outer loop
        ref = super().output(fbk)
        ref = super().get_torque_reference(fbk, ref)

        # Compute flux and torque references
        ref.psi_s = self.get_flux_reference(fbk)
        ref.tau_M = np.clip(ref.tau_M, -self.cfg.tau_max, self.cfg.tau_max)

        # Torque estimates
        tau_M = 1.5*par.n_p*np.imag(fbk.i_s*np.conj(fbk.psi_s))

        # Torque-production factor, c_tau = 0 corresponds to the MTPV condition
        c_tau = 1.5*par.n_p*np.real(fbk.psi_R*np.conj(fbk.psi_s))

        # References for the flux and torque controllers
        v_psi = self.alpha_psi*(ref.psi_s - np.abs(fbk.psi_s))
        v_tau = self.alpha_tau*(ref.tau_M - tau_M)
        if c_tau > 0:
            v = (
                1.5*par.n_p*np.abs(fbk.psi_s)*fbk.psi_R*v_psi +
                1j*fbk.psi_s*par.L_sgm*v_tau)/c_tau
        else:
            v = v_psi

        # Stator voltage reference
        ref.u_s = par.R_s*fbk.i_s + 1j*(fbk.w_m + fbk.w_r)*fbk.psi_s + v
        u_ss = ref.u_s*np.exp(1j*fbk.theta_s)

        ref.d_abc = self.pwm(ref.T_s, u_ss, fbk.u_dc, fbk.w_s)

        return ref
    
