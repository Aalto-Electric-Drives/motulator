"""Observer-based V/Hz control of synchronous motor drives."""

from types import SimpleNamespace
from dataclasses import dataclass, InitVar

import numpy as np

from motulator.drive.control import DriveCtrl
from motulator.common.control import RateLimiter
from motulator.drive.control.sm._flux_vector import (
    FluxTorqueReference, FluxTorqueReferenceCfg)
from motulator.common.utils import wrap


# %%
@dataclass
class ObserverBasedVHzCtrlCfg(FluxTorqueReferenceCfg):
    """
    Control system configuration.

    Parameters
    ----------
    alpha_psi : float, optional
        Flux control bandwidth (rad/s). The default is 2*pi*50.
    alpha_tau : float
        Torque control bandwidth (rad/s). The default is 2*pi*50.
    alpha_f : float, optional
        Bandwidth of the high-pass filter (rad/s). The default is 2*pi*1.

    """
    alpha_psi: float = 2*np.pi*50
    alpha_tau: InitVar[float] = 2*np.pi*50
    alpha_f: float = 2*np.pi*1

    def __post_init__(self, par, alpha_tau):
        super().__post_init__(par)
        # Gain k_tau
        G = (par.L_d - par.L_q)/(par.L_d*par.L_q)
        psi_s0 = par.psi_f if par.psi_f > 0 else self.min_psi_s
        if par.psi_f > 0:  # PMSM or PM-SyRM
            c_delta0 = 1.5*par.n_p*(par.psi_f*psi_s0/par.L_d - G*psi_s0**2)
        else:  # SyRM
            c_delta0 = 1.5*par.n_p*G*psi_s0**2
        self.k_tau = alpha_tau/c_delta0


# %%
class ObserverBasedVHzCtrl(DriveCtrl):
    """
    Observer-based V/Hz control for synchronous motors.

    This observer-based V/Hz control control method is based on [#Tii2022]_.

    Parameters
    ----------
    par : SynchronousMachinePars
        Machine model parameters.
    cfg : ObserverBasedVHzCtrlCfg
        Control system configuration.
    T_s : float, optional
        Sampling period (s). The default is 250e-6.

    References
    ----------
    .. [#Tii2022] Tiitinen, Hinkkanen, Kukkola, Routimo, Pellegrino, Harnefors, 
       "Stable and passive observer-based V/Hz control for synchronous Motors," 
       Proc. IEEE ECCE, 2022, https://doi.org/10.1109/ECCE50734.2022.9947858

    """

    def __init__(self, par, cfg, T_s=250e-6):
        super().__init__(par, T_s, sensorless=True)
        self.par, self.cfg = par, cfg
        # Subsystems
        self.flux_torque_reference = FluxTorqueReference(cfg)
        self.observer = FluxObserver(par)
        self.rate_limiter = RateLimiter(np.inf)
        # Initialize the states
        self.ref.tau_M, self.theta_s = 0, 0

    def output(self, fbk):
        """Output."""
        ref = super().output(fbk)

        # Unpack
        par, cfg = self.par, self.cfg

        # Get the reference signals
        ref.w_m = self.rate_limiter(ref.T_s, self.ref.w_m(ref.t))
        ref.tau_M = self.ref.tau_M

        # Coordinate transformations
        fbk.theta_s = self.theta_s
        fbk.i_s = np.exp(-1j*fbk.theta_s)*fbk.i_ss
        fbk.u_s = np.exp(-1j*fbk.theta_s)*fbk.u_ss

        # Limited flux references
        ref = self.flux_torque_reference(fbk, ref)

        # Electromagnetic torque
        fbk.tau_M = 1.5*par.n_p*np.imag(fbk.i_s*np.conj(fbk.psi_s))

        # Dynamic frequency
        fbk.w_s = ref.w_m - cfg.k_tau*(fbk.tau_M - ref.tau_M)

        # Voltage reference
        err = ref.psi_s - fbk.psi_s
        ref.u_s = par.R_s*fbk.i_s + 1j*fbk.w_s*ref.psi_s + cfg.alpha_psi*err
        u_ss = ref.u_s*np.exp(1j*fbk.theta_s)
        ref.d_abc = self.pwm(ref.T_s, u_ss, fbk.u_dc, fbk.w_s)
        return ref

    def update(self, fbk, ref):
        """Update the states."""
        super().update(fbk, ref)
        # Low-pass filtered torque
        self.ref.tau_M += ref.T_s*self.cfg.alpha_f*(fbk.tau_M - ref.tau_M)
        # Update the angle
        self.theta_s = wrap(fbk.theta_s + ref.T_s*fbk.w_s)


# %%
class FluxObserver:
    """
    Sensorless stator flux observer in external coordinates.

    This observer estimates the stator flux linkage and the angle of the 
    coordinate system with respect to the d-axis of the rotor. Speed-estimation 
    is omitted. The observer gain decouples the electrical and mechanical 
    dynamics and allows placing the poles of the corresponding linearized 
    estimation error dynamics. This implementation operates in external 
    coordinates (typically synchronous coordinates defined by reference signals 
    of a control system).

    Parameters
    ----------
    par : SynchronousMachinePars
        Machine model parameters.
    alpha_o : float, optional
        Observer gain (rad/s). The default is 2*pi*20.
    zeta_inf : float, optional
        Damping ratio at infinite speed. The default is 0.2.
  
    """

    def __init__(self, par, alpha_o=2*np.pi*20, zeta_inf=.2):
        self.par = par
        self.alpha_o = alpha_o
        self.b_p = .5*par.R_s*(par.L_d + par.L_q)/(par.L_d*par.L_q)
        self.zeta_inf = zeta_inf
        # Initial states
        self.est = SimpleNamespace(psi_s=par.psi_f, delta=0)

    def output(self, fbk):
        """Output."""
        fbk.psi_s, fbk.delta = self.est.psi_s, self.est.delta

        return fbk

    def update(self, T_s, fbk):
        """Update the states."""
        par = self.par

        # Transformations to rotor coordinates. This is mathematically
        # equivalent to the version in [Tii2022].
        i_sr = fbk.i_s*np.exp(1j*fbk.delta)
        psi_sr = fbk.psi_s*np.exp(1j*fbk.delta)

        # Auxiliary flux and estimation error in rotor coordinates
        psi_ar = par.psi_f + (par.L_d - par.L_q)*np.conj(i_sr)
        e_r = par.L_d*i_sr.real + 1j*par.L_q*i_sr.imag + par.psi_f - psi_sr

        # Auxiliary flux in controller coordinates
        psi_a = np.exp(-1j*fbk.delta)*psi_ar

        k = self.b_p + 2*self.zeta_inf*np.abs(fbk.w_s)

        if np.abs(psi_ar) > 0:
            # Correction voltage in controller coordinates
            v = k*psi_a*np.real(e_r/psi_ar)
            # Error signal
            w_delta = self.alpha_o*np.imag(e_r/psi_ar)
        else:
            v, w_delta = 0, 0

        # Update the states
        self.est.psi_s += T_s*(
            fbk.u_s - par.R_s*fbk.i_s - 1j*fbk.w_s*fbk.psi_s + v)
        self.est.delta += T_s*w_delta
