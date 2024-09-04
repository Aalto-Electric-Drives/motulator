"""Power synchronization control for grid-connected converters."""

from dataclasses import dataclass

import numpy as np

from motulator.common.utils import wrap
from motulator.grid.control._common import (
    CurrentLimiter, GridConverterControlSystem)


# %%
@dataclass
class RFPSCControlCfg:
    """
    Power synchronization control configuration.

    Parameters
    ----------
    nom_u : float
        Nominal grid voltage (V), line-to-neutral peak value.
    nom_w : float
        Nominal grid frequency (rad/s).
    max_i : float
        Maximum current (A), peak value.
    R_a : float, optional
        Active resistance (Ω). The default is 0.25*nom_u/max_i.
    T_s : float, optional
        Sampling period of the controller (s). The default is 100e-6.
    w_b : float, optional
        Current low-pass filter bandwidth (rad/s). The default is 2*pi*5.

    """
    nom_u: float
    nom_w: float
    max_i: float
    R_a: float = None
    T_s: float = 100e-6
    w_b: float = 2*np.pi*5

    def __post_init__(self):
        if self.R_a is None:
            self.R_a = .25*self.nom_u/self.max_i
        self.k_p_psc = self.nom_w*self.R_a/(1.5*self.nom_u**2)


# %%
class RFPSCControl(GridConverterControlSystem):
    """
    Reference-feedforward power synchronization control for grid converters.
    
    This implements the reference-feedforward power synchronization control 
    (RFPSC) method [#Har2020]_. 

    Parameters
    ----------
    cfg : PSCControlCfg
        Model and controller configuration parameters.
    
    References
    ----------
    .. [#Har2020] Harnefors, Rahman, Hinkkanen, Routimo, "Reference-feedforward
        power-synchronization control," IEEE Trans. Power Electron., 2020,
        https://doi.org/10.1109/TPEL.2020.2970991

    """

    def __init__(self, cfg):
        super().__init__(cfg.T_s)
        self.cfg = cfg
        self.current_limiter = CurrentLimiter(cfg.max_i)
        self.ref.q_g = 0
        self.theta_c = 0
        self.i_c_flt = 0j

    def get_feedback_signals(self, mdl):
        """Get the feedback signals."""
        fbk = super().get_feedback_signals(mdl)
        fbk.theta_c = self.theta_c
        fbk.i_c_flt = self.i_c_flt
        # Transform the measured values into synchronous coordinates
        fbk.i_c = np.exp(-1j*fbk.theta_c)*fbk.i_cs
        fbk.u_c = np.exp(-1j*fbk.theta_c)*fbk.u_cs
        # Active power
        fbk.p_c = 1.5*np.real(fbk.u_c*np.conj(fbk.i_c))
        return fbk

    def output(self, fbk):
        """Extend the base class method."""
        cfg = self.cfg

        # Get the reference signals
        ref = super().output(fbk)
        ref = super().get_power_reference(fbk, ref)
        ref.v_c = self.ref.v_c(ref.t) if callable(
            self.ref.v_c) else self.ref.v_c

        # Calculation of power droop
        fbk.w_c = cfg.nom_w + cfg.k_p_psc*(ref.p_g - fbk.p_c)

        # Optionally, use of reference feedforward for d-axis current
        ref.i_c = ref.p_g/(1.5*ref.v_c) + 1j*fbk.i_c_flt.imag
        # ref.i_c = fbk.i_c_flt  # Conventional PSC

        # Limit the current reference
        ref.i_c = self.current_limiter(ref.i_c)

        # Calculation of converter voltage output reference
        ref.u_c = ref.v_c + cfg.R_a*(ref.i_c - fbk.i_c)
        ref.u_cs = np.exp(1j*fbk.theta_c)*ref.u_c

        # Duty ratios for PWM
        ref.d_abc = self.pwm(ref.T_s, ref.u_cs, fbk.u_dc, fbk.w_c)

        return ref

    def update(self, fbk, ref):
        """Extend the base class method."""
        super().update(fbk, ref)
        self.i_c_flt += ref.T_s*self.cfg.w_b*(fbk.i_c - self.i_c_flt)
        self.theta_c += ref.T_s*fbk.w_c
        self.theta_c = wrap(self.theta_c)
