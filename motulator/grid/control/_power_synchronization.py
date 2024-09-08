"""Power-synchronization control for grid-connected converters."""

from dataclasses import dataclass

import numpy as np

from motulator.common.utils import wrap
from motulator.grid.control._common import (
    CurrentLimiter, GridConverterControlSystem)


# %%
@dataclass
class PowerSynchronizationControlCfg:
    """
    Reference-feedforward power-synchronization control configuration.

    Parameters
    ----------
    nom_u : float
        Nominal grid voltage (V), line-to-neutral peak value.
    nom_w : float
        Nominal grid angular frequency (rad/s).
    max_i : float
        Maximum current (A), peak value.
    R : float, optional
        Total series resistance (Ω). The default is 0.
    R_a : float, optional
        Active resistance (Ω). The default is 0.25*nom_u/max_i.
    w_b : float, optional
        Low-pass filter bandwidth (rad/s). The default is 2*pi*5.
    T_s : float, optional
        Sampling period (s). The default is 100e-6.

    """
    nom_u: float
    nom_w: float
    max_i: float
    R: float = 0
    R_a: float = None
    w_b: float = 2*np.pi*5
    T_s: float = 100e-6

    def __post_init__(self):
        if self.R_a is None:
            self.R_a = .25*self.nom_u/self.max_i
        self.k_p_psc = self.nom_w*self.R_a/(1.5*self.nom_u**2)


# %%
class PowerSynchronizationControl(GridConverterControlSystem):
    """
    Reference-feedforward power-synchronization control.

    This implements the reference-feedforward power-synchronization control
    method [#Har2020]_.

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
        p_loss = 1.5*self.cfg.R*np.abs(fbk.i_c)**2
        fbk.p_g = 1.5*np.real(fbk.u_c*np.conj(fbk.i_c)) - p_loss
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
        fbk.w_c = cfg.nom_w + cfg.k_p_psc*(ref.p_g - fbk.p_g)

        # Optionally, use of reference feedforward for d-axis current
        ref.i_c = ref.p_g/(1.5*ref.v_c) + 1j*fbk.i_c_flt.imag
        # ref.i_c = fbk.i_c_flt  # Conventional PSC

        # Limit the current reference
        ref.i_c = self.current_limiter(ref.i_c)

        # Calculation of converter voltage output reference
        ref.u_c = ref.v_c + cfg.R_a*(ref.i_c - fbk.i_c) + cfg.R*fbk.i_c
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
