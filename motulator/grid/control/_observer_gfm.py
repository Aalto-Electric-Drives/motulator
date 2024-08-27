"""Disturbance-observer-based grid-forming control for grid converters."""

from dataclasses import dataclass

import numpy as np

from motulator.common.utils import wrap
from motulator.grid.control._common import GridConverterControlSystem


# %%
@dataclass
class ObserverBasedGFMControlCfg:
    """
    Disturbance-observer-based grid-forming control configuration.

    Parameters
    ----------
    L : float
        Total inductance.
    nom_u : float
        Nominal grid voltage (V), line-to-neutral peak value.
    nom_w : float
        Nominal grid frequency (rad/s).
    max_i : float
        Maximum current (A), peak value.
    R : float, optional
        Total series resistance (Ω). The default is 0.
    R_a : float, optional
        Active resistance (Ω). The default is 0.25*num_u/max_i.
    T_s : float, optional
        Sampling period of the controller (s). The default is 100e-6.
    alpha_c : float, optional
        Current control bandwidth (rad/s). The default is 2*pi*400.
    alpha_o : float, optional
        Observer gain (rad/s). The default is 2*pi*50.
    C_dc : float, optional
        DC-bus capacitance (F). The default is None.

    """
    L: float
    nom_u: float
    nom_w: float
    max_i: float
    R: float = 0
    R_a: float = None
    T_s: float = 100e-6
    alpha_c: float = 2*np.pi*400
    alpha_o: float = 2*np.pi*50
    C_dc: float = None

    def __post_init__(self):
        if self.R_a is None:
            self.R_a = .25*self.nom_u/self.max_i
        self.k_c = self.alpha_c*self.L  # Current control gain


# %%
class ObserverBasedGFMControl(GridConverterControlSystem):
    """
    Disturbance-observer-based grid-forming control for grid converters.
    
    This implements the RFPSC-type grid-forming mode of the control method 
    described in [#Nur2024]_. Transparent current control is also implemented.

    Parameters
    ----------
    cfg : ObserverBasedGFMControlCfg
        Controller configuration parameters.

    Notes
    -----
    In this implementation, the control system operates in synchronous 
    coordinates rotating at the nominal grid angular frequency, which is worth
    noticing when plotting the results. For other implementation options, see
    [#Nur2024]_. 
    
    References
    ----------
    .. [#Nur2024] Nurminen, Mourouvin, Hinkkanen, Kukkola, "Multifunctional
        grid-forming converter control based on a disturbance observer, "IEEE
        Trans. Power Electron., 2024, https://doi.org/10.1109/TPEL.2024.3433503

    """

    def __init__(self, cfg):
        super().__init__(cfg.C_dc, cfg.T_s)
        self.cfg = cfg
        self.observer = DisturbanceObserver(
            cfg.nom_w, cfg.L, cfg.alpha_o, cfg.nom_u)
        self.ref.q_g = 0

    def get_feedback_signals(self, mdl):
        """Get the feedback signals."""
        fbk = super().get_feedback_signals(mdl)
        fbk.theta_c = self.observer.theta_c
        # Transform the measured values into synchronous coordinates
        fbk.i_c = np.exp(-1j*fbk.theta_c)*fbk.i_cs
        # Get estimates from the observer
        fbk = self.observer.output(fbk)
        return fbk

    def output(self, fbk):
        """Extend the base class method."""
        cfg = self.cfg

        # Get the reference signals
        ref = super().output(fbk)
        ref = super().get_power_reference(fbk, ref)
        ref.v_c = self.ref.v_c(ref.t) if callable(
            self.ref.v_c) else self.ref.v_c

        # Complex gains for grid-forming mode
        abs_k_p = cfg.R_a/(1.5*ref.v_c)
        abs_v_c = np.abs(fbk.v_c)
        k_p = abs_k_p*fbk.v_c/abs_v_c if abs_v_c > 0 else 0
        k_v = (1 - 1j)*fbk.v_c/abs_v_c if abs_v_c > 0 else 0

        # Feedback correction for grid-forming mode
        fbk.e_c = k_p*(ref.p_g - fbk.p_g) + k_v*(ref.v_c - abs_v_c)

        # Current limitation
        ref.i_c = fbk.i_c + fbk.e_c/cfg.k_c
        if np.abs(ref.i_c) > cfg.max_i:
            ref.i_c_lim = ref.i_c/np.abs(ref.i_c)*cfg.max_i
            fbk.e_c = cfg.k_c*(ref.i_c_lim - fbk.i_c)

        # Voltage reference
        ref.u_c = fbk.e_c + fbk.v_c + cfg.R*fbk.i_c
        ref.u_cs = np.exp(1j*fbk.theta_c)*ref.u_c

        # Duty ratios for PWM
        ref.d_abc = self.pwm(ref.T_s, ref.u_cs, fbk.u_dc, cfg.nom_w)

        return ref

    def update(self, fbk, ref):
        """Extend the base class method."""
        super().update(fbk, ref)
        self.observer.update(fbk, ref)


class DisturbanceObserver:
    """
    Disturbance observer.
    
    This implements a disturbance observer, which estimates the quasi-static
    converter output voltage. Coordinates rotating at the nominal grid angular 
    frequency are used.

    Parameters
    ----------
    w_g : float
        Nominal grid angular frequency (rad/s).
    L : float
        Estimate of total inductance (H) between converter and grid.
    alpha_o : float
        Observer gain (rad/s).
    v_c0 : float
        Initial value of converter voltage state (V).
      
    """

    def __init__(self, w_g, L, alpha_o, v_c0):
        self.w_g = w_g
        self.w_c = w_g
        self.L = L
        self.k_o = alpha_o - 1j*w_g
        # Initial states
        self.v_cp = v_c0
        self.theta_c = 0

    def output(self, fbk):
        """Compute the estimates."""
        # Quasi-static converter voltage
        fbk.v_c = self.v_cp - self.k_o*self.L*fbk.i_c
        # Grid voltage
        fbk.u_g = fbk.v_c - 1j*self.w_g*self.L*fbk.i_c
        # Active and reactive power
        fbk.p_g = 1.5*np.real(fbk.v_c*np.conj(fbk.i_c))
        fbk.q_g = 1.5*np.imag(fbk.u_g*np.conj(fbk.i_c))
        return fbk

    def update(self, fbk, ref):
        """Update the observer states."""
        self.v_cp += ref.T_s*(self.k_o + 1j*self.w_c)*fbk.e_c + 1j*(
            self.w_g - self.w_c)*self.v_cp
        self.theta_c += ref.T_s*self.w_c
        self.theta_c = wrap(self.theta_c)
