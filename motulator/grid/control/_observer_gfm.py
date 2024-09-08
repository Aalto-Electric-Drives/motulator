"""Disturbance-observer-based grid-forming control."""

from dataclasses import dataclass

import numpy as np

from motulator.common.utils import wrap
from motulator.grid.control._common import (
    GridConverterControlSystem, CurrentLimiter)


# %%
@dataclass
class ObserverBasedGridFormingControlCfg:
    """
    Disturbance-observer-based grid-forming control configuration.

    Parameters
    ----------
    L : float
        Total inductance.
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
    k_v : float, optional
        Voltage control gain. The default is `alpha_o/nom_w`.
    alpha_c : float, optional
        Current control bandwidth (rad/s). The default is 2*pi*400.
    alpha_o : float, optional
        Observer gain (rad/s). The default is 2*pi*50.
    T_s : float, optional
        Sampling period (s). The default is 100e-6.

    """
    L: float
    nom_u: float
    nom_w: float
    max_i: float
    R: float = 0
    R_a: float = None
    k_v: float = None
    alpha_c: float = 2*np.pi*400
    alpha_o: float = 2*np.pi*50
    T_s: float = 100e-6

    def __post_init__(self):
        if self.R_a is None:
            self.R_a = .25*self.nom_u/self.max_i
        if self.k_v is None:
            self.k_v = self.alpha_o/self.nom_w
        self.k_c = self.alpha_c*self.L  # Current control gain


# %%
class ObserverBasedGridFormingControl(GridConverterControlSystem):
    """
    Disturbance-observer-based grid-forming control.

    This implements the RFPSC-type grid-forming mode of the control method
    described in [#Nur2024]_. Transparent current control is also implemented.

    Parameters
    ----------
    cfg : ObserverBasedGridFormingControlCfg
        Control system configuration parameters.

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
        super().__init__(cfg.T_s)
        self.cfg = cfg
        self.observer = DisturbanceObserver(cfg)
        self.current_limiter = CurrentLimiter(cfg.max_i)

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
        exp_j_theta = fbk.v_c/np.abs(fbk.v_c) if np.abs(fbk.v_c) > 0 else 1
        k_p = cfg.R_a/(1.5*ref.v_c)*exp_j_theta
        k_v = (1 - 1j*cfg.k_v)*exp_j_theta

        # Feedback correction for grid-forming mode
        fbk.e_c = k_p*(ref.p_g - fbk.p_g) + k_v*(ref.v_c - np.abs(fbk.v_c))

        # Transparent current limitation
        ref.i_c = fbk.i_c + fbk.e_c/cfg.k_c
        ref.i_c = self.current_limiter(ref.i_c)
        fbk.e_c = cfg.k_c*(ref.i_c - fbk.i_c)

        # Voltage reference (with optional resistive voltage drop compensation)
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
    cfg : ObserverBasedGridFormingControlCfg
        Control system configuration parameters.

    """

    def __init__(self, cfg):
        self.alpha_o = cfg.alpha_o
        self.L = cfg.L
        self.w_g = cfg.nom_w
        self.u_gp = cfg.nom_u
        self.theta_c = 0

    def output(self, fbk):
        """Compute the estimates."""
        # Estimates for the quasi-static converter voltage and the grid voltage
        fbk.v_c = self.u_gp - (self.alpha_o - 1j*self.w_g)*self.L*fbk.i_c
        fbk.u_g = self.u_gp - self.alpha_o*self.L*fbk.i_c
        # Active and reactive powers
        s_g = 1.5*fbk.u_g*np.conj(fbk.i_c)
        fbk.p_g, fbk.q_g = s_g.real, s_g.imag
        return fbk

    def update(self, fbk, ref):
        """Update the states."""
        self.u_gp += ref.T_s*self.alpha_o*fbk.e_c
        self.theta_c += ref.T_s*self.w_g
        self.theta_c = wrap(self.theta_c)
