"""V/Hz control for induction motor drives."""

from dataclasses import dataclass, field, InitVar
from types import SimpleNamespace
from typing import Literal

import numpy as np

from motulator.drive.control import DriveControlSystem
from motulator.drive.utils import InductionMachineInvGammaPars
from motulator.common.control import PWM, RateLimiter
from motulator.common.utils import wrap


# %%
@dataclass
class VHzControlCfg:
    """V/Hz control configuration."""

    par: InductionMachineInvGammaPars
    nom_psi_s: float = None
    T_s: float = 250e-6
    overmodulation: Literal["MME", "MPE", "six-step"] = "MME"
    rate_limit: float = 2*np.pi*120
    gain: SimpleNamespace = field(init=False)
    k_u: InitVar[float] = 1
    k_w: InitVar[float] = 4
    alpha_f: InitVar[float] = None
    alpha_i: InitVar[float] = None

    def __post_init__(self, k_u, k_w, alpha_f, alpha_i):
        par = self.par
        w_rb = par.R_R*(par.L_M + par.L_sgm)/(par.L_sgm*par.L_M)
        alpha_f = 0.1*w_rb if alpha_f is None else alpha_f
        alpha_i = 0.1*w_rb if alpha_i is None else alpha_f
        self.gain = SimpleNamespace(
            k_u=k_u, k_w=k_w, alpha_f=alpha_f, alpha_i=alpha_i)


# %%
class VHzControl(DriveControlSystem):
    """
    V/Hz control with the stator current feedback.

    The method is similar to [#Hin2022]_. Open-loop V/Hz control can be
    obtained as a special case by choosing::

        R_s, R_R = 0, 0
        k_u, k_w = 0, 0

    References
    ----------
    .. [#Hin2022] Hinkkanen, Tiitinen, Mölsä, Harnefors, "On the stability of
       volts-per-hertz control for induction motors," IEEE J. Emerg. Sel.
       Topics Power Electron., 2022,
       https://doi.org/10.1109/JESTPE.2021.3060583

    """

    def __init__(self, cfg):
        super().__init__(cfg.par, cfg.T_s, sensorless=True)
        self.gain = cfg.gain
        self.nom_psi_s = cfg.nom_psi_s
        self.pwm = PWM(overmodulation=cfg.overmodulation)
        self.rate_limiter = RateLimiter(cfg.rate_limit)
        # Initialize the states
        self.ref.i_s, self.ref.w_r, self.theta_s = 0j, 0, 0

    def get_feedback_signals(self, mdl):
        """Get the feedback signals."""
        fbk = super().get_feedback_signals(mdl)
        fbk.theta_s = self.theta_s
        fbk.i_s = np.exp(-1j*fbk.theta_s)*fbk.i_ss
        fbk.u_s = np.exp(-1j*fbk.theta_s)*fbk.u_ss

        return fbk

    def output(self, fbk):
        """Extend the base class method."""
        # Unpack
        par, gain = self.par, self.gain
        par.alpha = par.R_R/par.L_M

        # Define the stator frequency computation
        def stator_frequency(fbk, ref):
            # Operating-point quantities
            ref.psi_R = ref.psi_s - par.L_sgm*ref.i_s
            ref_psi_R_sqr = np.abs(ref.psi_R)**2
            # Compute the slip frequency and the dynamic stator frequency
            if ref_psi_R_sqr > 0:
                fbk.w_r = (
                    par.R_R*np.imag(fbk.i_s*np.conj(ref.psi_R))/ref_psi_R_sqr)
                fbk.w_s = ref.w_s + gain.k_w*(ref.w_r - fbk.w_r)
            else:
                fbk.w_s, fbk.w_r = 0, 0

            return fbk, ref

        # Define the voltage reference computation
        def voltage_reference(fbk, ref):
            # Nominal magnetizing current
            nom_i_sd = ref.psi_s/(par.L_M + par.L_sgm)
            # Operating-point current for RI compensation
            ref_i_s0 = nom_i_sd + 1j*ref.i_s.imag
            # Term -R_s omitted to avoid problems due to the voltage saturation
            # k = -R_s + k_u*L_sgm*(alpha + 1j*w_m0)
            k = gain.k_u*par.L_sgm*(par.alpha + 1j*fbk.w_s)
            ref.u_s = (
                par.R_s*ref_i_s0 + 1j*fbk.w_s*ref.psi_s + k*
                (ref.i_s - fbk.i_s))
            ref.u_ss = ref.u_s*np.exp(1j*fbk.theta_s)

            ref.d_abc = self.pwm(ref.T_s, ref.u_ss, fbk.u_dc, fbk.w_s)

            return ref

        # Get the reference signals
        ref = super().output(fbk)
        ref.w_m = self.rate_limiter(ref.T_s, self.ref.w_m(ref.t))
        ref.i_s, ref.w_r = self.ref.i_s, self.ref.w_r
        ref.psi_s = self.nom_psi_s
        ref.w_s = ref.w_m + ref.w_r  # Slip compensation

        # Compute the dynamic stator frequency and the voltage reference
        fbk, ref = stator_frequency(fbk, ref)
        ref = voltage_reference(fbk, ref)

        return ref

    def update(self, fbk, ref):
        """Extend the base class method."""
        super().update(fbk, ref)
        self.ref.i_s += ref.T_s*self.gain.alpha_i*(fbk.i_s - self.ref.i_s)
        self.ref.w_r += ref.T_s*self.gain.alpha_f*(fbk.w_r - self.ref.w_r)
        self.theta_s = wrap(self.theta_s + ref.T_s*fbk.w_s)
