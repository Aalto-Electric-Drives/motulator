"""
V/Hz control for induction motor drives.

The method is similar to [#Hin2022]_. Open-loop V/Hz control can be obtained as 
a special case by choosing::

    R_s, R_R = 0, 0
    k_u, k_w = 0, 0

References
----------
.. [#Hin2022] Hinkkanen, Tiitinen, Mölsä, Harnefors, "On the stability of
   volts-per-hertz control for induction motors," IEEE J. Emerg. Sel. Topics
   Power Electron., 2022, https://doi.org/10.1109/JESTPE.2021.3060583

"""
# %%
from dataclasses import dataclass, field, InitVar
from types import SimpleNamespace
import numpy as np
from motulator.control._common import PWM, DriveCtrl, RateLimiter
from motulator.control.im._common import ModelPars
from motulator._helpers import abc2complex, wrap


# %%
@dataclass
class VHzCtrlPars:
    """V/Hz control parameters."""

    model_par: ModelPars
    nom_psi_s: float = None
    T_s: float = 250e-6
    six_step: bool = False
    rate_limiter: callable = RateLimiter(2*np.pi*120)
    gain: SimpleNamespace = field(init=False)
    k_u: InitVar[float] = 1
    k_w: InitVar[float] = 4
    alpha_f: InitVar[float] = None
    alpha_i: InitVar[float] = None

    def __post_init__(self, k_u, k_w, alpha_f, alpha_i):
        par = self.model_par
        w_rb = par.R_R*(par.L_M + par.L_sgm)/(par.L_sgm*par.L_M)
        alpha_f = .1*w_rb if alpha_f is None else alpha_f
        alpha_i = .1*w_rb if alpha_i is None else alpha_f
        self.gain = SimpleNamespace(
            k_u=k_u, k_w=k_w, alpha_f=alpha_f, alpha_i=alpha_i)


# %%
class VHzCtrl(DriveCtrl):
    """
    V/Hz control with the stator current feedback.

    Parameters
    ----------
    par : ModelPars
        Control parameters.

    """
    def __init__(self, vhz_par):
        super().__init__(vhz_par.model_par, vhz_par.T_s, sensorless=True)
        self.gain = vhz_par.gain
        self.nom_psi_s = vhz_par.nom_psi_s
        self.pwm = PWM(six_step=vhz_par.six_step)
        self.rate_limiter = vhz_par.rate_limiter
        self.ref = SimpleNamespace()
        self.state = SimpleNamespace(theta_s=0, i_s=0j, w_r=0)

    def get_feedback_signals(self, mdl):
        """Get the feedback signals."""
        fbk = super().get_feedback_signals(mdl)
        fbk.theta_s = self.state.theta_s
        fbk.i_s = np.exp(-1j*fbk.theta_s)*fbk.i_ss
        fbk.u_s = np.exp(-1j*fbk.theta_s)*fbk.u_ss

        return fbk

    def output(self, fbk):
        """Extend the base class method."""
        ref = super().output(fbk)

        ref.i_s, ref.w_r = self.state.i_s, self.state.w_r
        ref.psi_s = self.nom_psi_s

        # Rate limit the frequency reference
        ref.w_m = self.rate_limiter(ref.T_s, self.ref.w_m(ref.t))

        # Slip compensation
        ref.w_s = ref.w_m + ref.w_r

        # Dynamic stator frequency and slip frequency
        fbk, ref = self._stator_freq(fbk, ref)

        # Voltage reference
        ref = self._voltage_reference(fbk, ref)

        return ref

    def _stator_freq(self, fbk, ref):
        """Compute the stator frequency for coordinate transformations."""
        par = self.par

        # Operating-point quantities
        ref.psi_R = ref.psi_s - par.L_sgm*ref.i_s
        ref_psi_R_sqr = np.abs(ref.psi_R)**2

        # Compute the dynamic stator frequency
        if ref_psi_R_sqr > 0:
            # Slip estimate based on the measured current
            fbk.w_r = par.R_R*np.imag(fbk.i_s*np.conj(ref.psi_R))/ref_psi_R_sqr
            # Dynamic frequency
            fbk.w_s = ref.w_s + self.gain.k_w*(ref.w_r - fbk.w_r)
        else:
            fbk.w_s, fbk.w_r = 0, 0

        return fbk, ref

    def _voltage_reference(self, fbk, ref):
        """Compute the stator voltage reference."""
        par, gain = self.par, self.gain

        # Nominal magnetizing current
        nom_i_sd = ref.psi_s/(par.L_M + par.L_sgm)

        # Operating-point current for RI compensation
        ref_i_s0 = nom_i_sd + 1j*ref.i_s.imag
        # Term -R_s omitted to avoid problems due to the voltage saturation
        # k = -R_s + k_u*L_sgm*(alpha + 1j*w_m0)
        k = gain.k_u*par.L_sgm*(par.R_R/par.L_M + 1j*fbk.w_s)

        ref.u_s = (
            par.R_s*ref_i_s0 + 1j*fbk.w_s*ref.psi_s + k*(ref.i_s - fbk.i_s))
        ref.u_ss = ref.u_s*np.exp(1j*fbk.theta_s)

        ref.d_abc = self.pwm(ref.T_s, ref.u_ss, fbk.u_dc, fbk.w_s)

        return ref

    def update(self, fbk, ref):
        """Extend the base class method."""
        super().update(fbk, ref)

        self.state.i_s += ref.T_s*self.gain.alpha_i*(fbk.i_s - self.state.i_s)
        self.state.w_r += ref.T_s*self.gain.alpha_f*(fbk.w_r - self.state.w_r)
        self.state.theta_s = wrap(self.state.theta_s + ref.T_s*fbk.w_s)
