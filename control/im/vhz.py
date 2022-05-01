# pylint: disable=C0103
'''
This module includes V/Hz control for induction motor drives. The method
is similar to the paper "On the stability of volts-per-hertz control for
induction motors":

    https://doi.org/10.1109/JESTPE.2021.3060583

Open-loop V/Hz control can be obtained as a special case by choosing:

    R_s, R_R = 0, 0
    k_u, k_w = 0, 0

Notes
-----
The low-pass-filtered values are marked with ref at the end of the variable
name. These slowly varying quasi-steady-state quantities can be seen to
represent the operating point.

'''
# %%
import numpy as np
from sklearn.utils import Bunch
from control.common import PWM, RateLimiter, Datalogger
from helpers import abc2complex


# %%
class VHzCtrl:
    """
    V/Hz control algorithm with the stator current feedback.

    """

    def __init__(self, pars):
        """
        Instantiate the classes and get the parameters.

        """
        # Instantiate classes
        self.pwm = PWM(pars)
        self.rate_limiter = RateLimiter(pars)
        self.datalog = Datalogger()
        # Parameters
        self.T_s = pars.T_s
        self.k_u = pars.k_u
        self.k_w = pars.k_w
        self.psi_s_ref = pars.psi_s_nom
        self.R_s = pars.R_s
        self.R_R = pars.R_R
        self.L_sgm = pars.L_sgm
        self.L_M = pars.L_M
        self.alpha_f = pars.alpha_f
        self.alpha_i = pars.alpha_i
        # States
        self.i_s_ref = 0j
        self.theta_s = 0
        self.w_r_ref = 0

    def __call__(self, w_m_ref, i_s_abc, u_dc):
        """
        Main control loop.

        Parameters
        ----------
        w_m_ref : float
            Speed reference (in electrical rad/s).
        i_s_abc : ndarray, shape (3,)
            Phase currents.
        u_dc : float
            DC-bus voltage.

        Returns
        -------
        d_abc_ref : ndarray, shape (3,)
            Duty ratio references.
        T_s : float
            Sampling period.

        """
        # Space vector transformation
        i_s = np.exp(-1j*self.theta_s)*abc2complex(i_s_abc)

        # Rate limit the frequency reference
        w_m_ref = self.rate_limiter(w_m_ref)

        # Slip compensation
        w_s_ref = w_m_ref + self.w_r_ref

        # Dynamic stator frequency and slip frequency
        w_s, w_r = self.stator_freq(w_s_ref, i_s)

        # Voltage reference
        u_s_ref = self.voltage_reference(w_s, i_s)

        # Compute the duty ratios
        d_abc_ref, u_s = self.pwm.output(u_s_ref, u_dc, self.theta_s, 0)

        # Data logging
        data = Bunch(i_s_ref=self.i_s_ref, i_s=i_s, u_s=u_s, w_m_ref=w_m_ref,
                     w_r=w_r, w_s=w_s, psi_s_ref=self.psi_s_ref,
                     theta_s=self.theta_s, u_dc=u_dc, T_s=self.pwm.T_s)
        self.datalog.save(data)

        # Update the states
        self.i_s_ref += self.T_s*self.alpha_i*(i_s - self.i_s_ref)
        self.w_r_ref += self.T_s*self.alpha_f*(w_r - self.w_r_ref)
        self.theta_s += self.T_s*w_s
        self.theta_s = np.mod(self.theta_s, 2*np.pi)    # Limit to [0, 2*pi]

        return d_abc_ref, self.T_s

    def stator_freq(self, w_s_ref, i_s):
        """
        Computes the dynamic stator frequency reference used in
        the coordinate transformations.

        """
        # Operating-point quantities
        psi_R_ref = self.psi_s_ref - self.L_sgm*self.i_s_ref
        psi_R_ref_sqr = np.abs(psi_R_ref)**2
        # Compute the dynamic stator frequency
        if psi_R_ref_sqr > 0:
            # Slip estimate based on the measured current
            w_r = self.R_R*np.imag(i_s*np.conj(psi_R_ref))/psi_R_ref_sqr
            # Dynamic frequency
            w_s = w_s_ref + self.k_w*(self.w_r_ref - w_r)
        else:
            w_s, w_r = 0, 0
        return w_s, w_r

    def voltage_reference(self, w_s, i_s):
        """
        Compute the stator voltage reference in synchronous coordinates.

        """
        # Nominal magnetizing current
        i_sd_nom = self.psi_s_ref/(self.L_M + self.L_sgm)
        # Operating-point current for RI compensation
        i_s_ref0 = i_sd_nom + 1j*self.i_s_ref.imag
        # Term -R_s omitted to avoid problems due to the voltage saturation
        # k = -R_s + k_u*L_sgm*(alpha + 1j*w_m0)
        k = self.k_u*self.L_sgm*(self.R_R/self.L_M + 1j*w_s)
        u_s_ref = (self.R_s*i_s_ref0 + 1j*w_s*self.psi_s_ref
                   + k*(self.i_s_ref - i_s))
        return u_s_ref

    def __str__(self):
        desc = (('V/Hz control\n'
                 '------------\n'
                 'Sampling period:\n'
                 '    T_s={}\n'
                 'Motor parameter estimates:\n'
                 '    R_s={}  R_R={}  L_sgm={}  L_M={}\n'
                 'Tuning parameters:\n'
                 '    k_u={:.1f}  k_w={:.1f}  alpha_f=2*pi*{:.1f}\n')
                .format(self.T_s, self.R_s, self.R_R, self.L_sgm,
                        self.L_M, self.k_u, self.k_w, self.alpha_f))
        return desc
