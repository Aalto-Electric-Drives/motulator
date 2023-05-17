"""
V/Hz control for induction motor drives.

The method is similar to [Hin2022]_. Open-loop V/Hz control can be obtained as a
special case by choosing::

    R_s, R_R = 0, 0
    k_u, k_w = 0, 0

Notes
-----
The low-pass-filtered values are marked with ref at the end of the variable
name. These slowly varying quasi-steady-state quantities can be seen to
represent the operating point (marked with the subscript 0 in [Hin2022]_).

References
----------
.. [Hin2022] Hinkkanen, Tiitinen, Mölsä, Harnefors, "On the stability of
   volts-per-hertz control for induction motors," IEEE J. Emerg. Sel. Topics
   Power Electron., 2022, https://doi.org/10.1109/JESTPE.2021.3060583

"""
# %%
import numpy as np
from motulator.control.common import Ctrl, PWM
from motulator.helpers import abc2complex, Bunch


# %%
class VHzCtrl(Ctrl):
    """
    V/Hz control with the stator current feedback.

    Parameters
    ----------
    pars : InductionMachinePars
        Control parameters.

    """

    # pylint: disable=too-many-instance-attributes
    def __init__(self, T_s, pars, psi_s_nom, k_u=1., k_w=4., six_step=False):
        super().__init__()
        self.T_s = T_s
        self.T_s0 = T_s
        self.w_m_ref = callable
        # Motor parameters
        self.R_s = pars.R_s
        self.R_R = pars.R_R
        self.L_sgm = pars.L_sgm
        self.L_M = pars.L_M
        # Control parameters
        self.k_u = k_u
        self.k_w = k_w
        self.psi_s_ref = psi_s_nom
        w_rb = self.R_R*(self.L_M + self.L_sgm)/(self.L_sgm*self.L_M)
        self.alpha_f = .1*w_rb
        self.alpha_i = .1*w_rb
        # Instantiate classes
        self.pwm = PWM(six_step=six_step)
        self.rate_limiter = callable
        self.six_step = six_step
        self.N = 100
        # States
        self.i_s_ref = 0j
        self.theta_s = 0
        self.w_r_ref = 0

    def __call__(self, mdl):
        """
        Run the main control loop.

        Parameters
        ----------
        mdl : InductionMotorDrive
            Continuous-time model of an induction motor drive for getting the
            feedback signals.

        Returns
        -------
        T_s : float
            Sampling period.
        d_abc : ndarray, shape (3,)
            Duty ratios.

        """
        # Rate limit the frequency reference
        w_m_ref = self.rate_limiter(self.T_s, self.w_m_ref(self.clock.t))

        # Measure the feedback signals
        i_s_abc = mdl.machine.meas_currents()  # Phase currents
        u_dc = mdl.converter.meas_dc_voltage()  # DC-bus voltage

        # Space vector transformation
        theta_s = self.theta_s
        i_s = np.exp(-1j*theta_s)*abc2complex(i_s_abc)

        # Slip compensation
        w_s_ref = w_m_ref + self.w_r_ref

        # Dynamic stator frequency and slip frequency
        w_s, w_r = self._stator_freq(w_s_ref, i_s)

        # Voltage reference
        u_s_ref = self._voltage_reference(w_s, i_s)

        # Synchronize the PWM to the stator frequency in overmodulation
        # (a simplistic approach)
        f_s = w_s/(2*np.pi)  # Stator frequency
        if self.six_step and self.N*np.abs(f_s) > 1/self.T_s0:
            self.T_s = 1/(self.N*f_s)
        else:
            self.T_s = self.T_s0

        # Data logging
        data = Bunch(
            i_s=i_s,
            i_s_ref=self.i_s_ref,
            psi_s_ref=self.psi_s_ref,
            t=self.clock.t,
            theta_s=self.theta_s,
            u_dc=u_dc,
            u_s=self.pwm.realized_voltage,
            w_m_ref=w_m_ref,
            w_r=w_r,
            w_s=w_s,
        )
        self.save(data)

        # Update the states
        self.i_s_ref += self.T_s*self.alpha_i*(i_s - self.i_s_ref)
        self.w_r_ref += self.T_s*self.alpha_f*(w_r - self.w_r_ref)
        self.theta_s += self.T_s*w_s  # Next line: limit into [-pi, pi)
        self.theta_s = np.mod(self.theta_s + np.pi, 2*np.pi) - np.pi
        self.clock.update(self.T_s)

        # Compute the duty ratios
        d_abc = self.pwm(self.T_s, u_s_ref, u_dc, theta_s, w_s)

        return self.T_s, d_abc

    def _stator_freq(self, w_s_ref, i_s):
        """Compute the stator frequency for coordinate transformations."""
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

    def _voltage_reference(self, w_s, i_s):
        """Compute the stator voltage reference."""
        # Nominal magnetizing current
        i_sd_nom = self.psi_s_ref/(self.L_M + self.L_sgm)
        # Operating-point current for RI compensation
        i_s_ref0 = i_sd_nom + 1j*self.i_s_ref.imag
        # Term -R_s omitted to avoid problems due to the voltage saturation
        # k = -R_s + k_u*L_sgm*(alpha + 1j*w_m0)
        k = self.k_u*self.L_sgm*(self.R_R/self.L_M + 1j*w_s)
        u_s_ref = (
            self.R_s*i_s_ref0 + 1j*w_s*self.psi_s_ref + k*(self.i_s_ref - i_s))
        return u_s_ref
