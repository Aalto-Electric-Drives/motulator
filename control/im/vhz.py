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
import matplotlib.pyplot as plt
from cycler import cycler
from control.common import PWM, RateLimiter
from helpers import abc2complex, complex2abc


# %%
class VHzCtrl:
    """
    V/Hz control algorithm with the stator current feedback.

    """

    def __init__(self, pars, datalog):
        """
        Instantiate the classes and get the parameters.

        """
        # Instantiate classes
        self.pwm = PWM(pars)
        self.rate_limiter = RateLimiter(pars)
        self.datalog = datalog
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
        self.desc = (('V/Hz control\n'
                      '------------\n'
                      'Sampling period:\n'
                      '    T_s={}\n'
                      'Motor parameter estimates:\n'
                      '    R_s={}  R_R={}  L_sgm={}  L_M={}\n'
                      'Tuning parameters:\n'
                      '    k_u={:.1f}  k_w={:.1f}  alpha_f=2*pi*{:.1f}')
                     .format(self.T_s, self.R_s, self.R_R, self.L_sgm,
                             self.L_M, self.k_u, self.k_w, self.alpha_f))

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

        # Datalogger
        self.datalog.save([self.i_s_ref, i_s, u_s, w_m_ref, w_s, w_r,
                           self.psi_s_ref, self.theta_s, u_dc, self.T_s])

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
        return self.desc


# %%
class Datalogger:
    """
    This class contains a data logger.

    """

    def __init__(self):
        """
        Initialize the attributes.

        """
        self.t = []
        self.i_s_ref = []
        self.i_s = []
        self.u_s = []
        self.w_m_ref = []
        self.w_r = []
        self.w_s = []
        self.theta_s = []
        self.u_dc = []
        self.psi_s_ref = []
        self.u_ss, self.i_ss = 0j, 0j

    def save(self, data):
        """
        Saves the solution.

        Parameters
        ----------
        mdl : instance of a class
            Continuous-time model.

        """
        (i_s_ref, i_s, u_s, w_m_ref, w_s, w_r,
         psi_s_ref, theta_s, u_dc, T_s) = data

        try:
            t_new = self.t[-1] + T_s
        except IndexError:
            t_new = 0   # At the first step t = []
        self.t.extend([t_new])
        self.i_s_ref.extend([i_s_ref])
        self.i_s.extend([i_s])
        self.u_s.extend([u_s])
        self.w_m_ref.extend([w_m_ref])
        self.w_r.extend([w_r])
        self.w_s.extend([w_s])
        self.theta_s.extend([theta_s])
        self.u_dc.extend([u_dc])
        self.psi_s_ref.extend([psi_s_ref])

    def post_process(self):
        """
        Transforms the lists to the ndarray format and post-process them.

        """
        self.i_s_ref = np.asarray(self.i_s_ref)
        self.i_s = np.asarray(self.i_s)
        self.u_s = np.asarray(self.u_s)
        self.w_m_ref = np.asarray(self.w_m_ref)
        self.w_r = np.asarray(self.w_r)
        self.w_s = np.asarray(self.w_s)
        self.theta_s = np.asarray(self.theta_s)
        self.u_dc = np.asarray(self.u_dc)
        self.psi_s_ref = np.asarray(self.psi_s_ref)
        self.u_ss = np.exp(1j*self.theta_s)*self.u_s
        self.i_ss = np.exp(1j*self.theta_s)*self.i_s

    def plot(self, mdl, base):
        """
        Plots example figures.

        Parameters
        ----------
        mdl : object
            Continuous-time solution.
        base : object
            Base values.

        """
        data = mdl.datalog          # Continuous-time data
        t_range = (0, self.t[-1])   # Time span

        # Plotting parameters
        plt.rcParams['axes.prop_cycle'] = cycler(color='brgcmyk')
        plt.rcParams['lines.linewidth'] = 1.
        plt.rcParams['axes.grid'] = True
        plt.rcParams.update({"text.usetex": False})

        fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5, 1, figsize=(8, 10))

        ax1.step(self.t, self.w_m_ref/base.w, '--', where='post')
        ax1.step(self.t, self.w_s/base.w, where='post')
        ax1.plot(data.t, data.w_m/base.w)
        ax1.legend([r'$\omega_\mathrm{m,ref}$',
                    r'$\omega_\mathrm{s}$',
                    r'$\omega_\mathrm{m}$'])
        ax1.set_xlim(t_range)
        ax1.set_xticklabels([])
        ax1.set_ylabel('Speed (p.u.)')

        ax2.plot(data.t, data.tau_L/base.tau, '--')
        ax2.plot(data.t, data.tau_M/base.tau)
        ax2.set_xlim(t_range)
        ax2.legend([r'$\tau_\mathrm{L}$', r'$\tau_\mathrm{M}$'])
        ax2.set_ylabel('Torque (p.u.)')
        ax2.set_xticklabels([])

        ax3.step(self.t, self.i_s_ref.real/base.i, '--', where='post')
        ax3.step(self.t, self.i_s.real/base.i, where='post')
        ax3.step(self.t, self.i_s_ref.imag/base.i, '--', where='post')
        ax3.step(self.t, self.i_s.imag/base.i, where='post')
        ax3.set_ylabel('Current (p.u.)')
        ax3.legend([r'$i_\mathrm{sd,ref}$', r'$i_\mathrm{sd}$',
                    r'$i_\mathrm{sq,ref}$', r'$i_\mathrm{sq}$'])
        ax3.set_xlim(t_range)
        ax3.set_xticklabels([])

        ax4.step(self.t, np.abs(self.u_s)/base.u, where='post')
        ax4.step(self.t, self.u_dc/np.sqrt(3)/base.u, '--', where='post')
        ax4.set_ylabel('Voltage (p.u.)')
        ax4.set_xlim(t_range)
        ax4.set_ylim(0, 1.2)
        ax4.legend([r'$u_\mathrm{s}$', r'$u_\mathrm{dc}/\sqrt{3}$'])
        ax4.set_xticklabels([])

        ax5.plot(data.t, np.abs(data.psi_ss)/base.psi)
        ax5.plot(data.t, np.abs(data.psi_Rs)/base.psi)
        ax5.set_xlim(t_range)
        ax5.set_ylim(0, 1.2)
        ax5.legend([r'$\psi_\mathrm{s}$', r'$\psi_\mathrm{R}$'])
        ax5.set_ylabel('Flux (p.u.)')
        ax5.set_xlabel('Time (s)')

        fig.align_ylabels()
        plt.tight_layout()
        plt.show()

    def plot_latex(self, mdl, base):
        """
        Plots example figures using LaTeX in a format suitable for two-column
        articles. This method requires that LaTeX is installed.

        Parameters
        ----------
        mdl : object
            Continuous-time solution.
        base : object
            Base values.

        """
        data = mdl.datalog          # Continuous-time data
        t_range = (0, self.t[-1])   # Time span

        # Plotting parameters
        plt.rcParams['axes.prop_cycle'] = cycler(color='brgcmyk')
        plt.rcParams['lines.linewidth'] = 1.
        plt.rcParams['axes.grid'] = True
        plt.rcParams.update({"text.usetex": True,
                             "font.family": "serif",
                             "font.sans-serif": ["Computer Modern Roman"]})

        fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5, 1, figsize=(3, 7.5))

        ax1.step(self.t, self.w_m_ref/base.w, '--', where='post')
        ax1.plot(data.t, data.w_m/base.w)
        ax1.legend([r'$\omega_\mathrm{m,ref}$',
                    r'$\omega_\mathrm{m}$'])
        ax1.set_xlim(t_range)
        ax1.set_ylim(-1.2, 1.2)
        ax1.set_xticklabels([])
        ax1.set_ylabel('Speed (p.u.)')

        ax2.plot(data.t, data.tau_L/base.tau, '--')
        ax2.plot(data.t, data.tau_M/base.tau)
        ax2.set_xlim(t_range)
        ax2.set_ylim(-.2, 1)
        ax2.legend([r'$\tau_\mathrm{L}$', r'$\tau_\mathrm{M}$'])
        ax2.set_ylabel('Torque (p.u.)')
        ax2.set_xticklabels([])

        ax3.step(self.t, self.i_s.real/base.i, where='post')
        ax3.step(self.t, self.i_s.imag/base.i, where='post')
        ax3.set_ylabel('Current (p.u.)')
        ax3.legend([r'$i_\mathrm{sd}$',  r'$i_\mathrm{sq}$'])
        ax3.set_xlim(t_range)
        ax3.set_ylim(-.2, 1.5)
        ax3.set_xticklabels([])

        ax4.step(self.t, np.abs(self.u_s)/base.u, where='post')
        ax4.step(self.t, self.u_dc/np.sqrt(3)/base.u, '--', where='post')
        ax4.set_ylabel('Voltage (p.u.)')
        ax4.set_xlim(t_range)
        ax4.set_ylim(0, 1.2)
        ax4.legend([r'$u_\mathrm{s}$', r'$u_\mathrm{dc}/\sqrt{3}$'])
        ax4.set_xticklabels([])

        ax5.plot(data.t, np.abs(data.psi_ss)/base.psi)
        ax5.plot(data.t, np.abs(data.psi_Rs)/base.psi)
        ax5.set_xlim(t_range)
        ax5.set_ylim(0, 1.2)
        ax5.legend([r'$\psi_\mathrm{s}$', r'$\psi_\mathrm{R}$'])
        ax5.set_ylabel('Flux (p.u.)')
        ax5.set_xlabel('Time (s)')

        fig.align_ylabels()
        plt.tight_layout()
        plt.show()
        # plt.savefig('fig.pdf')

    def plot_extra(self, mdl, base):
        """
        Plots extra waveforms if the PWM is enabled or if the DC-bus dynamics
        are modeled.

        Parameters
        ----------
        t : ndarray
            Discrete time.
        mdl : object
            Continuous-time solution.
        base : object
            Base values.

        """
        # Continuous-time data
        data = mdl.datalog
        # Time span
        t_zoom = (.9, .925)

        # Plotting parameters
        plt.rcParams['axes.prop_cycle'] = cycler(color='brgcmyk')
        plt.rcParams['lines.linewidth'] = 1.
        plt.rcParams.update({"text.usetex": False})

        if mdl.pwm is not None:
            # Plots a zoomed view of voltages and currents
            fig1, (ax1, ax2) = plt.subplots(2, 1)
            ax1.plot(data.t, data.u_ss.real/base.u)
            ax1.plot(self.t, self.u_ss.real/base.u)
            ax1.set_xlim(t_zoom)
            ax1.set_ylim(-1.5, 1.5)
            ax1.legend([r'$u_\mathrm{sa}$', r'$\hat u_\mathrm{sa}$'])
            ax1.set_ylabel('Voltage (p.u.)')
            ax1.set_xticklabels([])
            ax2.plot(data.t, complex2abc(data.i_ss).T/base.i)
            ax2.step(self.t, self.i_ss.real/base.i, where='post')
            ax2.set_xlim(t_zoom)
            ax2.legend([r'$i_\mathrm{sa}$', r'$i_\mathrm{sb}$',
                        r'$i_\mathrm{sc}$'])
            ax2.set_ylabel('Current (p.u.)')
            ax2.set_xlabel('Time (s)')
            fig1.align_ylabels()

        # Plots the DC bus and grid-side variables (if data exists)
        try:
            data.i_L
        except AttributeError:
            data.i_L = None
        if data.i_L is not None:
            fig2, (ax1, ax2) = plt.subplots(2, 1)
            ax1.plot(data.t, data.u_di/base.u)
            ax1.plot(data.t, data.u_dc/base.u)
            ax1.plot(data.t, complex2abc(data.u_g).T/base.u)
            ax1.set_xlim(t_zoom)
            ax1.set_ylim(-1.5, 2)
            ax1.set_xticklabels([])
            ax1.legend([r'$u_\mathrm{di}$',
                        r'$u_\mathrm{dc}$',
                        r'$u_\mathrm{ga}$'])
            ax1.set_ylabel('Voltage (p.u.)')
            ax2.plot(data.t, data.i_L/base.i)
            ax2.plot(data.t, data.i_dc/base.i)
            ax2.plot(data.t, data.i_g.real/base.i)
            ax2.set_xlim(t_zoom)
            ax2.legend([r'$i_\mathrm{L}$',
                        r'$i_\mathrm{dc}$',
                        r'$i_\mathrm{ga}$'])
            ax2.set_ylabel('Current (p.u.)')
            ax2.set_xlabel('Time (s)')
            fig2.align_ylabels()

        plt.tight_layout()
        plt.show()
