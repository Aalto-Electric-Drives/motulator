# pylint: disable=C0103
'''
This module includes V/Hz control for induction motor drives. The method
corresponds to the paper "On the stability of volts-per-hertz control for
induction motors":

    https://doi.org/10.1109/JESTPE.2021.3060583

Open-loop V/Hz control can be obtained as a special case by choosing:

    R_s, R_R = 0, 0
    k_u, k_w = 0, 0

Notes
-----
The reference values and low-pass-filtered values are marked with 0 at
the end of the variable name. These slowly varying quasi-steady-state
quantities can be seen to represent the operating point.

'''
# %%
import numpy as np
import matplotlib.pyplot as plt
from cycler import cycler
from control.common import RateLimiter, PWM
from helpers import abc2complex, complex2abc


# %%
class ScalarCtrl:
    """
    V/Hz control algorithm with the stator current feedback.

    """

    def __init__(self, pars, datalog):

        # Parameters
        self.pars = pars
        # Compute the breakdown slip angular frequency
        w_rb = (pars.L_M + pars.L_sgm)/pars.L_sgm*(pars.R_R/pars.L_M)
        # LPF bandwidth
        self.alpha_f = .1*w_rb
        # States
        self.i_s0 = 0
        self.theta_s = 0
        # Instantiate classes
        self.pwm = PWM(pars)
        self.rate_limiter = RateLimiter(pars)
        self.datalog = datalog

    def __call__(self, w_m_ref, i_s_abc, u_dc):
        """
        Main control loop.

        Parameters
        ----------
        w_m_ref : float
            Rotor speed reference (in electrical rad/s).
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
        T_s = self.pars.T_s

        # Get the states
        theta_s = self.theta_s
        i_s0 = self.i_s0

        # Limit the rate of change of the speed reference (in electrical rad/s)
        w_m0 = self.rate_limiter(w_m_ref)

        # Space vector transformation
        i_s = np.exp(-1j*theta_s)*abc2complex(i_s_abc)

        w_s = self.stator_freq(w_m0, i_s, i_s0)
        u_s_ref = self.voltage_reference(w_s, w_m0, i_s, i_s0)

        # Compute the duty ratios
        d_abc_ref, u_s = self.pwm.output(u_s_ref, u_dc, theta_s, 0)

        # Update the states
        self.update(w_s, i_s)

        # Data logger
        self.datalog.save([i_s0, i_s, u_s, w_m_ref, w_m0, w_s, theta_s,
                           u_dc, self.pars.psi_s_nom, T_s])

        return d_abc_ref, T_s

    def stator_freq(self, w_m_ref, i_s, i_s0):
        """
        Computes the dynamic stator frequency reference used in
        the coordinate transformations.

        """
        k_w = self.pars.k_w
        R_R = self.pars.R_R
        L_sgm = self.pars.L_sgm
        psi_s_nom = self.pars.psi_s_nom

        # Operating-point quantities are marked with zero
        psi_R0 = psi_s_nom - L_sgm*i_s0
        psi_R0_sqr = np.abs(psi_R0)**2

        # Compute the dynamic stator frequency
        if psi_R0_sqr > 0:
            # Operating-point slip
            w_r0 = R_R*np.imag(i_s0*np.conj(psi_R0))/psi_R0_sqr
            # Operating-point stator frequency
            w_s0 = w_m_ref + w_r0
            # Dynamic frequency
            err = R_R*np.imag((i_s0 - i_s)*np.conj(psi_R0))/psi_R0_sqr
            w_s = w_s0 + k_w*err
        else:
            w_s = 0

        return w_s

    def voltage_reference(self, w_s, w_m0, i_s, i_s0):
        """
        Compute the stator voltage reference in synchronous coordinates.

        """
        R_s = self.pars.R_s
        L_sgm = self.pars.L_sgm
        alpha = self.pars.R_R/self.pars.L_M
        k_u = self.pars.k_u
        psi_s_nom = self.pars.psi_s_nom
        # Nominal magnetizing current
        i_sd_nom = psi_s_nom/(self.pars.L_M + L_sgm)
        # Operating-point current for RI compensation
        i_s_ref0 = i_sd_nom + 1j*i_s0.imag
        # Term -R_s omitted to avoid problems due to the voltage saturation
        # k = -R_s + k_u*L_sgm*(alpha + 1j*w_m0)
        k = k_u*L_sgm*(alpha + 1j*w_m0)
        u_s_ref = R_s*i_s_ref0 + 1j*w_s*psi_s_nom + k*(i_s0 - i_s)

        return u_s_ref

    def update(self, w_s, i_s):
        """
        Update the states.

        """
        T_s = self.pars.T_s

        self.i_s0 += T_s*self.alpha_f*(i_s - self.i_s0)
        self.theta_s += T_s*w_s                         # Integrate
        self.theta_s = np.mod(self.theta_s, 2*np.pi)    # Limit to [0, 2*pi]

    def __str__(self):
        desc = ('V/Hz control:\n'
                '    R_s={}  R_R={}  L_sgm={}  L_M={}\n'
                '    k_u={:.1f}  k_w={:.1f}  alpha_f=2*pi*{:.1f}')
        return desc.format(self.pars.R_s, self.pars.R_R,
                           self.pars.L_sgm, self.pars.L_M,
                           self.pars.k_u, self.pars.k_w, self.alpha_f)


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
        self.i_s0 = []
        self.i_s = []
        self.u_s = []
        self.w_m_ref = []
        self.w_m = []
        self.w_s = []
        self.theta_s = []
        self.u_dc = []
        self.psi_s_nom = []
        self.u_ss, self.i_ss = 0j, 0j

    def save(self, data):
        """
        Saves the solution.

        Parameters
        ----------
        mdl : instance of a class
            Continuous-time model.

        """
        (i_s0, i_s, u_s, w_m_ref, w_m, w_s,
         theta_s, u_dc, psi_s_nom, T_s) = data
        try:
            t_new = self.t[-1] + T_s
        except IndexError:
            t_new = 0   # At the first step t = []
        self.t.extend([t_new])
        self.i_s0.extend([i_s0])
        self.i_s.extend([i_s])
        self.u_s.extend([u_s])
        self.w_m_ref.extend([w_m_ref])
        self.w_m.extend([w_m])
        self.w_s.extend([w_s])
        self.theta_s.extend([theta_s])
        self.u_dc.extend([u_dc])
        self.psi_s_nom.extend([psi_s_nom])

    def post_process(self):
        """
        Transforms the lists to the ndarray format and post-process them.

        """
        self.i_s0 = np.asarray(self.i_s0)
        self.i_s = np.asarray(self.i_s)
        self.u_s = np.asarray(self.u_s)
        self.w_m_ref = np.asarray(self.w_m_ref)
        self.w_m = np.asarray(self.w_m)
        self.w_s = np.asarray(self.w_s)
        self.theta_s = np.asarray(self.theta_s)
        self.u_dc = np.asarray(self.u_dc)
        self.psi_s_nom = np.asarray(self.psi_s_nom)
        self.u_ss = np.exp(1j*self.theta_s)*self.u_s
        self.i_ss = np.exp(1j*self.theta_s)*self.i_s

    def plot(self, mdl):
        """
        Plots some example figures.

        Parameters
        ----------
        t : ndarray
            Discrete time.
        mdl : instance of a class
            Continuous-time solution.

        """
        # Continuous-time data
        data = mdl.datalog
        # Plotting parameters
        plt.rcParams['axes.prop_cycle'] = cycler(color='brgcmyk')
        plt.rcParams['lines.linewidth'] = 1.
        plt.rcParams.update({"text.usetex": True,
                             "font.family": "serif",
                             "font.sans-serif": ["Computer Modern Roman"]})
        t_range = (0, self.t[-1])
        t_zoom = (1.2, 1.225)

        # Plots speeds and torques
        fig1, (ax1, ax2) = plt.subplots(2, 1)
        ax1.step(self.t, self.w_m_ref, '--', where='post')
        ax1.plot(data.t, data.w_m)
        ax1.step(self.t, self.w_m, where='post')
        ax1.legend([r'$\omega_\mathrm{m,ref}$',
                    r'$\omega_\mathrm{m}$',
                    r'$\omega_\mathrm{m0}$'])
        ax1.set_xlim(t_range)
        ax1.set_ylabel('Angular speed (rad/s)')
        ax2.plot(data.t, data.T_L, '--')
        ax2.plot(data.t, data.T_M)
        ax2.set_xlim(t_range)
        ax2.legend([r'$\tau_\mathrm{L}$', r'$\tau_\mathrm{m}$'])
        ax2.set_ylabel('Torque (Nm)')
        ax2.set_xlabel('Time (s)')
        # Plots currents components and flux magnitudes
        fig2, (ax1, ax2, ax3) = plt.subplots(3, 1)
        ax1.step(self.t, self.i_s0.real, '--', where='post')
        ax1.step(self.t, self.i_s.real, where='post')
        ax1.step(self.t, self.i_s0.imag, '--', where='post')
        ax1.step(self.t, self.i_s.imag, where='post')
        ax1.set_ylabel('Current (A)')
        ax1.legend([r'$i_\mathrm{sd0}$', r'$i_\mathrm{sd}$',
                    r'$i_\mathrm{sq0}$', r'$i_\mathrm{sq}$'])
        ax1.set_xlim(t_range)
        ax2.step(self.t, self.psi_s_nom, '--', where='post')
        ax2.plot(data.t, np.abs(data.psi_ss))
        ax2.plot(data.t, np.abs(data.psi_Rs))
        ax2.set_xlim(t_range)
        ax2.legend([r'$\psi_\mathrm{s,ref}$',
                    r'$\psi_\mathrm{s}$', r'$\psi_\mathrm{R}$'])
        ax2.set_ylabel('Flux linkage (Vs)')
        ax3.step(self.t, np.abs(self.u_s), where='post')
        ax3.step(self.t, self.u_dc/np.sqrt(3), '--', where='post')
        ax3.set_ylabel('Voltage (V)')
        ax3.set_xlim(t_range)
        ax3.legend([r'$u_\mathrm{s}$', r'$u_\mathrm{dc}/\sqrt{3}$'])
        ax3.set_xlabel('Time (s)')
        if mdl.pwm is not None:
            # Plots a zoomed view of voltages and currents
            fig3, (ax1, ax2) = plt.subplots(2, 1)
            ax1.plot(data.t, data.u_ss.real)
            ax1.plot(self.t, self.u_ss.real)
            ax1.set_xlim(t_zoom)
            ax1.legend([r'$u_\mathrm{sa}$', r'$\hat u_\mathrm{sa}$'])
            ax1.set_ylabel('Voltage (V)')
            ax2.plot(data.t, complex2abc(data.i_ss).T)
            ax2.step(self.t, self.i_ss.real, where='post')
            ax2.set_xlim(t_zoom)
            ax2.set_ylim(-10, 10)
            ax2.legend([r'$i_\mathrm{a}$', r'$i_\mathrm{b}$',
                        r'$i_\mathrm{c}$'])
            ax2.set_ylabel('Current (A)')
            ax2.set_xlabel('Time (s)')
        else:
            fig3 = None
        # Plots the DC bus and grid-side variables (if data exists)
        try:
            data.i_L
        except AttributeError:
            data.i_L = None
        if data.i_L is not None:
            fig4, (ax1, ax2) = plt.subplots(2, 1)
            ax1.plot(data.t, data.u_di)
            ax1.plot(data.t, data.u_dc)
            ax1.plot(data.t, complex2abc(data.u_g).T)
            ax1.set_xlim(t_zoom)
            ax1.legend([r'$u_\mathrm{di}$',
                        r'$u_\mathrm{dc}$',
                        r'$u_\mathrm{ga}$'])
            ax1.set_ylabel('Voltage (V)')
            ax2.plot(data.t, data.i_L)
            ax2.plot(data.t, data.i_dc)
            ax2.plot(data.t, data.i_g.real)
            ax2.set_xlim(t_zoom)
            ax2.legend([r'$i_\mathrm{L}$',
                        r'$i_\mathrm{dc}$',
                        r'$i_\mathrm{ga}$'])
            ax2.set_ylabel('Current (A)')
            ax2.set_xlabel('Time (s)')
        else:
            fig4 = None
        plt.show()
        # plt.savefig('test.pdf')
        return fig1, fig2, fig3, fig4
