# pylint: disable=C0103
"""
This module contains vector control for PMSM drives.

"""

# %%
import numpy as np
import matplotlib.pyplot as plt
from cycler import cycler
from helpers import abc2complex, complex2abc
from control.common import PWM


# %%
class VectorCtrl:
    """
    This class interconnects the subsystems of the PMSM control system and
    provides the interface to the solver.

    """

    def __init__(self, pars, speed_ctrl, current_ref, current_ctrl, datalog):
        """
        Instantiate the classes.

        """
        self.p = pars.p
        self.current_ctrl = current_ctrl
        self.speed_ctrl = speed_ctrl
        self.current_ref = current_ref
        self.pwm = PWM(pars)
        self.datalog = datalog
        self.desc = (('Sensored vector control for synchronous motors\n'
                      '----------------------------------------------\n'
                      'Sampling period:\n'
                      '    T_s={}\n'
                      'Motor parameter estimates:\n'
                      '    p={}  R_s={}  L_d={}  L_q={}  psi_f={}\n')
                     .format(pars.T_s, pars.p, pars.R_s, pars.L_d, pars.L_q,
                             pars.psi_f))
        self.desc += (self.current_ref.desc + self.current_ctrl.desc
                      + self.speed_ctrl.desc)

    def __call__(self, w_m_ref, w_M, theta_M, i_s_abc, u_dc):
        """
        Main control loop.

        Parameters
        ----------
        w_m_ref : float
            Rotor speed reference (in electrical rad/s).
        w_M : float
            Rotor speed (in mechanical rad/s).
        theta_M : float
            Rotor angle (in mechanical rad).
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
        # Get the states
        u_s = self.pwm.realized_voltage
        w_m = self.p*w_M
        theta_m = np.mod(self.p*theta_M, 2*np.pi)

        # Space vector and coordinate transformation
        i_s = np.exp(-1j*theta_m)*abc2complex(i_s_abc)

        # Outputs
        tau_M_ref, tau_L = self.speed_ctrl.output(w_m_ref/self.p, w_M)
        i_s_ref, tau_M = self.current_ref.output(tau_M_ref, w_m, u_dc)
        u_s_ref, e = self.current_ctrl.output(i_s_ref, i_s)
        d_abc_ref, u_s_ref_lim = self.pwm.output(u_s_ref, u_dc, theta_m, w_m)

        # Update all the states
        self.speed_ctrl.update(tau_M, tau_L)
        self.current_ref.update(tau_M, u_s_ref, u_dc)
        self.current_ctrl.update(e, u_s_ref, u_s_ref_lim, w_m)
        self.pwm.update(u_s_ref_lim)

        # Data logging
        self.datalog.save([i_s_ref, i_s, u_s, np.nan, w_m_ref, w_m, theta_m,
                           u_dc, tau_M, self.pwm.T_s])

        return d_abc_ref, self.pwm.T_s

    def __str__(self):
        return self.desc


# %%
class SensorlessVectorCtrl:
    """
    This class interconnects the subsystems of the PMSM control system and
    provides the interface to the solver.

    """

    def __init__(self, pars, speed_ctrl, current_ref, current_ctrl, observer,
                 datalog):
        """
        Instantiate the classes.

        """
        self.p = pars.p
        self.current_ctrl = current_ctrl
        self.speed_ctrl = speed_ctrl
        self.current_ref = current_ref
        self.observer = observer
        self.pwm = PWM(pars)
        self.datalog = datalog
        self.desc = (('Sensorless vector control for synchronous motors\n'
                      '------------------------------------------------\n'
                      'Sampling period:\n'
                      '    T_s={}\n'
                      'Motor parameter estimates:\n'
                      '    p={}  R_s={}  L_d={}  L_q={}  psi_f={}\n')
                     .format(pars.T_s, pars.p, pars.R_s, pars.L_d, pars.L_q,
                             pars.psi_f))
        self.desc += (self.current_ref.desc + self.current_ctrl.desc
                      + self.speed_ctrl.desc)

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
        # Get the states
        u_s = self.pwm.realized_voltage
        w_m = self.observer.w_m
        theta_m = self.observer.theta_m
        psi_s = self.observer.psi_s

        # Space vector and coordinate transformation
        i_s = np.exp(-1j*theta_m)*abc2complex(i_s_abc)

        # Outputs
        tau_M_ref, tau_L = self.speed_ctrl.output(w_m_ref/self.p, w_m/self.p)
        i_s_ref, tau_M = self.current_ref.output(tau_M_ref, w_m, u_dc)
        u_s_ref, e = self.current_ctrl.output(i_s_ref, i_s, psi_s, w_m)
        d_abc_ref, u_s_ref_lim = self.pwm.output(u_s_ref, u_dc, theta_m, w_m)

        # Update all the states
        self.observer.update(u_s, i_s)
        self.speed_ctrl.update(tau_M, tau_L)
        self.current_ref.update(tau_M, u_s_ref, u_dc)
        self.current_ctrl.update(e, u_s_ref, u_s_ref_lim, w_m)
        self.pwm.update(u_s_ref_lim)

        # Data logging
        self.datalog.save([i_s_ref, i_s, u_s, psi_s, w_m_ref, w_m, theta_m,
                           u_dc, tau_M, self.pwm.T_s])

        return d_abc_ref, self.pwm.T_s

    def __str__(self):
        return self.desc


# %%
class CurrentCtrl2DOFPI:
    """
    A current controller corresponding to the paper "Flux-linkage-based current
    control of saturated synchronous motors":

        https://doi.org/10.1109/TIA.2019.291925

    The continuous-time complex-vector design corresponding to (13) is used
    here. This design could be equivalently presented as a 2DOF PI controller.
    For better performance at high speeds with low sampling frequencies, the
    discrete-time design in (18) is recommended.

    """

    def __init__(self, pars):
        """
        Parameters
        ----------
        pars : data object
            Controller parameters.

        """
        self.T_s = pars.T_s
        self.L_d = pars.L_d
        self.L_q = pars.L_q
        self.alpha_c = pars.alpha_c
        self.u_i = 0  # Integral state
        self.desc = (('2DOF PI current control:\n'
                      '    alpha_c=2*pi*{:.1f}\n')
                     .format(pars.alpha_c/(2*np.pi)))

    def output(self, i_s_ref, i_s, *_):
        """
        Compute the unlimited voltage reference.

        Parameters
        ----------
        i_s_ref : complex
            Current reference.
        i_s : complex
            Measured current.
        *_ : not used
            Just for compatibility.

        Returns
        -------
        u_s_ref : complex
            Unlimited voltage reference.
        e : complex
            Error signal (scaled, corresponds to the stator flux linkage).

        """
        # Gains
        k_t = self.alpha_c
        k = 2*self.alpha_c
        # PM-flux linkage cancels out
        psi_s_ref = self.L_d*i_s_ref.real + 1j*self.L_q*i_s_ref.imag
        psi_s = self.L_d*i_s.real + 1j*self.L_q*i_s.imag
        u_s_ref = k_t*psi_s_ref - k*psi_s + self.u_i
        e = psi_s_ref - psi_s

        return u_s_ref, e

    def update(self, e, u_s_ref, u_s_ref_lim, w_m):
        """
        Updates the integral state.

        Parameters
        ----------
        e : complex
            Error signal (scaled, corresponds to the stator flux linkage).
        u_s_ref : complex
            Unlimited voltage reference.
        u_s_ref_lim : complex
            Limited voltage reference.
        w_m : float
            Angular rotor speed.

        """
        k_t = self.alpha_c
        k_i = self.alpha_c*(self.alpha_c + 1j*w_m)
        self.u_i += self.T_s*k_i*(e + (u_s_ref_lim - u_s_ref)/k_t)

    def __str__(self):
        return self.desc


# %%
class CurrentCtrl:
    """
    This class represents a state-feedback current controller, with reference
    feedforward, without integral action.

    """

    def __init__(self, pars):
        """
        Parameters
        ----------
        pars : data object
            Controller parameters.

        """
        self.alpha_c = pars.alpha_c
        self.L_d = pars.L_d
        self.L_q = pars.L_q
        self.R_s = pars.R_s
        self.desc = (('State-feedback current control (no integral action):\n'
                      '    alpha_c=2*pi*{:.1f}\n')
                     .format(self.alpha_c/(2*np.pi)))

    def output(self, i_s_ref, i_s, psi_s, w_m):
        """
        State-feedback current controller.

        Parameters
        ----------
        i_s_ref : complex
            Stator current reference.
        i_s : complex
            Stator current.
        psi_s : complex
            Stator flux linkage.
        w_m : float
            Rotor speed (in electrical rad/s).

        Returns
        -------
        u_s_ref : complex
            Voltage reference.
        e : complex
            Flux linkage error (not needed, just for compatibility).

        """
        # Map current error to the flux linkage error
        e = self.L_d*(i_s_ref - i_s).real + 1j*self.L_q*(i_s_ref - i_s).imag
        # Voltage reference in rotor coordinates
        u_s_ref = self.R_s*i_s + 1j*w_m*psi_s + self.alpha_c*e

        return u_s_ref, e

    def update(self, *_):
        """
        No states, nothing to update. This method is just for compatibility.

        """

    def __str__(self):
        return self.desc


# %%
class CurrentRef:
    """
    This reference calculation method resembles the method presented in
    "Analytical design and autotuning of adaptive flux-weakening voltage
    regulation loop in IPMSM drives with accurate torque regulation":

        https://doi.org/10.1109/TIA.2019.2942807

    Instead of the PI controller, we use a simpler integral controller with a
    constant gain. The resulting operating-point-dependent closed-loop pole
    could be derived using (12) of the paper. The MTPV limit is also included
    by means of limiting the reference torque and the d-axis current reference.

    """

    def __init__(self, pars):
        """
        Parameters
        ----------
        pars : data object
            Controller parameters.

        """
        self.T_s = pars.T_s
        self.i_s_max = pars.i_s_max
        self.p = pars.p
        self.L_d = pars.L_d
        self.L_q = pars.L_q
        self.psi_f = pars.psi_f
        self.k = pars.alpha_fw/(pars.w_nom*self.L_d)
        self.k_u = pars.k_u
        self.tau_M_lim = pars.tau_M_lim
        self.i_sd_mtpa = pars.i_sd_mtpa
        self.i_sd_lim = pars.i_sd_lim
        self.i_sd_ref = 0
        self.desc = ('Current reference computation and field weakening:\n'
                     '    i_s_max={:.1f}\n').format(pars.i_s_max)

    def output(self, tau_M_ref, w_m, u_dc):
        """
        Compute the stator current reference.

        Parameters
        ----------
        tau_M_ref : float
            Torque reference.
        w_m : float
            Rotor speed (in electrical rad/s)
        u_dc : float
            DC-bus voltage.

        Returns
        -------
        i_s_ref : complex
            Stator current reference.
        tau_M : float
            Limited torque reference.

        """
        def limit_torque(tau_M_ref, w_m, u_dc):
            if np.abs(w_m) > 0:
                psi_s_max = self.k_u*u_dc/np.sqrt(3)/np.abs(w_m)
                tau_M_max = self.tau_M_lim(psi_s_max)
            else:
                tau_M_max = self.tau_M_lim(np.inf)

            if np.abs(tau_M_ref) > tau_M_max:
                tau_M_ref = np.sign(tau_M_ref)*tau_M_max

            return tau_M_ref

        # Limit the torque reference according to MTPV and current limits
        tau_M_ref = limit_torque(tau_M_ref, w_m, u_dc)

        # q-axis current reference
        psi_t = self.psi_f + (self.L_d - self.L_q)*self.i_sd_ref
        if psi_t != 0:
            i_sq_ref = tau_M_ref/(1.5*self.p*psi_t)
        else:
            i_sq_ref = 0

        # Limit the q-axis current reference
        i_sd_mtpa = self.i_sd_mtpa(np.abs(tau_M_ref))
        i_sq_max = np.min([np.sqrt(self.i_s_max**2 - self.i_sd_ref**2),
                           np.sqrt(self.i_s_max**2 - i_sd_mtpa**2)])
        if np.abs(i_sq_ref) > i_sq_max:
            i_sq_ref = np.sign(i_sq_ref)*i_sq_max

        # Current reference
        i_s_ref = self.i_sd_ref + 1j*i_sq_ref

        # Limited torque (for the speed controller)
        tau_M = 1.5*self.p*psi_t*i_sq_ref

        return i_s_ref, tau_M

    def update(self, tau_M, u_s_ref, u_dc):
        """
        Field-weakening based on the unlimited reference voltage.

        Parameters
        ----------
        tau_M : float
            Limited torque reference.
        u_s_ref : complex
            Unlimited stator voltage reference.
        u_dc : DC-bus voltage.
            float.

        """
        u_s_max = self.k_u*u_dc/np.sqrt(3)
        self.i_sd_ref += self.T_s*self.k*(u_s_max - np.abs(u_s_ref))

        # Limit the current
        i_sd_mtpa = self.i_sd_mtpa(np.abs(tau_M))
        i_sd_lim = self.i_sd_lim(np.abs(tau_M))

        if self.i_sd_ref > i_sd_mtpa:
            self.i_sd_ref = i_sd_mtpa
        elif self.i_sd_ref < i_sd_lim:
            self.i_sd_ref = i_sd_lim

    def __str__(self):
        return self.desc


# %%
class SensorlessObserver:
    """
    A sensorless observer corresponding to the paper "Observers for sensorless
    synchronous motor drives: Framework for design and analysis":

        https://doi.org/10.1109/TIA.2018.2858753

    The observer gain decouples the electrical and mechanical dynamicas and
    allows placing the poles of the corresponding linearized estimation
    error dynamics.

    """

    def __init__(self, pars):
        """
        Parameters
        ----------
        pars : data object
            Controller parameters.

        """
        self.T_s = pars.T_s
        self.R_s = pars.R_s
        self.L_d = pars.L_d
        self.L_q = pars.L_q
        self.psi_f = pars.psi_f
        self.k_p = 2*pars.w_o
        self.k_i = pars.w_o**2
        self.b_p = .5*pars.R_s*(pars.L_d + pars.L_q)/(pars.L_d*pars.L_q)
        self.zeta_inf = .7
        # Initial states
        self.theta_m, self.w_m, self.psi_s = 0, 0, pars.psi_f
        self.desc = (('Sensorless observer:\n'
                      '    w_o=2*pi*{:.1f}\n')
                     .format(pars.w_o/(2*np.pi)))

    def update(self, u_s, i_s):
        """
        Update the states for the next sampling period.

        Parameters
        ----------
        u_s : complex
            Stator voltage in estimated rotor coordinates.
        i_s : complex
            Stator current in estimated rotor coordinates.

        """
        # Auxiliary flux (12)
        psi_a = self.psi_f + (self.L_d - self.L_q)*np.conj(i_s)

        # Estimation error (6)
        e = self.L_d*i_s.real + 1j*self.L_q*i_s.imag + self.psi_f - self.psi_s

        # Pole locations are chosen according to (36), with c = w_m**2
        # and w_inf = inf, and the gain corresponding to (30) is used
        k = self.b_p + 2*self.zeta_inf*np.abs(self.w_m)
        psi_a_sqr = np.abs(psi_a)**2
        if psi_a_sqr > 0:
            # Correction voltage
            v = k*psi_a*np.real(psi_a*np.conj(e))/psi_a_sqr
            # Error signal (10)
            eps = np.imag(psi_a*np.conj(e))/psi_a_sqr
        else:
            v, eps = 0, 0

        # Speed estimation (9)
        w_m = self.k_p*eps + self.w_m

        # Update the states
        self.psi_s += self.T_s*(u_s - self.R_s*i_s - 1j*w_m*self.psi_s + v)
        self.w_m += self.T_s*self.k_i*eps
        self.theta_m += self.T_s*w_m
        self.theta_m = np.mod(self.theta_m, 2*np.pi)    # Limit to [0, 2*pi]

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
        self.psi_s = []
        self.w_m_ref = []
        self.w_m = []
        self.theta_m = []
        self.u_dc = []
        self.tau_M = []
        self.u_ss, self.i_ss = 0j, 0j

    def save(self, data):
        """
        Saves the solution.

        Parameters
        ----------
        mdl : instance of a class
            Continuous-time model.

        """
        (i_s_ref, i_s, u_s, psi_s, w_m_ref, w_m, theta_m,
         u_dc, tau_M, T_s) = data
        try:
            t_new = self.t[-1] + T_s
        except IndexError:
            t_new = 0   # At the first step t = []
        self.t.extend([t_new])
        self.i_s_ref.extend([i_s_ref])
        self.i_s.extend([i_s])
        self.u_s.extend([u_s])
        self.psi_s.extend([psi_s])
        self.w_m_ref.extend([w_m_ref])
        self.w_m.extend([w_m])
        self.theta_m.extend([theta_m])
        self.u_dc.extend([u_dc])
        self.tau_M.extend([tau_M])

    def post_process(self):
        """
        Transforms the lists to the ndarray format and post-process them.

        """
        self.i_s_ref = np.asarray(self.i_s_ref)
        self.i_s = np.asarray(self.i_s)
        self.u_s = np.asarray(self.u_s)
        self.psi_s = np.asarray(self.psi_s)
        self.w_m_ref = np.asarray(self.w_m_ref)
        self.w_m = np.asarray(self.w_m)
        self.theta_m = np.asarray(self.theta_m)
        self.u_dc = np.asarray(self.u_dc)
        self.tau_M = np.asarray(self.tau_M)
        self.u_ss = np.exp(1j*self.theta_m)*self.u_s
        self.i_ss = np.exp(1j*self.theta_m)*self.i_s

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
        ax1.step(self.t, self.w_m/base.w, where='post')
        ax1.plot(data.t, data.w_m/base.w)
        ax1.legend([r'$\omega_\mathrm{m,ref}$',
                    r'$\hat \omega_\mathrm{m}$',
                    r'$\omega_\mathrm{m}$'])
        ax1.set_xlim(t_range)
        ax1.set_xticklabels([])
        ax1.set_ylabel('Speed (p.u.)')

        ax2.plot(data.t, data.tau_L/base.tau, '--')
        ax2.plot(data.t, data.tau_M/base.tau)
        ax2.step(self.t, self.tau_M/base.tau)  # Limited torque reference
        ax2.set_xlim(t_range)
        ax2.legend([r'$\tau_\mathrm{L}$', r'$\tau_\mathrm{M}$',
                    r'$\tau_\mathrm{M,ref}$'])
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

        ax5.plot(data.t, np.abs(data.psi_s)/base.psi)
        ax5.step(self.t, np.abs(self.psi_s)/base.psi, '--', where='post')
        ax5.set_xlim(t_range)
        ax5.legend([r'$\psi_\mathrm{s}$', r'$\hat\psi_\mathrm{s}$'])
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
        ax2.set_ylim(-.2, 1.5)
        ax2.legend([r'$\tau_\mathrm{L}$', r'$\tau_\mathrm{M}$'])
        ax2.set_ylabel('Torque (p.u.)')
        ax2.set_xticklabels([])

        ax3.step(self.t, self.i_s.real/base.i, where='post')
        ax3.step(self.t, self.i_s.imag/base.i, where='post')
        ax3.set_ylabel('Current (p.u.)')
        ax3.legend([r'$i_\mathrm{sd}$',  r'$i_\mathrm{sq}$'])
        ax3.set_xlim(t_range)
        ax3.set_ylim(-1.5, 1.5)
        ax3.set_xticklabels([])

        ax4.step(self.t, np.abs(self.u_s)/base.u, where='post')
        ax4.step(self.t, self.u_dc/np.sqrt(3)/base.u, '--', where='post')
        ax4.set_ylabel('Voltage (p.u.)')
        ax4.set_xlim(t_range)
        ax4.set_ylim(0, 1.2)
        ax4.legend([r'$u_\mathrm{s}$', r'$u_\mathrm{dc}/\sqrt{3}$'])
        ax4.set_xticklabels([])

        ax5.plot(data.t, np.abs(data.psi_s)/base.psi)
        ax5.step(self.t, np.abs(self.psi_s)/base.psi, '--', where='post')
        ax5.set_xlim(t_range)
        ax5.set_ylim(0, 1.2)
        ax5.legend([r'$\psi_\mathrm{s}$', r'$\hat\psi_\mathrm{s}$'])
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
            ax2.plot(data.t,
                     complex2abc(data.i_s*np.exp(1j*data.theta_m)).T/base.i)
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
