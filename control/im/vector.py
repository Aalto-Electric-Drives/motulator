# pylint: disable=C0103
"""
This module includes vector control methods for induction motor drives.

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
    This class interconnects the subsystems of an induction motor control
    system and provides the interface to the solver.

    """

    def __init__(self, pars, speed_ctrl, current_ref, current_ctrl, observer,
                 datalog):
        """
        Instantiate the classes.

        """
        self.p = pars.p
        self.current_ctrl = current_ctrl
        self.observer = observer
        self.speed_ctrl = speed_ctrl
        self.current_ref = current_ref
        self.pwm = PWM(pars)
        self.datalog = datalog

    def __call__(self, w_m_ref, w_M, _, i_s_abc, u_dc):
        """
        Main control loop.

        Parameters
        ----------
        w_m_ref : float
            Rotor speed reference (in electrical rad/s).
        w_M : float
            Rotor speed (in mechanical rad/s).
        _ : float
            Rotor angle (in mechanical rad), not used.
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
        psi_R = self.observer.psi_R
        theta_s = self.observer.theta_s

        # Space vector and coordinate transformation
        i_s = np.exp(-1j*theta_s)*abc2complex(i_s_abc)

        # Outputs
        tau_M_ref, tau_L = self.speed_ctrl.output(w_m_ref/self.p, w_M)
        i_s_ref, tau_M = self.current_ref.output(tau_M_ref, psi_R)
        w_s = self.observer.output(i_s, w_m)
        u_s_ref, e = self.current_ctrl.output(i_s_ref, i_s)
        d_abc_ref, u_s_ref_lim = self.pwm.output(u_s_ref, u_dc, theta_s, w_s)

        # Update the states
        self.pwm.update(u_s_ref_lim)
        self.speed_ctrl.update(tau_M, tau_L)
        self.current_ref.update(u_s_ref, u_dc)
        self.current_ctrl.update(e, u_s_ref, u_s_ref_lim, w_s)
        self.observer.update(i_s, w_s)

        # Data logging
        self.datalog.save([i_s_ref, i_s, u_s, w_m_ref, w_m, w_s, psi_R,
                           theta_s, u_dc, tau_M, self.pwm.T_s])

        return d_abc_ref, self.pwm.T_s

    def __str__(self):
        desc = 'Sensored vector control of an induction motor'
        return desc


# %%
class SensorlessVectorCtrl:
    """
    This class interconnects the subsystems of an induction motor control
    system and provides the interface to the solver.

    """

    def __init__(self, pars, speed_ctrl, current_ref, current_ctrl, observer,
                 datalog):
        """
        Instantiate the classes.

        """
        self.p = pars.p
        self.current_ctrl = current_ctrl
        self.observer = observer
        self.speed_ctrl = speed_ctrl
        self.current_ref = current_ref
        self.pwm = PWM(pars)
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
        # Get the states
        u_s = self.pwm.realized_voltage
        w_m = self.observer.w_m
        psi_R = self.observer.psi_R
        theta_s = self.observer.theta_s

        # Space vector and coordinate transformation
        i_s = np.exp(-1j*theta_s)*abc2complex(i_s_abc)

        # Outputs
        tau_M_ref, tau_L = self.speed_ctrl.output(w_m_ref/self.p, w_m/self.p)
        i_s_ref, tau_M = self.current_ref.output(tau_M_ref, psi_R)
        w_s, w_r, dpsi_R = self.observer.output(u_s, i_s)
        u_s_ref = self.current_ctrl.output(i_s_ref, i_s, psi_R, w_s, w_m)
        d_abc_ref, u_s_ref_lim = self.pwm.output(u_s_ref, u_dc, theta_s, w_s)

        # Update the states
        self.pwm.update(u_s_ref_lim)
        self.speed_ctrl.update(tau_M, tau_L)
        self.current_ref.update(u_s_ref, u_dc)
        self.current_ctrl.update(i_s_ref, i_s, u_s_ref, u_s_ref_lim)
        self.observer.update(i_s, w_s, w_r, dpsi_R)

        # Data logging
        self.datalog.save([i_s_ref, i_s, u_s, w_m_ref, w_m, w_s, psi_R,
                           theta_s, u_dc, tau_M, self.pwm.T_s])

        return d_abc_ref, self.pwm.T_s

    def __str__(self):
        desc = 'Sensorless vector control of an induction motor'
        return desc


# %%
class CurrentRef:
    """
    This class contains a method for current reference computation with
    field weakening based on the voltage reference. The field-weakening method
    and its tuning corresponds roughly to the paper "Braking scheme for
    vector-controlled induction motor drives equipped with diode rectifier
    without braking resistor":

        https://doi.org/10.1109/TIA.2006.880852

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
        self.L_sgm = pars.L_sgm
        self.p = pars.p
        # Local parameters for initializing the constants
        psi_R_nom = pars.psi_R_nom
        L_M = pars.L_M
        R_R = pars.R_R
        u_dc_nom = pars.u_dc_nom
        # Nominal d-axis current
        self.i_sd_nom = psi_R_nom/L_M
        # Torque gain
        self.gain_tq = 1/(1.5*pars.p*psi_R_nom)
        # Field weakening
        self.gain_fw = 3*R_R*psi_R_nom/(pars.L_sgm*u_dc_nom)**2
        # State variable
        self.i_sd_ref = self.i_sd_nom

    def output(self, tau_M_ref, psi_R):
        """
        Compute the stator current reference.

        Parameters
        ----------
        tau_M_ref : float
            Torque reference.
        psi_R : float
            Rotor flux magnitude.

        Returns
        -------
        i_s_ref : complex
            Stator current reference.
        tau_M : float
            Limited torque reference.

        """
        def q_axis_current_limit(i_sd_ref, psi_R):
            # Priority given to the d component
            i_sq_max1 = np.sqrt(self.i_s_max**2 - i_sd_ref**2)
            # Breakdown torque limit
            i_sq_max2 = psi_R/self.L_sgm + i_sd_ref
            # q-axis current limit
            i_sq_max = np.min([i_sq_max1, i_sq_max2])
            return i_sq_max

        # To improve the robustness, we keep the torque gain constant
        i_sq_ref = self.gain_tq*tau_M_ref
        # Limit the current
        i_sq_max = q_axis_current_limit(self.i_sd_ref, psi_R)
        if np.abs(i_sq_ref) > i_sq_max:
            i_sq_ref = np.sign(i_sq_ref)*i_sq_max
        # Current reference
        i_s_ref = self.i_sd_ref + 1j*i_sq_ref
        # Limited torque (for the speed controller)
        tau_M = 1.5*self.p*psi_R*i_sq_ref
        return i_s_ref, tau_M

    def update(self, u_s_ref, u_dc):
        """
        Field-weakening based on the unlimited reference voltage.

        Parameters
        ----------
        u_s_ref : complex
            Unlimited stator voltage reference.
        u_dc : DC-bus voltage.
            float.

        """
        u_s_max = u_dc/np.sqrt(3)
        self.i_sd_ref += self.T_s*self.gain_fw*(u_s_max**2
                                                - np.abs(u_s_ref)**2)
        if self.i_sd_ref > self.i_sd_nom:
            self.i_sd_ref = self.i_sd_nom
        elif self.i_sd_ref < -self.i_s_max:
            self.i_sd_ref = -self.i_s_max

    def __str__(self):
        desc = ('Current reference computation and field weakening:\n'
                '    i_s_max={:.1f}  i_sd_nom={:.1f}')
        return desc.format(self.i_s_max, self.i_sd_nom)


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
        self.L_sgm = pars.L_sgm
        self.alpha = pars.R_R/pars.L_M
        self.R_sgm = pars.R_s + pars.R_R

    def output(self, i_s_ref, i_s, psi_R, w_s, w_m):
        """
        State-feedback current controller.

        Parameters
        ----------
        i_s_ref : complex
            Stator current reference.
        i_s : complex
            Stator current.
        psi_R : complex
            Rotor flux (or its estimate).
        w_s : float
            Angular frequency of the reference frame.
        w_m : float
            Rotor speed (in electrical rad/s).

        Returns
        -------
        u_s_ref : complex
            Voltage reference.

        """
        # pylint: disable=R0913
        # State feedback gains
        k_1 = (self.alpha_c - 1j*w_s)*self.L_sgm - self.R_sgm
        k_2 = self.alpha - 1j*w_m
        # Reference feedforward gain
        k_t = self.R_sgm + 1j*w_s*self.L_sgm + k_1
        # Control law
        u_s_ref = k_t*i_s_ref - k_1*i_s - k_2*psi_R
        return u_s_ref

    def update(self, *_):
        """
        No states, nothing to update. This method is just for compatibility.

        """

    def __str__(self):
        desc = ('State-feedback current control (without integral action):\n'
                '    alpha_c=2*pi*{:.1f}')
        return desc.format(self.alpha_c/(2*np.pi))


# %%
class CurrentCtrl2DOFPI:
    """
    A current controller corresponding to the paper "Flux-linkage-based current
    control of saturated synchronous motors":

        https://doi.org/10.1109/TIA.2019.291925

    The continuous-time complex-vector design corresponding to (13) is used
    here. This design could be equivalently presented as a 2DOF PI controller.
    For better performance at high speed with low sampling frequencies, the
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
        self.L_sgm = pars.L_sgm
        self.alpha_c = pars.alpha_c
        # Integral state
        self.u_i = 0

    def output(self, i_s_ref, i_s):
        """
        Computes the unlimited voltage reference.

        Parameters
        ----------
        i_s_ref : complex
            Stator current reference.
        i_s : complex
            Measured stator current.

        Returns
        -------
        u_s_ref : complex
            Unlimited voltage reference.

        """
        psi_sgm_ref = self.L_sgm*i_s_ref
        psi_sgm = self.L_sgm*i_s
        k_t = self.alpha_c
        k = 2*self.alpha_c
        u_s_ref = k_t*psi_sgm_ref - k*psi_sgm + self.u_i
        e = psi_sgm_ref - psi_sgm
        return u_s_ref, e

    def update(self, e, u_s_ref, u_s_ref_lim, w_s):
        """
        Updates the integral state.

        Parameters
        ----------
        e : complex
            Error (scaled, corresponds to the leakage flux linkage).
        u_s_ref : complex
            Unlimited voltage reference.
        u_s_ref_lim : complex
            Limited voltage reference.
        w_s : float
            Angular stator frequency.

        """
        k_i = self.alpha_c*(self.alpha_c + 1j*w_s)
        k_t = self.alpha_c
        self.u_i += self.T_s*k_i*(e + (u_s_ref_lim - u_s_ref)/k_t)

    def __str__(self):
        desc = ('2DOF PI current control:\n'
                '    alpha_c=2*pi*{:.1f}')
        return desc.format(self.alpha_c/(2*np.pi))


# %%
class SensorlessObserver:
    """
    Sensorless reduced-order observer corresponding to the paper
    "Reduced-order flux observers with stator-resistance adaptation for
    speed-sensorlessinduction motor drives":

        https://doi.org/10.1109/TPEL.2009.2039650

    This implementation corresponds to (26)-(30) with the fixed selection
    c = w_s**2 in (17). This selection allows to avoid the algebraic loop
    in (26b). The closed-loop poles, cf. (40), can still be affected via the
    choice of the coefficient b > 0.

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
        self.R_R = pars.R_R
        self.L_sgm = pars.L_sgm
        self.alpha = pars.R_R/pars.L_M
        self.alpha_o = pars.alpha_o
        self.zeta_inf = .7
        # Initial states
        self.theta_s, self.psi_R, self.i_s_old, self.w_m = 0, 0, 0, 0

    def output(self, u_s, i_s):
        """
        Computes the outputs of the observer.

        Parameters
        ----------
        u_s : complex
            Stator voltage in estimated rotor flux coordinates.
        i_s : complex
            Stator current in estimated rotor flux coordinates.

        Returns
        -------
        w_s : float
            Angular frequency of the rotor flux.
        w_r : float
            Angular slip frequency.
        dpsi_R : float
            Increment of the flux magnitude for the state update.

        """
        # Observer gain (17) with c = w_s**2 (without the orthogonal projection
        # which is embedded into the state update)
        b = 2*self.zeta_inf*np.abs(self.w_m) + self.alpha
        g = b*(self.alpha + 1j*self.w_m)/(self.alpha**2 + self.w_m**2)

        # Auxiliary variable: e = e_s + 1j*w_s*L_sgm*i_s
        e = u_s - self.R_s*i_s - self.L_sgm*(i_s - self.i_s_old)/self.T_s
        # Induced voltage (8) from the rotor quantities
        e_r = self.R_R*i_s - (self.alpha - 1j*self.w_m)*self.psi_R

        # Angular frequency of the rotor flux vector
        den = self.psi_R + self.L_sgm*(i_s.real + g.imag*i_s.imag)
        if den > 0:
            w_s = (e.imag + g.imag*(e_r - e).real)/den
        else:
            w_s = self.w_m
        # Induced voltage (7) from the stator quantities
        e_s = e - 1j*w_s*self.L_sgm*i_s

        # Slip angular frequency
        if self.psi_R > 0:
            w_r = e_r.imag/self.psi_R
        else:
            w_r = 0

        # Increment of the flux magnitude
        dpsi_R = e_s.real + g.real*(e_r - e_s).real

        return w_s, w_r, dpsi_R

    def update(self, i_s, w_s, w_r, d_psi_R):
        """
        Update the states for the next sampling period.

        """
        self.w_m += self.T_s*self.alpha_o*(w_s - w_r)
        self.psi_R += self.T_s*d_psi_R
        self.theta_s += self.T_s*w_s
        self.theta_s = np.mod(self.theta_s, 2*np.pi)    # Limit to [0, 2*pi]
        self.i_s_old = i_s

    def __str__(self):
        desc = ('Sensorless reduced-order observer:\n'
                '    alpha_o=2*pi*{:.1f}')
        return desc.format(self.alpha_o/(2*np.pi))


# %%
class CurrentModelEstimator:
    """
    This class contains a simple sensored flux estimator, commonly known as
    the current model.

    """

    def __init__(self, pars):
        """
        Parameters
        ----------
        pars : data object
            Controller parameters.

        """
        # Parameters
        self.T_s = pars.T_s
        self.R_R = pars.R_R
        self.L_M = pars.L_M
        # Initialize the states
        self.theta_s, self.psi_R = 0, 0

    def output(self, i_s, w_m):
        """
        Computes the outputs of the observer.

        Parameters
        ----------
        i_s : complex
            Stator current.
        w_m : float
            Rotor speed (in electrical rad/s).

        Returns
        -------
        w_s : float
            Angular frequency of the rotor flux.

        """
        if self.psi_R > 0:
            w_s = w_m + self.R_R*i_s.imag/self.psi_R
        else:
            w_s = w_m
        return w_s

    def update(self, i_s, w_s):
        """
        Update the states for the next sampling period.

        Parameters
        ----------
        i_s : complex
            Stator current.
        w_s : float
            Angular frequency of the rotor flux.

        """
        self.psi_R += self.T_s*self.R_R*(i_s.real - self.psi_R/self.L_M)
        self.theta_s += self.T_s*w_s
        self.theta_s = np.mod(self.theta_s, 2*np.pi)    # Limit to [0, 2*pi]

    def __str__(self):
        desc = ('Current model flux estimator')
        return desc


# %%
class Datalogger:
    """
    This class contains a default data logger.

    """

    def __init__(self):
        """
        Initialize the attributes.

        """
        # pylint: disable=too-many-instance-attributes
        self.t = []
        self.i_s_ref = []
        self.i_s = []
        self.u_s = []
        self.w_m_ref = []
        self.w_m = []
        self.w_s = []
        self.psi_R = []
        self.theta_s = []
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
        (i_s_ref, i_s, u_s, w_m_ref, w_m, w_s, psi_R,
         theta_s, u_dc, tau_M, T_s) = data
        try:
            t_new = self.t[-1] + T_s
        except IndexError:
            t_new = 0   # At the first step t = []
        self.t.extend([t_new])
        self.i_s_ref.extend([i_s_ref])
        self.i_s.extend([i_s])
        self.u_s.extend([u_s])
        self.w_m_ref.extend([w_m_ref])
        self.w_m.extend([w_m])
        self.w_s.extend([w_s])
        self.psi_R.extend([psi_R])
        self.theta_s.extend([theta_s])
        self.u_dc.extend([u_dc])
        self.tau_M.extend([tau_M])

    def post_process(self):
        """
        Transforms the lists to the ndarray format and post-process them.

        """
        # From lists to ndarrays
        self.i_s_ref = np.asarray(self.i_s_ref)
        self.i_s = np.asarray(self.i_s)
        self.u_s = np.asarray(self.u_s)
        self.w_m_ref = np.asarray(self.w_m_ref)
        self.w_m = np.asarray(self.w_m)
        self.w_s = np.asarray(self.w_s)
        self.psi_R = np.asarray(self.psi_R)
        self.theta_s = np.asarray(self.theta_s)
        self.u_dc = np.asarray(self.u_dc)
        self.tau_M = np.asarray(self.tau_M)
        # Additional variables
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
