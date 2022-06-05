# pylint: disable=C0103
"""
This module contains vector control for synchronous motor drives.

"""

# %%
import numpy as np
from sklearn.utils import Bunch
from helpers import abc2complex
from control.common import PWM, Datalogger


# %%
class VectorCtrl:
    """
    Interconnect the subsystems of the control method.

    This class interconnects the subsystems of the control system and
    provides the interface to the solver.

    """

    def __init__(self, pars, speed_ctrl, current_ref, current_ctrl, observer):
        self.p = pars.p
        self.sensorless = pars.sensorless
        self.current_ctrl = current_ctrl
        self.speed_ctrl = speed_ctrl
        self.current_ref = current_ref
        self.observer = observer
        self.pwm = PWM(pars)
        self.datalog = Datalogger()

    def __call__(self, w_m_ref, i_s_abc, u_dc, *args):
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
        w_M : float, optional
            Rotor speed (in mechanical rad/s), for the sensored control.
        theta_M : float, optional
            Rotor angle (in mechanical rad), for the sensored control.

        Returns
        -------
        d_abc_ref : ndarray, shape (3,)
            Duty ratio references.
        T_s : float
            Sampling period.

        """
        # Get the states
        u_s = self.pwm.realized_voltage
        if self.sensorless:
            w_m, theta_m = self.observer.w_m, self.observer.theta_m
            i_s = np.exp(-1j*theta_m)*abc2complex(i_s_abc)
            # Needed only for the current controller without integral action
            psi_s = self.observer.psi_s
        else:
            w_m = args[0]
            theta_m = np.mod(args[1], 2*np.pi)
            i_s = np.exp(-1j*theta_m)*abc2complex(i_s_abc)
            psi_s = np.nan
            # psi_s = self.L_d*i_s.real + self.psi_f + 1j*self.L_q*i_s.imag

        # Outputs
        tau_M_ref, tau_L = self.speed_ctrl.output(w_m_ref/self.p, w_m/self.p)
        i_s_ref, tau_M = self.current_ref.output(tau_M_ref, w_m, u_dc)
        u_s_ref, e = self.current_ctrl.output(i_s_ref, i_s)
        d_abc_ref, u_s_ref_lim = self.pwm.output(u_s_ref, u_dc, theta_m, w_m)

        # Update all the states
        if self.sensorless:
            self.observer.update(u_s, i_s)
        self.speed_ctrl.update(tau_M, tau_L)
        self.current_ref.update(tau_M, u_s_ref, u_dc)
        self.current_ctrl.update(e, u_s_ref, u_s_ref_lim, w_m)
        self.pwm.update(u_s_ref_lim)

        # Data logging
        data = Bunch(i_s_ref=i_s_ref, i_s=i_s, u_s=u_s, psi_s=psi_s,
                     w_m_ref=w_m_ref, w_m=w_m, theta_m=theta_m,
                     u_dc=u_dc, tau_M=tau_M, T_s=self.pwm.T_s)
        self.datalog.save(data)

        return d_abc_ref, self.pwm.T_s

    def __str__(self):
        desc = (('Vector control for a synchronous motor\n'
                 '-------------------------------------\n'
                 'Sensorless:\n'
                 '    {}\n'
                 'Sampling period:\n'
                 '    T_s={}\n'
                 'Number of pole pairs:\n'
                 '    p={}\n')
                .format(self.sensorless, self.pwm.T_s, self.p))
        if self.observer:
            desc += (self.current_ref.__str__() + self.current_ctrl.__str__()
                     + self.speed_ctrl.__str__() + self.observer.__str__())
        else:
            desc += (self.current_ref.__str__() + self.current_ctrl.__str__()
                     + self.speed_ctrl.__str__())
        return desc


# %%
class CurrentCtrl:
    """
    2DOF PI current controller.

    This 2DOF PI current controller corresponds to [1]_. The continuous-time
    complex-vector design corresponding to (13) is used here. This design could
    be equivalently presented as a 2DOF PI controller.

    Notes
    -----
    For better performance at high speeds with low sampling frequencies, the
    discrete-time design in (18) is recommended. This implementation does not
    take the magnetic saturation into account.

    References
    ----------
    .. [1] Awan, Saarakkala, Hinkkanen, "Flux-linkage-based current control of
       saturated synchronous motors," IEEE Trans. Ind. Appl. 2019,
       https://doi.org/10.1109/TIA.2019.2919258

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

    def output(self, i_s_ref, i_s):
        """
        Compute the unlimited voltage reference.

        Parameters
        ----------
        i_s_ref : complex
            Current reference.
        i_s : complex
            Measured current.

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
        Update the integral state.

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
        desc = (('2DOF PI current control:\n'
                 '    alpha_c=2*pi*{:.1f}\n')
                .format(self.alpha_c/(2*np.pi)))
        return desc


# %%
class CurrentRef:
    """
    Current reference calculation.

    This current reference calculation method includes the MTPA locus and
    field-weakenting operation based on the unlimited voltage reference
    feedback. The MTPV and current limits are taken into account. This
    resembles the method presented [2]_.

    Notes
    -----
    Instead of the PI controller used in [2]_, we use a simpler integral
    controller with a constant gain. The resulting operating-point-dependent
    closed-loop pole could be derived using (12) of the paper. Unlike in [2]_,
    the MTPV limit is also included here by means of limiting the reference
    torque and the d-axis current reference.

    References
    ----------
    .. [2] Bedetti, Calligaro, Petrella, "Analytical design and autotuning of
       adaptive flux-weakening voltage regulation loop in IPMSM drives with
       accurate torque regulation," IEEE Trans. Ind. Appl., 2020,
       https://doi.org/10.1109/TIA.2019.2942807

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
        desc = ('Current reference calculation:\n'
                '    i_s_max={:.1f}\n').format(self.i_s_max)
        return desc


# %%
class SensorlessObserver:
    """
    Sensorless observer.

    This sensorless observer corresponds to [3]_. The observer gain decouples
    the electrical and mechanical dynamics and allows placing the poles of the
    corresponding linearized estimation error dynamics. This implementation
    operates in estimated rotor coordinates.

    References
    ----------
    .. [3] Hinkkanen, Saarakkala, Awan, Mölsä, Tuovinen, "Observers for
       sensorless synchronous motor drives: Framework for design and analysis,"
       IEEE Trans. Ind. Appl., 2018, https://doi.org/10.1109/TIA.2018.2858753

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
        desc = (('Sensorless observer:\n'
                 '    w_o=2*pi*{:.1f}\n'
                 'Motor parameter estimates:\n'
                 '    R_s={}  L_d={}  L_q={}  psi_f={}\n')
                .format(.25*self.k_p/np.pi, self.R_s,
                        self.L_d, self.L_q, self.psi_f))
        return desc
