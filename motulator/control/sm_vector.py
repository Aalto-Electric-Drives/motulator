# pylint: disable=C0103
"""
This module contains vector control for synchronous motor drives.

"""
from __future__ import annotations
from collections.abc import Callable
from dataclasses import dataclass, field
import numpy as np

from motulator.helpers import abc2complex, Bunch
from motulator.control.common import SpeedCtrl, PWM, Datalogger, Delay
from motulator.control.sm_torque import TorqueCharacteristics


# %%
@dataclass
class SynchronousMotorVectorCtrlPars:
    """
    Control system parameters for synchronous motors.

    """
    # pylint: disable=too-many-instance-attributes
    # Speed reference (in electrical rad/s)
    w_m_ref: Callable[[float], float] = field(
        repr=False, default=lambda t: (t > .2)*(2*np.pi*75))
    # Mode
    sensorless: bool = True
    # Sampling period
    T_s: float = 250e-6
    delay: int = 1
    # Bandwidths
    alpha_c: float = 2*np.pi*200
    alpha_fw: float = 2*np.pi*20
    alpha_s: float = 2*np.pi*4
    # Maximum values
    tau_M_max: float = 2*14.6
    i_s_max: float = 1.5*np.sqrt(2)*5
    i_sd_min: float = None
    # Voltage margin
    k_u: float = .95
    # Nominal values
    w_nom: float = 2*np.pi*75
    # Motor parameter estimates
    R_s: float = 3.6
    L_d: float = .036
    L_q: float = .051
    psi_f: float = .545
    p: int = 3
    J: float = .015
    # Sensorless observer
    w_o: float = 2*np.pi*40  # Used only in the sensorless mode

    def __post_init__(self):
        """
        Generate control look-up tables.

        """
        # Generate LUTs
        tq = TorqueCharacteristics(self)
        mtpa = tq.mtpa_locus(i_s_max=self.i_s_max)
        lims = tq.mtpv_and_current_limits(i_s_max=self.i_s_max)
        # MTPA locus
        self.i_sd_mtpa = mtpa.i_sd_vs_tau_M
        # Merged MTPV and current limits
        self.tau_M_lim = lims.tau_M_vs_abs_psi_s
        self.i_sd_lim = lims.i_sd_vs_tau_M

    def plot(self, base):
        """
        Plot control look-up tables.

        Parameters
        ----------
        base : BaseValues
            Base values for scaling the plots.

        """
        tq = TorqueCharacteristics(self)
        tq.plot_current_loci(self.i_s_max, base)
        tq.plot_torque_flux(self.i_s_max, base)
        tq.plot_torque_current(self.i_s_max, base)
        # tq.plot_flux_loci(self.i_s_max, base)


class SynchronousMotorVectorCtrl(Datalogger):
    """
    Vector control for a synchronous motor drive.

    This class interconnects the subsystems of the control system and
    provides the interface to the solver.

    """

    def __init__(self, pars=SynchronousMotorVectorCtrlPars()):
        """
        Parameters
        ----------
        pars : SynchronousMotorVectorCtrlData
            Control parameters.

        """
        super().__init__()
        self.t = 0
        self.T_s = pars.T_s
        self.w_m_ref = pars.w_m_ref
        self.p = pars.p
        self.sensorless = pars.sensorless
        self.current_ctrl = CurrentCtrl(pars)
        self.speed_ctrl = SpeedCtrl(pars)
        self.current_ref = CurrentRef(pars)
        self.observer = SensorlessObserver(pars)
        self.pwm = PWM(pars)
        self.delay = Delay(pars.delay)
        if pars.sensorless:
            self.observer = SensorlessObserver(pars)
        else:
            self.observer = None
        self.desc = pars.__repr__()

    def __call__(self, mdl):
        """
        Main control loop.

        Parameters
        ----------
        mdl : SynchronousMotorDrive
            Continuous-time model of a synchronous motor drive for getting the
            feedback signals.

        Returns
        -------
        T_s : float
            Sampling period.
        d_abc_ref : ndarray, shape (3,)
            Duty ratio references.

        """
        # Get the speed reference
        w_m_ref = self.w_m_ref(self.t)

        # Measure the feedback signals
        i_s_abc = mdl.motor.meas_currents()  # Phase currents
        u_dc = mdl.conv.meas_dc_voltage()  # DC-bus voltage

        if not self.sensorless:
            # Measure the rotor speed and position
            w_m = self.p*mdl.mech.meas_speed()
            theta_m = self.p*np.mod(mdl.mech.meas_position(), 2*np.pi)
            # psi_s = self.L_d*i_s.real + self.psi_f + 1j*self.L_q*i_s.imag
            psi_s = np.nan  # Flux not estimated
        else:
            # Get the rotor speed and position estimates
            w_m, theta_m = self.observer.w_m, self.observer.theta_m
            # Get the flux estimate (not used)
            psi_s = self.observer.psi_s

        # Get the realized voltage from the PWM method
        u_s = self.pwm.realized_voltage

        # Current vector in estimated rotor coordinates
        i_s = np.exp(-1j*theta_m)*abc2complex(i_s_abc)

        # Outputs
        tau_M_ref, tau_L = self.speed_ctrl.output(w_m_ref/self.p, w_m/self.p)
        i_s_ref, tau_M = self.current_ref.output(tau_M_ref, w_m, u_dc)
        u_s_ref, e = self.current_ctrl.output(i_s_ref, i_s)
        d_abc_ref, u_s_ref_lim = self.pwm.output(u_s_ref, u_dc, theta_m, w_m)

        # Data logging
        data = Bunch(i_s_ref=i_s_ref, i_s=i_s, u_s=u_s, psi_s=psi_s,
                     w_m_ref=w_m_ref, w_m=w_m, theta_m=theta_m,
                     u_dc=u_dc, tau_M=tau_M, t=self.t)
        self.save(data)

        # Update states
        if self.sensorless:
            self.observer.update(u_s, i_s)
        self.speed_ctrl.update(tau_M, tau_L)
        self.current_ref.update(tau_M, u_s_ref, u_dc)
        self.current_ctrl.update(e, u_s_ref, u_s_ref_lim, w_m)
        self.pwm.update(u_s_ref_lim)
        self.t += self.T_s

        return self.T_s, d_abc_ref

    def __repr__(self):
        return self.desc


# %%
class CurrentCtrl:
    """
    2DOF PI current controller.

    This controller corresponds to [1]_. The continuous-time complex-vector
    design corresponding to (13) is used here. This design could be
    equivalently presented as a 2DOF PI controller.

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
        pars : SynchronousMotorVectorCtrlPars (or its subset)
            Control parameters.

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


# %%
class CurrentRef:
    """
    Current reference calculation.

    This method includes the MTPA locus and field-weakenting operation based on
    the unlimited voltage reference feedback. The MTPV and current limits are
    taken into account. This resembles the method presented [2]_.

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
        pars : SynchronousMotorVectorCtrlPars (or its subset)
            Control parameters.

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


# %%
class SensorlessObserver:
    """
    Sensorless observer.

    This observer corresponds to [3]_. The observer gain decouples the
    electrical and mechanical dynamics and allows placing the poles of the
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
        pars : SynchronousMotorVectorCtrlPars (or its subset)
            Control parameters.

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
