# pylint: disable=C0103
"""
This module contains vector control methods for an induction motor drive.

"""
from __future__ import annotations
from collections.abc import Callable
from dataclasses import dataclass, field
import numpy as np
from sklearn.utils import Bunch
from helpers import abc2complex
from control.common import SpeedCtrl, PWM, Delay, Datalogger


# %%
@dataclass
class InductionMotorVectorCtrlPars:
    """
    Vector control parameters for an induction motor drive.

    """
    # pylint: disable=too-many-instance-attributes
    # Speed reference (in electrical rad/s)
    w_m_ref: Callable[[float], float] = field(
        repr=False, default=lambda t: (t > .2)*(2*np.pi*50))
    # Mode
    sensorless: bool = True
    # Sampling period
    T_s: float = 250e-6
    delay: int = 1
    # Bandwidths
    alpha_c: float = 2*np.pi*200
    alpha_o: float = 2*np.pi*40  # Used only in the sensorless mode
    alpha_s: float = 2*np.pi*4
    # Maximum values
    tau_M_max: float = 1.5*14.6
    i_s_max: float = 1.5*np.sqrt(2)*5
    # Nominal values
    psi_R_nom: float = .9
    u_dc_nom: float = 540
    # Motor parameter estimates
    R_s: float = 3.7
    R_R: float = 2.1
    L_sgm: float = .021
    L_M: float = .224
    p: int = 2
    J: float = .015


class InductionMotorVectorCtrl(Datalogger):
    """
    Vector control for an induction motor drive.

    This class interconnects the subsystems of the control system and
    provides the interface to the solver.

    Parameters
    ----------
    pars : InductionMotorVectorControlPars
        Control parameters.

    """

    def __init__(self, pars=InductionMotorVectorCtrlPars()):
        super().__init__()
        self.t = 0
        self.T_s = pars.T_s
        self.w_m_ref = pars.w_m_ref
        self.sensorless = pars.sensorless
        self.p = pars.p
        self.speed_ctrl = SpeedCtrl(pars)
        self.current_ref = CurrentRef(pars)
        self.current_ctrl = CurrentCtrl(pars)
        if self.sensorless:
            self.observer = SensorlessObserver(pars)
        else:
            self.observer = CurrentModelEstimator(pars)
        self.pwm = PWM(pars)
        self.delay = Delay(pars.delay)
        self.desc = pars.__repr__()

    def __call__(self, i_s_abc, u_dc, *args):
        """
        Main control loop.

        Parameters
        ----------
        i_s_abc : ndarray, shape (3,)
            Phase currents.
        u_dc : float
            DC-bus voltage.
        w_M : float, optional
            Rotor speed (in mechanical rad/s), only for the sensored control.

        Returns
        -------
        d_abc_ref : ndarray, shape (3,)
            Duty ratio references.
        T_s : float
            Sampling period.

        """
        # Speed reference
        w_m_ref = self.w_m_ref(self.t)

        # States
        u_s = self.pwm.realized_voltage
        psi_R = self.observer.psi_R
        theta_s = self.observer.theta_s
        if self.sensorless:
            w_m = self.observer.w_m
        else:
            w_m = self.p*args[0]

        # Space vector and coordinate transformation
        i_s = np.exp(-1j*theta_s)*abc2complex(i_s_abc)

        # Outputs
        tau_M_ref, tau_L = self.speed_ctrl.output(w_m_ref/self.p, w_m/self.p)
        i_s_ref, tau_M = self.current_ref.output(tau_M_ref, psi_R)
        if self.sensorless:
            w_s = self.observer(u_s, i_s)
        else:
            w_s = self.observer(i_s, w_m)
        u_s_ref, e = self.current_ctrl.output(i_s_ref, i_s)
        d_abc_ref, u_s_ref_lim = self.pwm.output(u_s_ref, u_dc, theta_s, w_s)

        # Data logging
        data = Bunch(i_s_ref=i_s_ref, i_s=i_s, u_s=u_s, w_m_ref=w_m_ref,
                     w_m=w_m, w_s=w_s, psi_R=psi_R, theta_s=theta_s,
                     u_dc=u_dc, tau_M=tau_M, t=self.t)
        self.save(data)

        # Update the states
        self.pwm.update(u_s_ref_lim)
        self.speed_ctrl.update(tau_M, tau_L)
        self.current_ref.update(u_s_ref, u_dc)
        self.current_ctrl.update(e, u_s_ref, u_s_ref_lim, w_s)
        self.t += self.T_s

        return d_abc_ref, self.T_s

    def __repr__(self):
        return self.desc


# %%
class CurrentRef:
    """
    Current reference calculation.

    This method includes field-weakenting operation based on the unlimited
    voltage reference feedback. The breakdown torque and current limits are
    taken into account.

    Notes
    -----
    The field-weakening method and its tuning corresponds roughly to [1]_.

    References
    ----------
    .. [1] Hinkkanen, Luomi, "Braking scheme for vector-controlled induction
       motor drives equipped with diode rectifier without braking resistor,"
       IEEE Trans. Ind. Appl., 2006, https://doi.org/10.1109/TIA.2006.880852

    """

    def __init__(self, pars):
        """
        Parameters
        ----------
        pars : InductionMotorVectorCtrlPars (or its subset)
            Control parameters.

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
        u_dc : float.
            DC-bus voltage.

        """
        u_s_max = u_dc/np.sqrt(3)
        self.i_sd_ref += self.T_s*self.gain_fw*(u_s_max**2
                                                - np.abs(u_s_ref)**2)
        if self.i_sd_ref > self.i_sd_nom:
            self.i_sd_ref = self.i_sd_nom
        elif self.i_sd_ref < -self.i_s_max:
            self.i_sd_ref = -self.i_s_max


# %%
class CurrentCtrl:
    """
    2DOF PI current controller.

    This controller corresponds to [2]_. The continuous-time complex-vector
    design corresponding to (13) is used here. The rotor flux linkage is
    considered as a quasi-constant disturbance. This design could be
    equivalently presented as a 2DOF PI controller.

    Notes
    -----
    This implementation does not take the magnetic saturation into account.

    References
    ----------
    .. [2] Awan, Saarakkala, Hinkkanen, "Flux-linkage-based current control of
       saturated synchronous motors," IEEE Trans. Ind. Appl. 2019,
       https://doi.org/10.1109/TIA.2019.2919258


    """

    def __init__(self, pars):
        """
        Parameters
        ----------
        pars : InductionMotorVectorCtrlPars (or its subset)
            Control parameters.

        """
        self.T_s = pars.T_s
        self.L_sgm = pars.L_sgm
        self.alpha_c = pars.alpha_c
        self.u_i = 0  # Integral state

    def output(self, i_s_ref, i_s):
        """
        Compute the unlimited voltage reference.

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
        e : complex
            Error (scaled, corresponds to the leakage flux linkage).

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
        Update the integral state.

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


# %%
class SensorlessObserver:
    """
    Sensorless reduced-order observer.

    This observer corresponds to [3]_. The observer gain decouples the
    electrical and mechanical dynamics and allows placing the poles of the
    corresponding linearized estimation error dynamics. This implementation
    operates in estimated rotor flux coordinates.

    Notes
    -----
    This implementation corresponds to (26)-(30) in [3]_ with the fixed
    selection c = w_s**2 in (17). The closed-loop poles, cf. (40), can still be
    affected via the choice of the coefficient b > 0.

    References
    ----------
    .. [3] Hinkkanen, Harnefors, Luomi, "Reduced-order flux observers with
       stator-resistance adaptation for speed-sensorless induction motor
       drives," IEEE Trans. Power Electron., 2010,
       https://doi.org/10.1109/TPEL.2009.2039650

    """

    def __init__(self, pars):
        """
        Parameters
        ----------
        pars : InductionMotorVectorCtrlPars (or its subset)
            Control parameters.

        """
        self.T_s = pars.T_s
        self.R_s = pars.R_s
        self.R_R = pars.R_R
        self.L_sgm = pars.L_sgm
        self.L_M = pars.L_M
        self.alpha_o = pars.alpha_o
        self.zeta_inf = .7
        # Initial states
        self.theta_s, self.psi_R, self.i_s_old, self.w_m = 0, 0, 0, 0

    def __call__(self, u_s, i_s):
        """
        Output and update the sensorless observer.

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

        """
        w_s, w_r, dpsi_R = self.output(u_s, i_s)
        self.update(i_s, w_s, w_r, dpsi_R)

        return w_s

    def output(self, u_s, i_s):
        """
        Compute the outputs of the observer.

        """
        alpha = self.R_R/self.L_M

        # Observer gain (17) with c = w_s**2 (without the orthogonal projection
        # which is embedded into the state update)
        b = 2*self.zeta_inf*np.abs(self.w_m) + alpha
        g = b*(alpha + 1j*self.w_m)/(alpha**2 + self.w_m**2)

        # Auxiliary variable: e = e_s + 1j*w_s*L_sgm*i_s
        e = u_s - self.R_s*i_s - self.L_sgm*(i_s - self.i_s_old)/self.T_s
        # Induced voltage (8) from the rotor quantities
        e_r = self.R_R*i_s - (alpha - 1j*self.w_m)*self.psi_R

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

    def update(self, i_s, w_s, w_r, dpsi_R):
        """
        Update the states for the next sampling period.

        """
        self.w_m += self.T_s*self.alpha_o*(w_s - w_r)
        self.psi_R += self.T_s*dpsi_R
        self.theta_s += self.T_s*w_s
        self.theta_s = np.mod(self.theta_s, 2*np.pi)    # Limit to [0, 2*pi]
        self.i_s_old = i_s


# %%
class CurrentModelEstimator:
    """
    Current model flux estimator.

    This simple flux estimator requires speed measurement.

    Notes
    -----
    This is to be upgraded to a sensored flux observer later. The current model
    is not a feasible choice due to its sensitivity to the inductances.

    """

    def __init__(self, pars):
        """
        Parameters
        ----------
        pars : InductionMotorVectorCtrlPars (or its subset)
            Control parameters.

        """
        # Parameters
        self.T_s = pars.T_s
        self.R_R = pars.R_R
        self.L_M = pars.L_M
        # Initialize the states
        self.theta_s, self.psi_R = 0, 0

    def __call__(self, i_s, w_m):
        """
        Output and update the observer.

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
        w_s = self.output(i_s, w_m)
        self.update(i_s, w_s)

        return w_s

    def output(self, i_s, w_m):
        """
        Compute the outputs of the observer.

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
