"""
State observers for induction machines.
"""

import numpy as np


# %%
class Observer:
    """
    Reduced-order flux observer for induction machines in estimated 
    rotor flux coordinates.

    This class implements a reduced-order flux observer for induction machines.
    Both sensored and sensorless operation are supported. The observer structure
    is similar to [#Hin2010]_. The observer operates in estimated rotor flux coordinates. 
    
    Parameters
    ----------
    par : ModelPars
        Machine model parameters.
    k : callable, optional
        Observer gain as a function of the rotor angular speed. The default is 
        ``lambda w_m: (0.5*R_R/L_M + 0.2*abs(w_m))/(R_R/L_M - 1j*w_m)`` if
        `sensorless` else ``lambda w_m: 1 + 0.2*abs(w_m)/(R_R/L_M - 1j*w_m)``.
    alpha_o : float, optional
        Observer bandwidth (rad/s). The default is 2*pi*40.
    sensorless : bool, optional
        If True, sensorless mode is used. The default is True.

    Attributes
    ----------
    psi_R : float
        Rotor flux magnitude estimate (Vs).
    theta_s : float
        Rotor flux angle estimate (rad).
    w_m : float
        Rotor angular speed estimate (electrical rad/s). In sensored mode, this
        is the measured low-pass-filtered speed.

    Notes
    -----
    The pure voltage model corresponds to ``k = lambda w_m: 0``, resulting in 
    the marginally stable estimation-error dynamics. The current model is 
    obtained by setting ``k = lambda w_m: 1``. 

    References
    ----------
    .. [#Hin2010] Hinkkanen, Harnefors, Luomi, "Reduced-order flux observers 
       with stator-resistance adaptation for speed-sensorless induction motor 
       drives," IEEE Trans. Power Electron., 2010, 
       https://doi.org/10.1109/TPEL.2009.2039650

    """

    # pylint: disable=too-many-instance-attributes
    def __init__(self, par, k=None, alpha_o=2*np.pi*40, sensorless=True):
        # Model parameters
        self.R_s, self.R_R, self.L_sgm = par.R_s, par.R_R, par.L_sgm
        self.alpha = par.R_R/par.L_M

        # Other parameters
        self.alpha_o = alpha_o
        self.sensorless = sensorless

        # Observer gains
        self.k1 = k
        if self.sensorless:
            if self.k1 is None:  # If not given, use the default gains
                self.k1 = lambda w_m: (.5*self.alpha + .2*np.abs(w_m))/(
                    self.alpha - 1j*w_m)
            self.k2 = self.k1
        else:
            if self.k1 is None:
                self.k1 = lambda w_m: 1 + .2*np.abs(w_m)/(self.alpha - 1j*w_m)
            self.k2 = lambda w_m: 0

        # States
        self.psi_R, self.theta_s, self.w_m, self._i_s_old = 0, 0, 0, 0

    def _f(self, T_s, u_s, i_s, w_m):
        # Right-hand-side of the differential equation.

        # Induced voltage from stator quantities (without the w_s term that is
        # taken into account separately to avoid an algebraic loop)
        v_s = u_s - self.R_s*i_s - self.L_sgm*(i_s - self._i_s_old)/T_s

        # Induced voltage from the rotor quantities
        v_r = self.R_R*i_s - (self.alpha - 1j*w_m)*self.psi_R

        # Angular frequency of the rotor flux vector
        k1, k2 = self.k1(w_m), self.k2(w_m)
        den = self.psi_R + self.L_sgm*np.real((1 - k1)*i_s + k2*np.conj(i_s))
        num = np.imag(v_s + k1*(v_r - v_s) + k2*np.conj(v_r - v_s))
        w_s = num/den if den > 0 else w_m

        # Compute the derivative of the flux magnitude for the state update
        v = v_s - 1j*w_s*self.L_sgm*i_s
        dpsi_R = np.real(v + k1*(v_r - v) + k2*np.conj(v_r - v))

        return dpsi_R, w_s  # Note: w_s = dtheta_s

    # pylint: disable=too-many-arguments
    def _update(self, T_s, i_s, dpsi_R, w_s, w_m):
        # Update the states
        self.psi_R += T_s*dpsi_R
        self.theta_s += T_s*w_s  # Next line: limit into [-pi, pi)
        self.theta_s = np.mod(self.theta_s + np.pi, 2*np.pi) - np.pi
        self.w_m += T_s*self.alpha_o*(w_m - self.w_m)
        # Store the old current
        self._i_s_old = i_s

    def __call__(self, T_s, u_s, i_s, w_m=None):
        """
        Compute the angular frequency of the flux and update the states.

        Parameters
        ----------
        T_s : float
            Sampling period (s).
        u_s : complex
            Stator voltage (V) in estimated rotor flux coordinates.
        i_s : complex
            Stator current (A) in estimated rotor flux coordinates.
        w_m : float, optional
            Rotor angular speed (electrical rad/s). This signal is only used in
            sensored mode. The default is None.

        Returns
        -------
        w_s : float
            Angular frequency (rad/s) of the rotor flux estimate.

        """
        # Compute the time derivative of the flux estimate
        if self.sensorless:
            dpsi_R, w_s = self._f(T_s, u_s, i_s, self.w_m)
            # Slip and unfiltered speed estimates
            w_r = self.R_R*i_s.imag/self.psi_R if self.psi_R > 0 else 0
            w_m = w_s - w_r
        else:
            dpsi_R, w_s = self._f(T_s, u_s, i_s, w_m)

        # Update the states
        self._update(T_s, i_s, dpsi_R, w_s, w_m)

        return w_s


class FullOrderObserver:
    """
    Full-order flux observer for induction machines in estimated 
    rotor flux coordinates.

    This class implements a full-order flux observer for induction machines.
    The observer structure is similar to [#Tii2023]_. The observer operates in 
    estimated rotor flux coordinates. 
    
    Parameters
    ----------
    par : ModelPars
        Machine model parameters.
    k : callable, optional
        Observer gain as a function of the rotor angular speed. The default is 
        ``lambda w_m: (R_R/L_M + 0.2*abs(w_m))/(R_R/L_M - 1j*w_m)`` 
    alpha_o : float, optional
        Observer bandwidth (rad/s). The default is 2*pi*40.
    alpha_i : float, optional
        Current estimation bandwidth (rad/s). The default is 2*pi*400.
    sensorless : bool, optional
        If True, sensorless mode is used. The default is True.

    Attributes
    ----------
    psi_R : float
        Rotor flux magnitude estimate (Vs).
    i_s : float
        Stator current estimate (A).
    theta_s : float
        Rotor flux angle estimate (rad).
    w_m : float
        Integral state of the rotor angular speed estimate (electrical rad/s). 


    References
    ----------
    .. [#Tii2023] Tiitinen, Hinkkanen, Harnefors, "Speed-Adaptive Full-Order Observer 
        Revisited: Closed-Form Design for Induction Motor Drives," Proc. IEEE SLED, 
        Seoul, South Korea,  Aug. 2023, https://doi.org/10.1109/SLED57582.2023.10261359

    """

    # pylint: disable=too-many-instance-attributes
    def __init__(
            self,
            par,
            k=None,
            alpha_o=2*np.pi*40,
            alpha_i=2*np.pi*400,
            sensorless=True):
        # Model parameters
        self.R_s, self.R_R, self.L_sgm = par.R_s, par.R_R, par.L_sgm
        self.alpha = par.R_R/par.L_M

        # Other parameters
        self.alpha_o = alpha_o
        self.alpha_i = alpha_i
        self.g = alpha_i - self.alpha
        self.sensorless = sensorless

        # Observer gains
        self.k = k
        if self.sensorless:
            if self.k is None:  # If not given, use the default gains
                self.k = lambda w_m: (self.alpha + .2*np.abs(w_m))/(
                    self.alpha - 1j*w_m)
        else:
            raise NotImplementedError(
                'Sensored mode not implemented for full-order observer')

        # States
        self.psi_R, self.i_s, self.theta_s, self.w_m = 0, 0, 0, 0

    def _f(self, u_s, i_s, w_m):

        # Current estimation error
        i_err = i_s - self.i_s

        # Angular frequency of the rotor flux vector
        den = self.psi_R - self.L_sgm*i_err.real
        num = self.R_R*i_s.imag + self.L_sgm*(
            self.alpha_i*self.k(w_m).imag*i_err.real - self.g*i_err.imag)
        w_s = w_m + num/den if den > 0 else w_m

        # Compute the derivative of the current (vector) estimate
        # and flux magnitude for the state update.
        k_i = self.L_sgm*(self.g - 1j*(w_s - w_m)) - (self.R_R + self.R_s)
        di_s = (
            u_s - (self.R_s + self.R_R + 1j*w_s*self.L_sgm)*self.i_s +
            (self.alpha - 1j*w_m)*self.psi_R + k_i*i_err)/self.L_sgm

        dpsi_R = (
            -self.alpha*self.psi_R + self.R_R*i_s.real +
            (self.alpha_i*self.k(w_m).real - self.g)*self.L_sgm*i_err.real -
            (w_s - w_m)*self.L_sgm*i_err.imag)

        # Derivative of the integral term of the speed estimate
        dw_m = -self.alpha_o*self.alpha_i*self.L_sgm*i_err.imag/self.psi_R if self.psi_R > 0 else 0

        return di_s, dpsi_R, dw_m, w_s

    # pylint: disable=too-many-arguments
    def _update(self, T_s, di_s, dpsi_R, dw_m, w_s):
        # Update the states
        self.i_s += T_s*di_s
        self.psi_R += T_s*dpsi_R
        self.w_m += T_s*dw_m
        self.theta_s += T_s*w_s  # Next line: limit into [-pi, pi)
        self.theta_s = np.mod(self.theta_s + np.pi, 2*np.pi) - np.pi

    def __call__(self, T_s, u_s, i_s, w_m=None):
        """
        Compute the angular frequency of the flux and update the states.

        Parameters
        ----------
        T_s : float
            Sampling period (s).
        u_s : complex
            Stator voltage (V) in estimated rotor flux coordinates.
        i_s : complex
            Stator current (A) in estimated rotor flux coordinates.
        w_m : float, optional
            Rotor angular speed (electrical rad/s). This signal is not used
            and only exists for compatibility.

        Returns
        -------
        w_s : float
            Angular frequency (rad/s) of the rotor flux estimate.

        """
        # PI-type speed adaptation
        p_term = -self.alpha_o*self.L_sgm*(
            i_s - self.i_s).imag/self.psi_R if self.psi_R > 0 else 0
        w_m = p_term + self.w_m

        # Compute the time derivative of the flux estimate
        di_s, dpsi_R, dw_m, w_s = self._f(u_s, i_s, w_m)

        # Update the states
        self._update(T_s, di_s, dpsi_R, dw_m, w_s)

        return w_s


# %%
class FluxObserver:
    """
    Sensorless reduced-order flux observer in external coordinates.

    This is a sensorless reduced-order flux observer in synchronous coordinates
    for an induction machine. The observer gain decouples the electrical and
    mechanical dynamics and allows placing the poles of the linearized 
    estimation error dynamics. This implementation operates in external 
    coordinates (typically synchronous coordinates defined by reference signals 
    of a control system).

    Parameters
    ----------
    par : ModelPars
        Machine model parameters.
    alpha_o : float
        Speed-estimation bandwidth (rad/s).
    b : callable, optional 
        Coefficient (rad/s) of the characteristic polynomial as a function of 
        the rotor angular speed estimate. The default is 
        ``lambda w_m: R_R/L_M + .4*abs(w_m)``.

    Attributes
    ----------
    psi_R : complex
        Rotor flux estimate (Vs).
    w_m : float
        Rotor angular speed estimate (electrical rad/s).

    Notes
    -----
    The characteristic polynomial of the observer in synchronous coordinates is 
    ``s**2 + b*s + w_s**2``.  

    """

    # pylint: disable=too-many-instance-attributes
    def __init__(self, par, alpha_o, b=None):
        self.R_s, self.R_R, self.L_sgm = par.R_s, par.R_R, par.L_sgm
        self.alpha = par.R_R/par.L_M
        # Design parameters
        self.alpha_o = alpha_o
        zeta_inf = .2  # Damping ratio at high speeds
        self.b = lambda w_m: (
            self.alpha + 2*zeta_inf*np.abs(w_m) if b is None else b)
        # States
        self.psi_R, self.w_m = 0, 0
        # Previous current, private attribute
        self._i_s_old = 0

    def update(self, T_s, u_s, i_s, w_s):
        """
        Update the states.

        Parameters
        ----------
        T_s : float
            Sampling period (s).
        u_s : complex
            Stator voltage (V) in synchronous coordinates.
        i_s : complex
            Stator current (A) in synchronous coordinates.
        w_s : float
            Angular frequency (rad/s) of the coordinate system. 

        """
        # Observer gain with c = w_s**2 (without the orthogonal projection
        # which is embedded into the error signal and the state update)
        g = self.b(self.w_m)/(self.alpha - 1j*self.w_m)

        # Time derivative of the stator current
        di_s = (i_s - self._i_s_old)/T_s

        # Induced voltage from the rotor quantities
        e_r = self.R_R*i_s - (self.alpha - 1j*self.w_m)*self.psi_R

        # Induced voltage from stator quantities
        e_s = u_s - self.R_s*i_s - self.L_sgm*(di_s + 1j*w_s*i_s)

        # Error signal (rad/s)
        err = (e_s - e_r)/self.psi_R if np.abs(self.psi_R) > 0 else 0

        # Update the states
        self.psi_R += T_s*(e_s - (1j*w_s + g*err.real)*self.psi_R)
        self.w_m += T_s*self.alpha_o*err.imag
        self._i_s_old = i_s
