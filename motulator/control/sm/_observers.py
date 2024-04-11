"""
State observers for synchronous machines.
"""
import numpy as np


# %%
class Observer:
    """
    Observer for synchronous machines in estimated rotor coordinates.

    This observer estimates the rotor angle, the rotor speed, and the stator 
    flux linkage. The design is based on [#Hin2018]_. The observer gain 
    decouples the electrical and mechanical dynamics and allows placing the 
    poles of the corresponding linearized estimation error dynamics. This 
    implementation operates in estimated rotor coordinates. The observer can 
    also be used in the sensored mode by providing the measured rotor speed as 
    an input.

    Parameters
    ----------
    par : ModelPars
        Machine model parameters.
    alpha_o : float, optional
        Observer bandwidth (electrical rad/s). The default is 2*pi*40.
    k : callable, optional
        Observer gain as a function of the rotor angular speed. The default is 
        ``lambda w_m: 0.25*(R_s*(L_d + L_q)/(L_d*L_q) + 0.2*abs(w_m))`` if
        `sensorless` else ``lambda w_m: 2*pi*15``.
    sensorless : bool, optional
        If True, sensorless control is used. The default is True.

    Attributes
    ----------
    theta_m : float
        Rotor angle estimate (electrical rad).
    w_m : float
        Rotor speed estimate (electrical rad/s).
    psi_s : complex
        Stator flux estimate (Vs).

    References
    ----------
    .. [#Hin2018] Hinkkanen, Saarakkala, Awan, Mölsä, Tuovinen, "Observers for
       sensorless synchronous motor drives: Framework for design and analysis,"
       IEEE Trans. Ind. Appl., 2018, https://doi.org/10.1109/TIA.2018.2858753

    """

    # pylint: disable=too-many-instance-attributes
    def __init__(self, par, alpha_o=2*np.pi*40, k=None, sensorless=True):
        self.R_s = par.R_s
        self.L_d, self.L_q, self.psi_f = par.L_d, par.L_q, par.psi_f
        self.psi_f = par.psi_f
        self.alpha_o = alpha_o
        self.sensorless = sensorless
        self.k1 = k

        if self.sensorless:
            if self.k1 is None:  # If not given, use the default gains
                sigma0 = .25*par.R_s*(par.L_d + par.L_q)/(par.L_d*par.L_q)
                self.k1 = lambda w_m: (sigma0 + .2*np.abs(w_m))
            self.k2 = self.k1
        else:
            if self.k1 is None:
                self.k1 = lambda w_m: 2*np.pi*15
            self.k2 = lambda w_m: 0

        # Initial states
        self.theta_m, self.w_m, self.psi_s = 0, 0, par.psi_f

    def update(self, T_s, u_s, i_s, w_m=None):
        """
        Update the states for the next sampling period.

        Parameters
        ----------
        T_s : float
            Sampling period (s).
        u_s : complex
            Stator voltage (V) in estimated rotor coordinates.
        i_s : complex
            Stator current (A) in estimated rotor coordinates.
        w_m : float, optional
            Rotor angular speed (electrical rad/s). Needed only in the sensored
            mode. The default is None. 

        """

        # Estimation error
        e = self.L_d*i_s.real + 1j*self.L_q*i_s.imag + self.psi_f - self.psi_s

        if self.sensorless:
            # Auxiliary flux
            psi_a = self.psi_f + (self.L_d - self.L_q)*np.conj(i_s)

            # Observer gains
            k1 = self.k1(self.w_m)
            k2 = k1*psi_a/np.conj(psi_a) if np.abs(psi_a) > 0 else k1

            # Speed estimation
            eps = -np.imag(e/psi_a) if np.abs(psi_a) > 0 else 0
            w_s = 2*self.alpha_o*eps + self.w_m
        else:
            k1, k2 = self.k1(w_m), 0
            w_s = w_m
            eps = 0

        # Update the states
        self.psi_s += T_s*(
            u_s - self.R_s*i_s - 1j*w_s*self.psi_s + k1*e + k2*np.conj(e))
        self.w_m += T_s*self.alpha_o**2*eps
        self.theta_m += T_s*w_s  # Next line: limit into [-pi, pi)
        self.theta_m = np.mod(self.theta_m + np.pi, 2*np.pi) - np.pi


# %%
class FluxObserver:
    """
    Sensorless stator flux observer in external coordinates.

    This observer estimates the stator flux linkage and the angle of the 
    coordinate system with respect to the d-axis of the rotor. Speed-estimation 
    is omitted. The observer gain decouples the electrical and mechanical 
    dynamics and allows placing the poles of the corresponding linearized 
    estimation error dynamics. This implementation operates in external 
    coordinates (typically synchronous coordinates defined by reference signals 
    of a control system).

    Parameters
    ----------
    par : ModelPars
        Machine model parameters.
    alpha_o : float, optional
        Observer gain (rad/s). The default is 2*pi*20.
    zeta_inf : float, optional
        Damping ratio at infinite speed. The default is 0.2.

    Attributes
    ----------
    delta : float
        Angle estimate of the coordinate system with respect to the d-axis of 
        the rotor (electrical rad).
    psi_s : complex
        Stator flux estimate (Vs).
  
    """

    # pylint: disable=too-many-instance-attributes
    def __init__(self, par, alpha_o=2*np.pi*20, zeta_inf=.2):
        self.R_s = par.R_s
        self.L_d = par.L_d
        self.L_q = par.L_q
        self.psi_f = par.psi_f
        self.alpha_o = alpha_o
        self.b_p = .5*par.R_s*(par.L_d + par.L_q)/(par.L_d*par.L_q)
        self.zeta_inf = zeta_inf
        # Initial states
        self.delta, self.psi_s = 0, par.psi_f

    def update(self, T_s, u_s, i_s, w_s):
        """
        Update the states for the next sampling period.

        Parameters
        ----------
        T_s : float
            Sampling period (s).
        u_s : complex
            Stator voltage (V).
        i_s : complex
            Stator current (A).
        w_s : float
            Stator angular frequency (rad/s).

        """
        # Transformations to rotor coordinates. This is mathematically
        # equivalent to the version in [Tii2022].
        i_sr = i_s*np.exp(1j*self.delta)
        psi_sr = self.psi_s*np.exp(1j*self.delta)

        # Auxiliary flux and estimation error in rotor coordinates
        psi_ar = self.psi_f + (self.L_d - self.L_q)*np.conj(i_sr)
        e_r = self.L_d*i_sr.real + 1j*self.L_q*i_sr.imag + self.psi_f - psi_sr

        # Auxiliary flux in controller coordinates
        psi_a = np.exp(-1j*self.delta)*psi_ar

        k = self.b_p + 2*self.zeta_inf*np.abs(w_s)

        if np.abs(psi_ar) > 0:
            # Correction voltage in controller coordinates
            v = k*psi_a*np.real(e_r/psi_ar)
            # Error signal
            w_delta = self.alpha_o*np.imag(e_r/psi_ar)
        else:
            v, w_delta = 0, 0

        # Update the states
        self.psi_s += T_s*(u_s - self.R_s*i_s - 1j*w_s*self.psi_s + v)
        self.delta += T_s*w_delta
