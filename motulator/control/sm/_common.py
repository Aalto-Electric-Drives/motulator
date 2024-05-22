"""Common control functions and classes for for synchronous machine drives."""

from types import SimpleNamespace
from dataclasses import dataclass, field, InitVar
import numpy as np
from motulator._helpers import wrap


# %%
@dataclass
class ModelPars:
    """
    Model parameters of a synchronous machine.
    
    Parameters
    ----------
    R_s : float
        Stator resistance (Ω).
    L_d : float
        d-axis inductance (H).
    L_q : float
        q-axis inductance (H).
    psi_f : float
        PM flux linkage (Vs).
    n_p : int
        Number of pole pairs.
    J : float
        Moment of inertia (kgm²).

    """
    R_s: float = None
    L_d: float = None
    L_q: float = None
    psi_f: float = None
    n_p: int = None
    J: float = None


# %%
@dataclass
class ObserverPars:
    """
    Observer parameters.

    Parameters
    ----------
    model_par : ModelPars
        Machine model parameters.
    sensorless : bool
        If True, sensorless mode is used. 
    alpha_o : float, optional
        Observer bandwidth (rad/s). The default is 2*pi*40.
    k_o : callable, optional
        Observer gain as a function of the rotor angular speed. The default is 
        ``lambda w_m: 0.25*(R_s*(L_d + L_q)/(L_d*L_q) + 0.2*abs(w_m))`` if
        `sensorless` else ``lambda w_m: 2*pi*15``.
    k_f : callable, optional
        PM-flux estimation gain (V) as a function of the rotor angular speed. 
        The default is zero, ``lambda w_m: 0``. A typical nonzero gain is of 
        the form ``lambda w_m: max(k*(abs(w_m) - w_min), 0)``, i.e., zero below 
        the speed `w_min` (rad/s) and linearly increasing above that with the 
        slope `k` (Vs).

    """
    model_par: ModelPars
    sensorless: bool
    gain: SimpleNamespace = field(init=False)
    alpha_o: InitVar[float] = 2*np.pi*40
    k_o: InitVar[callable] = None
    k_f: InitVar[callable] = None

    def __post_init__(self, alpha_o, k_o, k_f):

        # Observer gain
        if self.sensorless:
            par = self.model_par
            sigma0 = .25*par.R_s*(par.L_d + par.L_q)/(par.L_d*par.L_q)
            k_o = (lambda w_m: sigma0 + .2*np.abs(w_m)) if k_o is None else k_o
        else:
            k_o = (lambda w_m: 2*np.pi*15) if k_o is None else k_o
        # PM-flux estimation gain
        k_f = (lambda w_m: 0) if k_f is None else k_f
        # Collect the gains
        self.gain = SimpleNamespace(alpha_o=alpha_o, k_o=k_o, k_f=k_f)


# %%
class Observer:
    """
    Observer for synchronous machines in estimated rotor coordinates.

    This observer estimates the stator flux linkage, the rotor angle, the rotor 
    speed, and (optionally) the PM-flux linkage. The design is based on 
    [#Hin2018]_ and [#Tuo2018]. The observer gain decouples the electrical and 
    mechanical dynamics and allows placing the poles of the corresponding 
    linearized estimation error dynamics. The PM-flux linkage can also be 
    estimated [#Tuo2018]_. The observer can also be used in the sensored mode,
    in which case the control system is fixed to the measured rotor angle.

    Parameters
    ----------
    observer_par : ObserverPars
        Observer parameters.
            
    References
    ----------
    .. [#Hin2018] Hinkkanen, Saarakkala, Awan, Mölsä, Tuovinen, "Observers for
       sensorless synchronous motor drives: Framework for design and analysis,"
       IEEE Trans. Ind. Appl., 2018, https://doi.org/10.1109/TIA.2018.2858753

    .. [#Tuo2018] Tuovinen, Awan, Kukkola, Saarakkala, Hinkkanen, "Permanent-
       magnet flux adaptation for sensorless synchronous motor drives," Proc. 
       IEEE SLED, 2018, https://doi.org/10.1109/SLED.2018.8485899

    """

    def __init__(self, observer_par):
        self.sensorless = observer_par.sensorless
        self.par = observer_par.model_par
        self.gain = observer_par.gain
        # Initialize states
        self.state = SimpleNamespace(
            theta_m=0, w_m=0, psi_s=self.par.psi_f, psi_f=self.par.psi_f)
        # Internal work variables for the update method
        self._work = SimpleNamespace(d_psi_s=0, d_psi_d=0, d_w_m=0)

    def output(self, fbk):
        """
        Compute the feedback signals for the control system.

        Parameters
        ----------
        fbk : SimpleNamespace
            Measured signals, which should contain the following fields:
            u_ss : complex
                Stator voltage (V) in stator coordinates.
            i_ss : complex
                Stator current (A) in stator coordinates.
            w_m : float, optional
                Rotor angular speed (electrical rad/s). This is only needed in 
                the sensored mode.
            theta_m : float, optional
                Rotor angle (electrical rad). This is only needed in the 
                sensored mode.

        Returns
        -------
        fbk : SimpleNamespace
            Measured and estimated feedback signals for the control system, 
            containing at least the following fields:
            u_s : complex
                Stator voltage (V) in estimated rotor coordinates.
            i_s : complex
                Stator current (A) in estimated rotor coordinates.
            psi_f : float
                PM-flux magnitude estimate (Vs). 
            theta_m : float
                Rotor angle estimate (electrical rad).
            w_s : float
                Angular frequency (rad/s) of the coordinate system.
            w_m : float
                Rotor speed estimate (electrical rad/s).
            psi_s : complex
                Stator flux estimate (Vs).

        """
        # Parameter estimates
        par = self.par

        # Get the flux estimates
        fbk.psi_s, fbk.psi_f = self.state.psi_s, self.state.psi_f

        # Get the mechanical variables
        if self.sensorless:
            fbk.theta_m, fbk.w_m = self.state.theta_m, self.state.w_m

        # Current and voltage vectors in (estimated) rotor coordinates
        fbk.i_s = np.exp(-1j*fbk.theta_m)*fbk.i_ss
        fbk.u_s = np.exp(-1j*fbk.theta_m)*fbk.u_ss

        # Current estimation error, scaled by the stator inductances
        e = (
            fbk.psi_f + par.L_d*fbk.i_s.real + 1j*par.L_q*fbk.i_s.imag -
            fbk.psi_s)

        # Auxiliary flux
        psi_a = fbk.psi_f + (par.L_d - par.L_q)*np.conj(fbk.i_s)

        # Observer gains and error terms
        if self.sensorless:
            # Observer gains
            k_o1 = self.gain.k_o(fbk.w_m)
            k_o2 = k_o1*psi_a/np.conj(psi_a) if np.abs(psi_a) > 0 else k_o1

            # Error term for the rotor angle and speed estimation
            eps_m = -np.imag(e/psi_a) if np.abs(psi_a) > 0 else 0
            # Angular speed of the coordinate system
            fbk.w_s = 2*self.gain.alpha_o*eps_m + fbk.w_m

            # Error term for the PM-flux estimation
            eps_f = -np.real(e/psi_a) if np.abs(psi_a) > 0 else 0
        else:
            # Sensored mode assumes that the control system operates in the
            # measured rotor coordinates
            fbk.w_s = fbk.w_m
            k_o1, k_o2 = self.gain.k_o(fbk.w_m), 0
            eps_m, eps_f = 0, 0

        # Compute and store the time derivatives for the update method
        self._work.d_psi_s = (
            fbk.u_s - par.R_s*fbk.i_s - 1j*fbk.w_s*fbk.psi_s + k_o1*e +
            k_o2*np.conj(e))
        self._work.d_psi_f = self.gain.k_f(fbk.w_m)*eps_f
        self._work.d_w_m = self.gain.alpha_o**2*eps_m

        return fbk

    def update(self, T_s, fbk):
        """
        Update the states.

        Parameters
        ----------
        T_s : float
            Sampling period (s).
        fbk : SimpleNamespace
            Feedback signals.

        """
        self.state.psi_s += T_s*self._work.d_psi_s
        self.state.psi_f += T_s*self._work.d_psi_f
        self.state.w_m += T_s*self._work.d_w_m
        self.state.theta_m = wrap(self.state.theta_m + T_s*fbk.w_s)
