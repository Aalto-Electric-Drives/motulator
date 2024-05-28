"""Observer-based V/Hz control for induction machine drives."""

# %%
from types import SimpleNamespace
from dataclasses import dataclass
import numpy as np
from motulator.control import DriveCtrl, RateLimiter
from motulator._helpers import wrap


# %%
@dataclass
class ObserverBasedVHzCtrlCfg:
    """
    Control system configuration.

    Parameters
    ----------
    nom_psi_s : float
        Nominal stator flux linkage (Vs). 
    max_i_s : float, optional
        Maximum stator current (A). The default is inf.
    k_tau : float, optional
        Torque controller gain. The default is 3.
    alpha_psi : float, optional
        Stator flux control bandwidth (rad/s). The default is 2*pi*20.
    alpha_f : float, optional
        Torque high-pass filter bandwidth (rad/s). The default is 2*pi*1.
    alpha_r : float, optional
        Low-pass-filter bandwidth (rad/s) for slip angular frequency. The
        default is 2*pi*1.
    slip_compensation : bool, optional
        Enable slip compensation. The default is False.

    """
    nom_psi_s: float = None
    max_i_s: float = np.inf
    k_tau: float = 3.
    alpha_psi: float = 2*np.pi*20
    alpha_f: float = 2*np.pi*1
    alpha_r: float = 2*np.pi*1
    slip_compensation: bool = False


# %%
class ObserverBasedVHzCtrl(DriveCtrl):
    """
    Observer-based V/Hz control for induction machines.

    This implements the observer-based V/Hz control method [#Tii2022]_. The 
    state-feedback control law is in the alternative form which uses an 
    intermediate stator current reference.

    Parameters
    ----------
    par : ModelPars
        Machine model parameters.
    cfg : ObserverBasedVHzCtrlCfg
        Control system configuration.
    T_s : float, optional
        Sampling period (s). The default is 250e-6.

    References
    ----------
    .. [#Tii2022] Tiitinen, Hinkkanen, Harnefors, "Stable and passive observer-
       based V/Hz control for induction motors," Proc. IEEE ECCE, Detroit, MI, 
       Oct. 2022, https://doi.org/10.1109/ECCE50734.2022.9948057

    """

    def __init__(self, par, cfg, T_s=250e-6):
        super().__init__(par, T_s, sensorless=True)
        self.par, self.cfg = par, cfg
        # Subsystems
        self.observer = FluxObserver(par, alpha_o=2*np.pi*40)
        self.rate_limiter = RateLimiter(np.inf)
        # Initialize the states
        self.ref.tau_M, self.ref.w_r, self.theta_s = 0, 0, 0

    def output(self, fbk):
        """Output."""
        ref = super().output(fbk)

        # Unpack
        par, cfg = self.par, self.cfg

        # Define the reference voltage computation
        def reference_voltage(fbk, ref):
            # Internal current reference for state feedback
            ref.i_s = (ref.psi_s - fbk.psi_R)/par.L_sgm
            # Limit the reference
            if np.abs(ref.i_s) > cfg.max_i_s:
                ref.i_s = cfg.max_i_s*ref.i_s/np.abs(ref.i_s)
            # State feedback
            ref.u_s = (
                par.R_s*ref.i_s + 1j*fbk.w_s*ref.psi_s +
                par.L_sgm*cfg.alpha_psi*(ref.i_s - fbk.i_s))
            u_ss = ref.u_s*np.exp(1j*fbk.theta_s)
            ref.d_abc = self.pwm(ref.T_s, u_ss, fbk.u_dc, fbk.w_s)
            return ref

        # Get the reference signals
        ref.w_m = self.rate_limiter(ref.T_s, self.ref.w_m(ref.t))
        ref.w_r = self.ref.w_r
        ref.w_s = ref.w_m + int(cfg.slip_compensation)*ref.w_r
        ref.psi_s, ref.tau_M = self.cfg.nom_psi_s, self.ref.tau_M

        # Coordinate transformations
        fbk.theta_s = self.theta_s
        fbk.i_s = np.exp(-1j*fbk.theta_s)*fbk.i_ss
        fbk.u_s = np.exp(-1j*fbk.theta_s)*fbk.u_ss

        # Torque and slip estimates
        fbk.tau_M = 1.5*par.n_p*np.imag(fbk.i_s*np.conj(fbk.psi_R))
        psi_R_sqr = np.abs(fbk.psi_R)**2
        fbk.w_r = par.R_R*fbk.tau_M/(
            1.5*par.n_p*psi_R_sqr) if psi_R_sqr > 0 else 0

        # Dynamic frequency
        fbk.w_s = ref.w_s - cfg.k_tau*(fbk.tau_M - ref.tau_M)

        # State feedback
        ref = reference_voltage(fbk, ref)

        return ref

    def update(self, fbk, ref):
        """Update the states."""
        super().update(fbk, ref)
        # Low-pass filtered signals
        self.ref.w_r += ref.T_s*self.cfg.alpha_r*(fbk.w_r - ref.w_r)
        self.ref.tau_M += ref.T_s*self.cfg.alpha_f*(fbk.tau_M - ref.tau_M)
        # Update the angle
        self.theta_s = wrap(fbk.theta_s + ref.T_s*fbk.w_s)


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

    Notes
    -----
    The characteristic polynomial of the observer in synchronous coordinates is 
    ``s**2 + b*s + w_s**2``.  

    """

    def __init__(self, par, alpha_o, b=None):
        self.par = par
        # Design parameters
        self.alpha_o = alpha_o
        zeta_inf = .2  # Damping ratio at high speeds
        alpha = par.R_R/par.L_M
        self.b = lambda w_m: alpha + 2*zeta_inf*np.abs(w_m) if b is None else b
        # Initialize states
        self.est = SimpleNamespace(psi_R=0, w_m=0)
        # Private work variable
        self._old_i_s = 0

    def output(self, fbk):
        """Output."""
        fbk.psi_R, fbk.w_m = self.est.psi_R, self.est.w_m
        return fbk

    def update(self, T_s, fbk):
        """Update the states."""
        # Unpack
        par = self.par
        par.alpha = par.R_R/par.L_M

        # Observer gain with c = w_s**2 (without the orthogonal projection
        # which is embedded into the error signal and the state update)
        g = self.b(fbk.w_m)/(par.alpha - 1j*fbk.w_m)

        # Time derivative of the stator current
        d_i_s = (fbk.i_s - self._old_i_s)/T_s

        # Induced voltage from the rotor quantities
        e_r = par.R_R*fbk.i_s - (par.alpha - 1j*fbk.w_m)*fbk.psi_R

        # Induced voltage from stator quantities
        e_s = fbk.u_s - par.R_s*fbk.i_s - par.L_sgm*(
            d_i_s + 1j*fbk.w_s*fbk.i_s)

        # Error signal (rad/s)
        err = (e_s - e_r)/fbk.psi_R if np.abs(fbk.psi_R) > 0 else 0

        # Update the states
        self.est.psi_R += T_s*(e_s - (1j*fbk.w_s + g*err.real)*fbk.psi_R)
        self.est.w_m += T_s*self.alpha_o*err.imag
        self._old_i_s = fbk.i_s
