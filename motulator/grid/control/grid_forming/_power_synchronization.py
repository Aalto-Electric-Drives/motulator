"""Power synchronization control for grid-connected converters."""

# %%
from dataclasses import dataclass

import numpy as np

from motulator.common.utils import wrap
from motulator.grid.control import CurrentLimiter, GridConverterControlSystem
from motulator.grid.utils import FilterPars, GridPars


# %%
@dataclass
class PSCControlCfg:
    """
    Power synchronization control configuration.

    Parameters
    ----------
    grid_par : GridPars
        Grid model parameters.
    filter_par : FilterPars
        Filter model parameters.
    C_dc : float, optional
        DC-bus capacitance (F). Default is None.
    T_s : float, optional
        Sampling period of the controller (s). Default is 1/(16e3).
    on_rf : bool, optional
        Enable reference-feedforward for the control. Default is False.
    i_max : float, optional
        Maximum current modulus (A). Default is 20.
    R_a : float, optional
        Damping resistance (Ω). Default is 4.6.
    w_b : float, optional
        Current low-pass filter bandwidth (rad/s). Default is 2*pi*5.
    overmodulation : str, optional
        Overmodulation method for the PWM. Default is Minimum Phase Error
        "MPE".
    """

    grid_par: GridPars
    filter_par: FilterPars
    C_dc: float = None
    T_s: float = 1/(16e3)
    on_rf: bool = False
    i_max: float = 20
    R_a: float = 4.6
    w_b: float = 2*np.pi*5
    overmodulation: str = "MPE"

    def __post_init__(self):
        par = self.grid_par
        self.k_p_psc = par.w_gN*self.R_a/(1.5*par.u_gN*par.u_gN)


# %%
class PSCControl(GridConverterControlSystem):
    """
    Power synchronization control for grid converters.
    
    This implements the power synchronization control (PSC) method described in
    [#Har2019]_. The alternative reference-feedforward PSC (RFPSC) can also be 
    used and is based on [#Har2020]_.

    Parameters
    ----------
    cfg : PSCControlCfg
        Model and controller configuration parameters.

    Attributes
    ----------
    current_limiter : CurrentLimiter
        Transparent current controller used for current limitation.
    
    References
    ----------
    .. [#Har2019] Harnefors, Hinkkanen, Riaz, Rahman, Zhang, "Robust Analytic
        Design of Power-Synchronization Control," IEEE Trans. Ind. Electron.,
        Aug. 2019, https://doi.org/10.1109/TIE.2018.2874584
        
    .. [#Har2020] Harnefors, Rahman, Hinkkanen, Routimo, "Reference-Feedforward
        Power-Synchronization Control," IEEE Trans. Power Electron., Sep. 2020,
        https://doi.org/10.1109/TPEL.2020.2970991

    """

    def __init__(self, cfg):
        super().__init__(
            cfg.grid_par,
            cfg.C_dc,
            cfg.T_s,
        )
        self.cfg = cfg
        self.current_limiter = CurrentLimiter(cfg.i_max)
        self.ref.q_g = 0
        # Initialize the states
        self.theta_c = 0
        self.i_c_filt = 0j

    def get_feedback_signals(self, mdl):
        """Get the feedback signals."""
        fbk = super().get_feedback_signals(mdl)
        fbk.theta_c = self.theta_c
        fbk.i_c_filt = self.i_c_filt
        # Transform the measured values into synchronous coordinates
        fbk.u_g = np.exp(-1j*fbk.theta_c)*fbk.u_gs
        fbk.i_c = np.exp(-1j*fbk.theta_c)*fbk.i_cs
        fbk.u_c = np.exp(-1j*fbk.theta_c)*fbk.u_cs

        # Calculation of active and reactive powers
        fbk.p_g = 1.5*np.real(fbk.u_c*np.conj(fbk.i_c))
        fbk.q_g = 1.5*np.imag(fbk.u_c*np.conj(fbk.i_c))

        return fbk

    def output(self, fbk):
        """Extend the base class method."""
        par, cfg = self.grid_par, self.cfg

        # Get the reference signals
        ref = super().output(fbk)
        ref = super().get_power_reference(fbk, ref)
        # Voltage magnitude reference
        ref.U = self.ref.U(ref.t) if callable(self.ref.U) else self.ref.U

        # Calculation of power droop
        fbk.w_c = par.w_gN + (cfg.k_p_psc)*(ref.p_g - fbk.p_g)

        # Optionally, use of reference feedforward for d-axis current
        if cfg.on_rf:
            ref.i_c = ref.p_g/(ref.U*1.5) + 1j*np.imag(fbk.i_c_filt)
        else:
            ref.i_c = fbk.i_c_filt

        # Transparent current control
        ref.i_c = self.current_limiter(ref.i_c)

        # Calculation of converter voltage output reference
        ref.u_c = (ref.U + 0j) + cfg.R_a*(ref.i_c - fbk.i_c)

        # Transform voltage reference into stator coordinates
        ref.u_cs = np.exp(1j*fbk.theta_c)*ref.u_c

        # Get duty ratios from PWM
        ref.d_abc = self.pwm(
            ref.T_s,
            ref.u_cs,
            fbk.u_dc,
            par.w_gN,
            cfg.overmodulation,
        )

        return ref

    def update(self, fbk, ref):
        """Extend the base class method."""
        super().update(fbk, ref)
        # Estimated phase angle
        self.theta_c = fbk.theta_c + ref.T_s*fbk.w_c
        # Limit to [-pi, pi]
        self.theta_c = wrap(self.theta_c)
        # Low-pass filtering of converter current
        cfg = self.cfg
        self.i_c_filt = (1 - ref.T_s*cfg.w_b)*self.i_c_filt + (
            ref.T_s*cfg.w_b*fbk.i_c)
