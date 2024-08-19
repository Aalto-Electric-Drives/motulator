"""grid following control methods for grid onverters."""

# %%
from dataclasses import dataclass

import numpy as np

from motulator.common.control import ComplexPIController

from motulator.grid.control import (
    CurrentLimiter,
    GridConverterControlSystem,
    PLL,
)
from motulator.grid.utils import FilterPars, GridPars


# %%
@dataclass
class GFLControlCfg:
    """Grid-following control configuration
    
    Parameters
    ----------
    grid_par : GridPars
        Grid model parameters.
    filter_par : FilterPars
        Filter parameters.
    C_dc : float, optional
        DC bus capacitance (F). The default is None.
    T_s : float, optional
        Sampling period (s). The default is 1/(16e3).
    on_u_cap : bool, optional
        Use filter capacitance voltage instead of PCC voltage for the feedback.
        Default is False.
    i_max : float, optional
        Maximum current modulus in A. The default is 20.
    alpha_c : float, optional
        Current controller bandwidth. The default is 2*pi*400.
    alpha_ff : float, optional
        Low pass filter bandwidth for voltage feedforward term. The default
        is 2*pi*(4*50).
    overmodulation : str, optional
        Overmodulation method for the PWM. Default is Minimum Phase Error
        "MPE".
        
    Parameters for the Phase Locked Loop (PLL)
    w0_pll : float, optional
        undamped natural frequency of the PLL. The default is 2*pi*20.
    zeta_pll : float, optional
        damping ratio of the PLL. The default is 1.
    """

    grid_par: GridPars
    filter_par: FilterPars
    C_dc: float = None
    T_s: float = 1/(16e3)
    on_u_cap: bool = False
    i_max: float = 20
    alpha_c: float = 2*np.pi*400
    alpha_ff: float = 2*np.pi*(4*50)

    w0_pll: float = 2*np.pi*20
    zeta_pll: float = 1
    overmodulation: str = "MPE"


# %%
class GFLControl(GridConverterControlSystem):
    """
    Grid-following control for power converters.
    
    Parameters
    ----------
    cfg : GFLControlCfg
        Control configuration.

    Attributes
    ----------
    current_ctrl : CurrentController
        Current controller.
    pll : PLL
        Phase locked loop.
    current_reference : CurrentRefCalc
        Current reference calculator.
    """

    def __init__(self, cfg):
        super().__init__(
            cfg.grid_par,
            cfg.C_dc,
            cfg.T_s,
            on_u_cap=cfg.on_u_cap,
        )
        self.cfg = cfg
        self.current_ctrl = CurrentController(cfg)
        self.pll = PLL(
            T_s=cfg.T_s,
            w0=cfg.w0_pll,
            zeta=cfg.zeta_pll,
            grid_par=cfg.grid_par,
        )
        self.current_reference = CurrentRefCalc(cfg)

        # Initialize the states
        self.u_filt = cfg.grid_par.u_gN + 1j*0

    def get_feedback_signals(self, mdl):
        fbk = super().get_feedback_signals(mdl)
        fbk.theta_c = self.pll.theta_c
        # Transform the measured current in dq frame
        fbk.u_g = np.exp(-1j*fbk.theta_c)*fbk.u_gs
        fbk.i_c = np.exp(-1j*fbk.theta_c)*fbk.i_cs
        fbk.u_c = np.exp(-1j*fbk.theta_c)*fbk.u_cs

        # Calculating of active and reactive powers
        fbk.p_g = 1.5*np.real(fbk.u_c*np.conj(fbk.i_c))
        fbk.q_g = 1.5*np.imag(fbk.u_c*np.conj(fbk.i_c))

        return fbk

    def output(self, fbk):
        """Extend the base class method."""
        grid_par = self.cfg.grid_par
        # Get the reference signals
        ref = super().output(fbk)
        super().get_power_reference(fbk, ref)

        # current reference calculation, with current limitation
        self.current_reference.get_current_reference(ref)

        # Low pass filter for the feedforward PCC voltage:
        u_filt = self.u_filt

        # Use of PLL to bring ugq to zero
        self.pll.output(fbk, ref)

        # Voltage reference generation in synchronous coordinates
        ref.u_c = self.current_ctrl.output(ref.i_c, fbk.i_c, u_filt)

        # Transform the voltage reference into stator coordinates
        ref.u_cs = np.exp(1j*fbk.theta_c)*ref.u_c

        # get the duty ratios from the PWM
        ref.d_abc = self.pwm(
            ref.T_s,
            ref.u_cs,
            fbk.u_dc,
            grid_par.w_gN,
            self.cfg.overmodulation,
        )

        return ref

    def update(self, fbk, ref):
        """Extend the base class method."""
        super().update(fbk, ref)
        self.current_ctrl.update(ref.T_s, fbk.u_c, self.grid_par.w_gN)
        self.pll.update(ref.u_gq)
        self.u_filt = (1 - ref.T_s*self.cfg.alpha_ff)*self.u_filt + (
            ref.T_s*self.cfg.alpha_ff*fbk.u_g)


# %%
class CurrentController(ComplexPIController):
    """
    2DOF PI current controller for grid converters.

    This class provides an interface for a current controller for grid 
    converters. The gains are initialized based on the desired closed-loop 
    bandwidth and the filter inductance. 

    Parameters
    ----------
    cfg : GFLControlCfg
        Control configuration parameters, of which the following fields
        are used:

            filter_par.L_fc : float
                Converter-side filter inductance (H).
            alpha_c : float
                Closed-loop bandwidth of the current controller (rad/s).

    """

    # TODO: should the total inductance be used here?
    def __init__(self, cfg):
        k_t = cfg.alpha_c*cfg.filter_par.L_fc
        k_i = cfg.alpha_c*k_t
        k_p = 2*k_t
        super().__init__(k_p, k_i, k_t)


# %%
class CurrentRefCalc:
    """
    Current controller reference generator
    
    This class is used to generate the current references for the current
    controllers based on the active and reactive power references. The current
    limiting algorithm is used to limit the current references.
    
    """

    def __init__(self, cfg):
        """
        Parameters
        ----------
        cfg : GFLControlCfg
            Model and controller configuration parameters.
    
        """
        self.u_gN = cfg.grid_par.u_gN
        self.current_limiter = CurrentLimiter(cfg.i_max)

    def get_current_reference(self, ref):
        """
        Current reference generator.
    
        Parameters
        ----------
        p_g_ref : float
            active power reference
        q_g_ref : float
            reactive power reference

        Returns
        -------
        ref : SimpleNamespace
            Reference signals, containing the following fields:
                
                i_c : complex
                    Current reference in the stationary frame (A).
        """

        # Calculation of the current references in the stationary frame:
        ref.i_c = 2*ref.p_g/(3*self.u_gN) - 2*1j*ref.q_g/(3*self.u_gN)
        ref.i_c = self.current_limiter(ref.i_c)

        return ref
