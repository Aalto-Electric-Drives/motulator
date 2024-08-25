"""Grid-following control methods for grid converters."""

from dataclasses import dataclass

import numpy as np

from motulator.common.control import ComplexPIController
from motulator.grid.control._common import (
    CurrentLimiter, GridConverterControlSystem, PLL)
from motulator.grid.utils import FilterPars, GridPars


# %%
@dataclass
class GFLControlCfg:
    """
    Grid-following control configuration
    
    Parameters
    ----------
    grid_par : GridPars
        Grid model parameters.
    filter_par : FilterPars
        Filter parameters.
    max_i : float
        Maximum current (A). 
    T_s : float, optional
        Sampling period (s). The default is 100e-6.
    alpha_c : float, optional
        Current-control bandwidth (rad/s). The default is 2*pi*400.
    alpha_pll : float, optional
        PLL frequency-tracking bandwidth (rad/s). The default is 2*pi*20.
    C_dc : float, optional
        DC-bus capacitance (F). The default is None.

    """
    grid_par: GridPars
    filter_par: FilterPars
    max_i: float
    T_s: float = 100e-6
    alpha_c: float = 2*np.pi*400
    alpha_pll: float = 2*np.pi*20
    C_dc: float = None


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
        super().__init__(cfg.grid_par, cfg.C_dc, cfg.T_s)
        self.cfg = cfg
        self.current_ctrl = CurrentController(cfg)
        self.pll = PLL(cfg.alpha_pll, cfg.grid_par.u_gN, cfg.grid_par.w_gN)
        self.current_reference = CurrentRefCalc(cfg)

    def get_feedback_signals(self, mdl):
        fbk = super().get_feedback_signals(mdl)
        fbk = self.pll.output(fbk)
        s_g = 1.5*fbk.u_c*np.conj(fbk.i_c)
        fbk.p_g = s_g.real
        fbk.q_g = s_g.imag
        return fbk

    def output(self, fbk):
        """Extend the base class method."""
        # Get the reference signals
        ref = super().output(fbk)
        ref = super().get_power_reference(fbk, ref)
        ref = self.current_reference.get_current_reference(ref)
        # Voltage reference generation in synchronous coordinates
        ref.u_c = self.current_ctrl.output(
            ref.i_c, fbk.i_c, self.pll.est.abs_u_g)
        ref.u_cs = np.exp(1j*fbk.theta_c)*ref.u_c
        # Duty ratios for PWM
        ref.d_abc = self.pwm(ref.T_s, ref.u_cs, fbk.u_dc, fbk.w_c)
        return ref

    def update(self, fbk, ref):
        """Extend the base class method."""
        super().update(fbk, ref)
        self.current_ctrl.update(ref.T_s, fbk.u_c, fbk.w_c)
        self.pll.update(ref.T_s, fbk)


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
        Control configuration parameters:

            filter_par.L_fc : float
                Converter-side filter inductance (H).
            alpha_c : float
                Closed-loop bandwidth (rad/s) of the current controller.

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
        self.current_limiter = CurrentLimiter(cfg.max_i)

    def get_current_reference(self, ref):
        """Current reference generator."""
        # Calculation of the current references in the stationary frame:
        ref.i_c = 2*ref.p_g/(3*self.u_gN) - 2j*ref.q_g/(3*self.u_gN)
        ref.i_c = self.current_limiter(ref.i_c)

        return ref
