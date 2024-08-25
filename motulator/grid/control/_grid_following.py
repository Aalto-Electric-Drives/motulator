"""Grid-following control methods for grid converters."""

# %%
from dataclasses import dataclass, InitVar

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
    alpha_ff : float, optional
        Low-pass-filtering bandwidth (rad/s) for the voltage-feedforward term. 
        The default is 2*pi*200.
    w0_pll : float, optional
        Undamped natural frequency of the PLL. The default is 2*pi*20.
    zeta_pll : float, optional
        Damping ratio of the PLL. The default is 1.
    C_dc : float, optional
        DC-bus capacitance (F). The default is None.

    """

    grid_par: GridPars
    filter_par: FilterPars
    max_i: float
    T_s: float = 100e-6
    alpha_c: float = 2*np.pi*400
    alpha_ff: float = 2*np.pi*200
    w0_pll: InitVar[float] = 2*np.pi*20
    zeta_pll: InitVar[float] = 1
    C_dc: float = None

    def __post_init__(self, w0_pll, zeta_pll):
        self.k_p_pll = 2*zeta_pll*w0_pll/self.grid_par.u_gN
        self.k_i_pll = w0_pll**2/self.grid_par.u_gN


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
        self.pll = PLL(cfg.k_p_pll, cfg.k_i_pll, cfg.grid_par.w_gN)
        self.current_reference = CurrentRefCalc(cfg)

        # Initialize the states
        self.u_flt = cfg.grid_par.u_gN + 0j

    def get_feedback_signals(self, mdl):
        fbk = super().get_feedback_signals(mdl)
        fbk.theta_c = self.pll.est.theta_g
        # Transform the measured current in dq frame
        fbk.u_g = np.exp(-1j*fbk.theta_c)*fbk.u_gs
        fbk.i_c = np.exp(-1j*fbk.theta_c)*fbk.i_cs
        fbk.u_c = np.exp(-1j*fbk.theta_c)*fbk.u_cs

        # Calculating of active and reactive powers
        fbk.p_g = 1.5*np.real(fbk.u_c*np.conj(fbk.i_c))
        fbk.q_g = 1.5*np.imag(fbk.u_c*np.conj(fbk.i_c))

        # PLL
        fbk = self.pll.output(fbk)

        return fbk

    def output(self, fbk):
        """Extend the base class method."""
        # Get the reference signals
        ref = super().output(fbk)
        ref = super().get_power_reference(fbk, ref)
        ref = self.current_reference.get_current_reference(ref)

        # Voltage reference generation in synchronous coordinates
        ref.u_c = self.current_ctrl.output(ref.i_c, fbk.i_c, self.u_flt)
        ref.u_cs = np.exp(1j*fbk.theta_c)*ref.u_c

        # Duty ratios for PWM
        ref.d_abc = self.pwm(ref.T_s, ref.u_cs, fbk.u_dc, fbk.w_g)

        return ref

    def update(self, fbk, ref):
        """Extend the base class method."""
        super().update(fbk, ref)
        self.current_ctrl.update(ref.T_s, fbk.u_c, self.grid_par.w_gN)
        self.u_flt += ref.T_s*self.cfg.alpha_ff*(fbk.u_g - self.u_flt)
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
