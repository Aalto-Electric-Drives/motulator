"""Common control functions and classes."""

from abc import ABC
from types import SimpleNamespace

import numpy as np

from motulator.common.control import ControlSystem, PIController, PWM
from motulator.common.utils import abc2complex, wrap


# %%
class PLL:
    """
    Phase-locked loop including the voltage-magnitude filtering.

    This class provides a simple frequency-tracking phase-locked loop. The
    magnitude of the measured PCC voltage is also filtered.

    Parameters
    ----------
    alpha_pll : float
        Frequency-tracking bandwidth.
    abs_u_g0 : float
        Initial value for the grid voltage estimate.
    w_g0 : float
        Initial value for the grid angular frequency estimate.

    """

    def __init__(self, alpha_pll, abs_u_g0, w_g0, theta_c0=0):
        self.est = SimpleNamespace(
            w_g=w_g0, theta_c=theta_c0, abs_u_g=abs_u_g0)
        self.gain = SimpleNamespace(alpha_g=2*alpha_pll, k_w=alpha_pll**2)

    def output(self, fbk):
        """Output the estimates and coordinate transformed quantities."""
        # Observer states
        fbk.theta_c = self.est.theta_c
        fbk.w_g = self.est.w_g
        # Coordinate transformations
        fbk.u_g = np.exp(-1j*fbk.theta_c)*fbk.u_gs
        fbk.i_c = np.exp(-1j*fbk.theta_c)*fbk.i_cs
        fbk.u_c = np.exp(-1j*fbk.theta_c)*fbk.u_cs
        # Error signal
        fbk.eps = fbk.u_g.imag/self.est.abs_u_g if self.est.abs_u_g > 0 else 0
        # Angular speed of the coordinate system
        fbk.w_c = fbk.w_g + self.gain.alpha_g*fbk.eps
        return fbk

    def update(self, T_s, fbk):
        """Update the integral states."""
        self.est.theta_c += T_s*fbk.w_c
        self.est.theta_c = wrap(self.est.theta_c)
        self.est.w_g += T_s*self.gain.k_w*fbk.eps
        self.est.abs_u_g += T_s*self.gain.alpha_g*(
            fbk.u_g.real - self.est.abs_u_g)


# %%
class DCBusVoltageController(PIController):
    """
    PI controller for the DC-bus voltage.

    This is a PI controller for the DC-bus voltage. The controller regulates
    the energy stored in the DC-bus capacitor (scaled square of the DC-bus
    voltage) in order to have a linear closed-loop system [#Hur2001]_. The
    gains are initialized based on the desired closed-loop bandwidth.

    Parameters
    ----------
    C_dc : float
        DC-bus capacitance (F).
    alpha_dc : float
        Closed-loop bandwidth (rad/s).
    zeta : float, optional
        Damping ratio of the closed-loop system. The default is 1.
    max_p : float, optional
        Limit for the maximum converter power (W). The default is `inf`.

    References
    ----------
    .. [#Hur2001] Hur, Jung, Nam, "A fast dynamic DC-link power-balancing
       scheme for a PWM converter-inverter system," IEEE Trans. Ind. Electron.,
       2001, https://doi.org/10.1109/41.937412

    """

    def __init__(self, C_dc, alpha_dc, zeta=1, max_p=np.inf):
        k_p, k_i = -2*zeta*alpha_dc, -alpha_dc**2
        k_t = k_p
        super().__init__(k_p, k_i, k_t, max_p)
        self.C_dc = C_dc

    def output(self, ref_u_dc, u_dc, u_ff=0):
        # pylint: disable=arguments-renamed
        # Extends the base class method by transforming the
        ref_W_dc = .5*self.C_dc*ref_u_dc**2
        W_dc = .5*self.C_dc*u_dc**2
        return super().output(ref_W_dc, W_dc, u_ff)


# %%
class GridConverterControlSystem(ControlSystem, ABC):
    """
    Base class for control of grid-connected converters.

    This base class provides typical functionalities for control of
    grid-connected converters. This can be used both in power control and
    DC-bus voltage control modes.

    Parameters
    ----------
    T_s : float
        Sampling period (s).

    Attributes
    ----------
    ref : SimpleNamespace
        References, possibly containing the following fields:

            v : float | callable
                Converter output voltage reference (V). Can be given either as
                a constant or a function of time (s).
            p_g : callable
                Active power reference (W) as a function of time (s). This
                signal is needed in power control mode.
            q_g : callable
                Reactive power reference (VAr) as a function of time (s). This
                signal is needed if grid-following control is used.
            u_dc : callable
                DC-voltage reference (V) as a function of time (s). This signal
                is needed in DC-bus voltage control mode.

    dc_bus_voltage_ctrl : DCBusVoltageController | None
        DC-bus voltage controller. The default is None.

    """

    def __init__(self, T_s):
        super().__init__(T_s)
        self.dc_bus_voltage_ctrl = None
        self.pwm = PWM(overmodulation="MPE")
        self.ref = SimpleNamespace()

    def get_electrical_measurements(self, fbk, mdl):
        """
        Measure the currents and voltages.

        Parameters
        ----------
        fbk : SimpleNamespace
            Measured signals are added to this object.
        mdl : Model
            Continuous-time system model.

        Returns
        -------
        fbk : SimpleNamespace
            Measured signals, containing the following fields:

                u_dc : float
                    DC-bus voltage (V).
                i_cs : complex
                    Converter current (A) in stationary coordinates.
                u_cs : complex
                    Realized converter output voltage (V) in stationary
                    coordinates. This signal is obtained from the PWM.
                u_gs : complex
                    PCC voltage (V) in stationary coordinates.

        """
        fbk.u_dc = mdl.converter.meas_dc_voltage()
        fbk.i_cs = abc2complex(mdl.ac_filter.meas_currents())
        fbk.u_cs = self.pwm.get_realized_voltage()
        fbk.u_gs = abc2complex(mdl.ac_filter.meas_pcc_voltages())
        # fbk.u_gs = abc2complex(mdl.ac_filter.meas_capacitor_voltages())

        return fbk

    def get_feedback_signals(self, mdl):
        """Get the feedback signals."""
        fbk = super().get_feedback_signals(mdl)
        fbk = self.get_electrical_measurements(fbk, mdl)

        return fbk

    def get_power_reference(self, fbk, ref):
        """
        Get the active power reference in DC bus voltage control mode.

        Parameters
        ----------
        fbk : SimpleNamespace
            Feedback signals.
        ref : SimpleNamespace
            Reference signals, containing the digital time `t`.

        Returns
        -------
        ref : SimpleNamespace
            Reference signals, containing the following fields:

                u_dc : float
                    DC-bus voltage reference (V).
                p_g : float
                    Active power reference (W).
                q_g : float
                    Reactive power reference (VAr).

        """
        if self.dc_bus_voltage_ctrl:
            # DC-bus voltage control mode
            ref.u_dc = self.ref.u_dc(ref.t)
            ref.p_g = self.dc_bus_voltage_ctrl.output(ref.u_dc, fbk.u_dc)
        else:
            # Power control mode
            #ref.u_dc = None
            ref.p_g = self.ref.p_g(ref.t)

        # Reactive power reference
        ref.q_g = self.ref.q_g(ref.t) if callable(
            self.ref.q_g) else self.ref.q_g

        return ref

    def update(self, fbk, ref):
        """Extend the base class method."""
        super().update(fbk, ref)
        if self.dc_bus_voltage_ctrl:
            self.dc_bus_voltage_ctrl.update(ref.T_s, ref.p_g)


# %%
class CurrentLimiter:
    """
    Limit the amplitude of the input signal.

    Parameters
    ----------
    max_i : float
        Maximum current (A).

    Returns
    -------
    complex
        Limited signal.

    """

    def __init__(self, max_i):
        self.max_i = max_i

    def __call__(self, i):
        abs_i = np.abs(i)
        if abs_i > self.max_i:
            i = (self.max_i/abs_i)*i
        return i
