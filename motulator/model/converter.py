# pylint: disable=C0103
"""
This module contains power converter models.

An inverter with constant DC-bus voltage and a frequency converter with a diode
front-end rectifier are modeled. Complex space vectors are used also for duty
ratios and switching states, wherever applicable. In this module, all space
vectors are in stationary coordinates. The default values correspond to a
2.2-kW 400-V motor drive.

"""
from __future__ import annotations
from dataclasses import dataclass, field
import numpy as np
from motulator.helpers import abc2complex


# %%
@dataclass
class Inverter:
    """
    Inverter with constant DC-bus voltage and switching-cycle averaging.

    Parameters
    ----------
    u_dc0 : float
        DC-bus voltage.

    """
    u_dc0: float = 540
    # Here q is the duty ratio vector. In the subclasses where the PWM is
    # modeled, the variable `q` refers to the switching state vector.
    q: complex = field(repr=False, default=0j)

    @staticmethod
    def ac_voltage(q, u_dc):
        """
        Compute the AC-side voltage of a lossless inverter.

        Parameters
        ----------
        q : complex
            Duty ratio vector (switching state vector in the subclasses).
        u_dc : float
            DC-bus voltage.

        Returns
        -------
        u_ac : complex
            AC-side voltage.

        """
        u_ac = q*u_dc
        return u_ac

    @staticmethod
    def dc_current(q, i_ac):
        """
        Compute the DC-side current of a lossless inverter.

        Parameters
        ----------
        q : complex
            Duty ratio vector (switching state vector in subclasses).
        i_ac : complex
            AC-side current.

        Returns
        -------
        i_dc : float
            DC-side current.

        """
        i_dc = 1.5*np.real(q*np.conj(i_ac))
        return i_dc

    @staticmethod
    def pwm(d_abc):
        """
        Zero-order hold of the duty ratios over the sampling period.

        The output arrays are compatible with the solver.

        Parameters
        ----------
        d_abc : array_like of floats, shape (3,)
            Duty ratios in the range [0, 1].

        Returns
        -------
        tn_sw : ndarray, shape (1,2)
            Normalized switching instant, tn_sw = [0, 1].
        q : complex ndarray, shape (1,)
            Duty ratio vector, having a shape compatible with the solver.

        """
        tn_sw = np.array([[0, 1]])
        q = np.array([1])*abc2complex(d_abc)
        return tn_sw, q

    def meas_dc_voltage(self):
        """
        Measure the DC-bus voltage.

        Returns
        -------
        float
            DC-bus voltage.

        """
        return self.u_dc0


# %%
@dataclass
class PWMInverter(Inverter):
    """
    PWM inverter with constant DC-bus voltage.

    This extends the Inverter class with pulse-width modulation.

    Parameters
    ----------
    N : int, optional
        Amount of PWM quantization levels. The default is 2**12.

    """
    N: int = 2**12
    # Stores the carrier direction
    falling_edge: bool = field(repr=False, default=False)

    def pwm(self, d_abc):
        """
        Compute the normalized switching instants and the switching states.

        Parameters
        ----------
        d_abc : array_like of floats, shape (3,)
            Duty ratios in the range [0, 1].

        Returns
        -------
        tn_sw : ndarray, shape (4,2)
            Normalized switching instants, tn_sw = [0, t1, t2, t3, 1].
        q : complex ndarray, shape (4,)
            Switching state vectors corresponding to the switching instants.
            For example, the switching state q[1] is applied at the interval
            tn_sw[1].

        Notes
        -----
        Switching instants t_sw split the sampling period T_s into
        four spans. No switching (e.g. da = 0 or da = 1) or simultaneous
        switching instants (e.g da == db) lead to zero spans, i.e.,
        t_sw[i] == t_sw[i].

        """
        # Quantize the duty ratios to N levels
        d_abc = np.round(self.N*np.asarray(d_abc))/self.N
        # Initialize the normalized switching instant array
        tn_sw = np.zeros((4, 2))
        tn_sw[3, 1] = 1
        # Could be understood as a carrier comparison
        if self.falling_edge:
            # Normalized switching instants (zero crossing instants)
            tn_sw[1:4, 0] = np.sort(d_abc)
            tn_sw[0:3, 1] = tn_sw[1:4, 0]
            # Compute the switching state array
            q_abc = (tn_sw[:, 0] < d_abc[:, np.newaxis]).astype(int)
        else:
            # Rising edge
            tn_sw[1:4, 0] = np.sort(1 - d_abc)
            tn_sw[0:3, 1] = tn_sw[1:4, 0]
            q_abc = (tn_sw[:, 0] >= 1 - d_abc[:, np.newaxis]).astype(int)
        # Change the carrier direction for the next call
        self.falling_edge = not self.falling_edge
        # Switching state space vector
        q = abc2complex(q_abc)
        return tn_sw, q


# %%
@dataclass
class FrequencyConverter(PWMInverter):
    """
    Frequency converter.

    This extends the PWMInverter class with models for a strong grid, a
    three-phase diode-bridge rectifier, an LC filter, and a three-phase
    inverter.

    Parameters
    ----------
    L : float
        DC-bus inductance.
    C : float
        DC-bus capacitance.
    U_g : float
        Grid voltage (line-line, rms).
    f_g : float
        Grid frequency.

    """
    L: float = 2e-3
    C: float = 235e-6
    U_g: float = 400
    f_g: float = 50
    # Initial value of the DC-bus inductor current
    i_L0: float = field(repr=False, default=0)
    # Initial value of the DC-bus voltage
    u_dc0: float = field(repr=False, init=False)
    # Peak-valued line-neutral grid voltage
    u_g: float = field(repr=False, init=False)
    # Grid angular frequeyncy
    w_g: float = field(repr=False, init=False)

    def __post_init__(self):
        self.u_dc0 = np.sqrt(2)*self.U_g
        self.u_g = np.sqrt(2/3)*self.U_g
        self.w_g = 2*np.pi*self.f_g

    def grid_voltages(self, t):
        """
        Compute three-phase grid voltages.

        Parameters
        ----------
        t : float
            Time.

        Returns
        -------
        u_g_abc : ndarray of floats, shape (3,)
            The phase voltages.

        """
        theta_g = self.w_g*t
        u_g_abc = self.u_g*np.array([np.cos(theta_g),
                                     np.cos(theta_g - 2*np.pi/3),
                                     np.cos(theta_g - 4*np.pi/3)])
        return u_g_abc

    def f(self, t, u_dc, i_L, i_dc):
        """
        Compute the state derivatives.

        Parameters
        ----------
        t : float
            Time.
        u_dc : float
            DC-bus voltage over the capacitor.
        i_L : float
            DC-bus inductor current.
        i_dc : float
            Current to the inverter.

        Returns
        -------
        list, length 2
            Time derivative of the state vector, [du_dc, d_iL]

        """
        # Grid phase voltages
        u_g_abc = self.grid_voltages(t)
        # Output voltage of the diode bridge
        u_di = np.amax(u_g_abc, 0) - np.amin(u_g_abc, 0)
        # State derivatives
        du_dc = (i_L - i_dc)/self.C
        di_L = (u_di - u_dc)/self.L
        # The inductor current cannot be negative due to the diode bridge
        if i_L < 0 and di_L < 0:
            di_L = 0
        return [du_dc, di_L]
