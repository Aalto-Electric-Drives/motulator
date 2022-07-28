# pylint: disable=invalid-name
"""
Power converter models.

An inverter with constant DC-bus voltage and a frequency converter with a diode
front-end rectifier are modeled. Complex space vectors are used also for duty
ratios and switching states, wherever applicable. In this module, all space
vectors are in stationary coordinates. The default values correspond to a
2.2-kW 400-V motor drive.

"""
from __future__ import annotations
import numpy as np


# %%
class Inverter:
    """
    Inverter with constant DC-bus voltage and switching-cycle averaging.

    Parameters
    ----------
    u_dc : float
        DC-bus voltage.

    """

    def __init__(self, u_dc=540):
        self.u_dc0 = u_dc
        self.q = 0j  # Switching state vector

    @staticmethod
    def ac_voltage(q, u_dc):
        """
        Compute the AC-side voltage of a lossless inverter.

        Parameters
        ----------
        q : complex
            Switching state vector.
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
            Switching state vector.
        i_ac : complex
            AC-side current.

        Returns
        -------
        i_dc : float
            DC-side current.

        """
        i_dc = 1.5*np.real(q*np.conj(i_ac))
        return i_dc

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
class FrequencyConverter(Inverter):
    """
    Frequency converter.

    This extends the Inverter class with models for a strong grid, a
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

    def __init__(self, L=2e-3, C=235e-6, U_g=400, f_g=50):
        # pylint: disable=super-init-not-called
        self.L, self.C = L, C
        # Initial value of the DC-bus inductor current
        self.i_L0 = 0
        # Initial value of the DC-bus voltage
        self.u_dc0 = np.sqrt(2)*U_g
        # Peak-valued line-neutral grid voltage
        self.u_g = np.sqrt(2/3)*U_g
        # Grid angular frequeyncy
        self.w_g = 2*np.pi*f_g
        # Switching state vector
        self.q = 0j

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
        u_g_abc = self.u_g*np.array(
            [
                np.cos(theta_g),
                np.cos(theta_g - 2*np.pi/3),
                np.cos(theta_g - 4*np.pi/3)
            ])
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
