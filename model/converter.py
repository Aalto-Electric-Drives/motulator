# pylint: disable=C0103
"""
This module contains models for the power converter. The inverter with the
constant DC-bus voltage and the frequency converter with the diode
front-end rectifier are modeled.

"""
import numpy as np


def ac_voltage(q, u_dc):
    """
    Computes the AC-side voltage of a lossless inverter.

    Parameters
    ----------
    q : complex
        Switching state vector (in stator coordinates).
    u_dc : float
        DC-Bus voltage.

    Returns
    -------
    u_ac : complex
        AC-side voltage.

    """
    u_ac = q*u_dc
    return u_ac


def dc_current(q, i_ac):
    """
    Computes the DC-side current of a lossless inverter.

    Parameters
    ----------
    q : complex
        Switching state vector (in stator coordinates).
    i_ac : complex
        AC-side current.

    Returns
    -------
    i_dc : float
        DC-side current.

    """
    i_dc = 1.5*np.real(q*np.conj(i_ac))
    return i_dc


# %%
class Inverter:
    """
    This class represents an inverter fed from the constant DC-voltage source.

    """

    # pylint: disable=R0903
    def __init__(self, u_dc=540):
        """
        Parameters
        ----------
        u_dc : float, optional
            DC-bus voltage. The default is 540.

        """
        self.u_dc0 = u_dc
        self.ac_voltage = ac_voltage

    def meas_dc_voltage(self):
        """
        Returns
        -------
        float
            Measured DC-bus voltage.

        """
        return self.u_dc0

    def __str__(self):
        desc = ('Inverter fed from the constant DC-voltage source:\n'
                '    u_dc={}')
        return desc.format(self.u_dc0)


# %%
class FrequencyConverter:
    """
    This class models a strong grid, a three-phase diode-bridge rectifier,
    an LC filter, and a three-phase inverter.

    """

    def __init__(self, C=235e-6, L=2e-3, u_g=np.sqrt(2/3)*400, w_g=2*np.pi*50):
        """
        Parameters
        ----------

        C : float, optional
            DC-bus capacitance. The default is 235e-6.
        L : float, optional
            DC-bus inductance. The default is 2e-3.
        u_g : float, optional
            Grid voltage (line-neutral, peak). The default is sqrt(2/3)*400.
        w_g : float, optional
            Grid angular frequency. The default is 2*pi*50.

        """
        # Parameters
        self.C, self.L, self.u_g, self.w_g = C, L, u_g, w_g
        # Initial states
        self.u_dc0, self.i_L0 = np.sqrt(3)*self.u_g, 0
        # Methods
        self.ac_voltage = ac_voltage
        self.dc_current = dc_current

    def grid_voltages(self, t):
        """
        Computes the three-phase grid voltages.

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
        Computes the state derivatives.

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

    def meas_dc_voltage(self):
        """
        Returns
        -------
        float
            Measured DC-bus voltage.

        """
        return self.u_dc0

    def __str__(self):
        desc = ('Frequency converter:\n'
                '  U_g={:.0f}  f_g={:.0f}  C={}  L={}')
        return desc.format(self.u_g*np.sqrt(3/2), self.w_g/(2*np.pi),
                           self.C, self.L)
