"""
Continuous-time model for an output LC filter.

The space vector model is implemented in stator coordinates. 

"""
from motulator._helpers import complex2abc


# %%
class LCFilter:
    """
    LC-filter model.

    Parameters
    ----------
    L : float
        Inductance (H).
    C : float
        Capacitance (F). 
    R : float, optional
        Series resistance (Ω) of the inductor. The default is 0.
   
    """

    def __init__(self, L, C, R=0):
        self.L, self.C, self.R, = L, C, R
        self.i_cs0, self.u_ss0 = 0j, 0j

    def f(self, i_cs, u_ss, u_cs, i_ss):
        """
        Compute the state derivative.

        Parameters
        ----------
        i_cs : complex
            Converter output current (A).
        u_ss : complex
            Capacitor (stator) voltage (V).
        u_cs : complex
            Converter output voltage (V).
        i_ss : complex
            Stator current (A).

        Returns
        -------
        complex list, length 2
            Time derivative of the state vector, [d_i_cs, d_u_ss]

        """

        d_i_cs = (u_cs - u_ss - self.R*i_cs)/self.L
        d_u_ss = (i_cs - i_ss)/self.C
        return [d_i_cs, d_u_ss]

    def meas_currents(self):
        """
        Returns the converter phase currents at the end of the sampling period.

        Returns
        -------
        i_c_abc : 3-tuple of floats
            Phase currents (A).

        """
        i_c_abc = complex2abc(self.i_cs0)
        return i_c_abc

    def meas_voltages(self):
        """
        Returns the capacitor (stator) phase voltages at the end of the 
        sampling period.

        Returns
        -------
        u_s_abc : 3-tuple of floats
            Phase voltages (V).

        """
        u_s_abc = complex2abc(self.u_ss0)
        return u_s_abc
