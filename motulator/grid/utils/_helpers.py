"""Helper functions and classes."""

# %%
from dataclasses import dataclass
import numpy as np

# %%
@dataclass 
class NominalValues:
    """
    Nominal values.

    Parameters
    ----------
    U : float
        Voltage (V, rms, line-line).
    I : float
        Current (A, rms).
    f : float
        Frequency (Hz).
    P : float
        Power (W).


    """

    U: float
    I: float
    f: float
    P: float

# %%
@dataclass
class BaseValues:
    """
    Base values.

    Base values are computed from the nominal values and the number of pole
    pairs. They can be used, e.g., for scaling the plotted waveforms.

    Parameters
    ----------
    
    u : float
        Base voltage (V, peak, line-neutral).
    i : float
        Base current (A, peak).
    w : float
        Base angular frequency (rad/s).
    psi : float
        Base flux linkage (Vs).
    p : float
        Base power (W).
    Z : float
        Base impedance (Ω).
    L : float
        Base inductance (H).

    """
    # pylint: disable=too-many-instance-attributes
    u: float
    i: float
    f: float
    p: float

@classmethod
def from_nominal(cls, nom):
     """
        Compute base values from nominal values.

        Parameters
        ----------
        nom : NominalValues
            Nominal values containing the following fields:

                U : float
                    Voltage (V, rms, line-line).
                I : float
                    Current (A, rms).
                f : float
                    Frequency (Hz).


        Returns
        -------
        BaseValues
            Base values.

        """
    u = np.sqrt(2/3)*nom.U
    i = np.sqrt(2)*nom.I
    w = 2*np.pi*nom.f
    psi = u/w
    p = 1.5*u*i
    Z = u/i
    L = Z/w

    return cls(u=u, i=i, w=w, psi=psi, p=p, Z=Z, L=L)