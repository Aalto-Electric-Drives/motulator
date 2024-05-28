"""Helper functions and classes."""

# %%
from dataclasses import dataclass
import numpy as np


# %%
def abc2complex(u):
    """
    Transform three-phase quantities to a complex space vector.

    Parameters
    ----------
    u : array_like, shape (3,)
        Phase quantities.

    Returns
    -------
    complex
        Complex space vector (peak-value scaling).

    Examples
    --------
    >>> from motulator import abc2complex
    >>> y = abc2complex([1, 2, 3])
    >>> y
    (-1-0.5773502691896258j)

    """
    return (2/3)*u[0] - (u[1] + u[2])/3 + 1j*(u[1] - u[2])/np.sqrt(3)


# %%
def complex2abc(u):
    """
    Transform a complex space vector to three-phase quantities.

    Parameters
    ----------
    u : complex
        Complex space vector (peak-value scaling).

    Returns
    -------
    ndarray, shape (3,)
        Phase quantities.

    Examples
    --------
    >>> from motulator import complex2abc
    >>> y = complex2abc(1-.5j)
    >>> y
    array([ 1.       , -0.9330127, -0.0669873])

    """
    return np.array([
        u.real, .5*(-u.real + np.sqrt(3)*u.imag),
        .5*(-u.real - np.sqrt(3)*u.imag)
    ])


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
    tau : float
        Torque (Nm).

    """

    U: float
    I: float
    f: float
    P: float
    tau: float


# %%
@dataclass
class BaseValues:
    # pylint: disable=too-many-instance-attributes
    """
    Base values.

    Parameters
    ----------
    u : float
        Voltage (V, peak, line-neutral).
    i : float
        Current (A, peak).
    w : float
        Angular frequency (rad/s).
    psi : float
        Flux linkage (Vs).
    p : float
        Power (W).
    Z : float
        Impedance (Ω).
    L : float
        Inductance (H).
    tau : float
        Torque (Nm).
    n_p : int
        Number of pole pairs.

    """
    u: float
    i: float
    w: float
    psi: float
    p: float
    Z: float
    L: float
    tau: float
    n_p: int

    @classmethod
    def from_nominal(cls, nom, n_p):
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

        n_p : int
            Number of pole pairs.

        Returns
        -------
        BaseValues
            Base values.

        Notes
        -----
        Notice that the nominal torque is larger than the base torque due to 
        the power factor and efficiency being less than unity.

        """
        u = np.sqrt(2/3)*nom.U
        i = np.sqrt(2)*nom.I
        w = 2*np.pi*nom.f
        psi = u/w
        p = 1.5*u*i
        Z = u/i
        L = Z/w
        tau = n_p*p/w

        return cls(u=u, i=i, w=w, psi=psi, p=p, Z=Z, L=L, tau=tau, n_p=n_p)


# %%
class Sequence:
    """
    Sequence generator.

    The time array must be increasing. The output values are interpolated
    between the data points.

    Parameters
    ----------
    times : ndarray
        Time values.
    values : ndarray
        Output values.
    periodic : bool, optional
        Enables periodicity. The default is False.

    """

    def __init__(self, times, values, periodic=False):
        self.times = times
        self.values = values
        if periodic is True:
            self._period = times[-1] - times[0]
        else:
            self._period = None

    def __call__(self, t):
        """
        Interpolate the output.

        Parameters
        ----------
        t : float
            Time.

        Returns
        -------
        float or complex
            Interpolated output.

        """
        return np.interp(t, self.times, self.values, period=self._period)


# %%
class Step:
    """Step function."""

    def __init__(self, step_time, step_value, initial_value=0):
        self.step_time = step_time
        self.step_value = step_value
        self.initial_value = initial_value

    def __call__(self, t):
        """
        Step function.

        Parameters
        ----------
        t : float
            Time.

        Returns
        -------
        float
            Step output.

        """
        return self.initial_value + (t >= self.step_time)*self.step_value


# %%
def wrap(theta):
    """
    Limit the angle into the range [-pi, pi).

    Parameters
    ----------
    theta : float
        Angle (rad).

    Returns
    -------
    float
        Limited angle.

    """

    return np.mod(theta + np.pi, 2*np.pi) - np.pi
