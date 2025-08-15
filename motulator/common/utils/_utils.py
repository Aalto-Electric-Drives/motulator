"""Helper functions and classes."""

from dataclasses import dataclass
from typing import Any, Callable

import numpy as np


# %%
def abc2complex(u: Any) -> Any:
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
    >>> from motulator.common.utils import abc2complex
    >>> y = abc2complex([1, 2, 3])
    >>> y
    (-1-0.5773502691896258j)

    """
    return (2 / 3) * u[0] - (u[1] + u[2]) / 3 + 1j * (u[1] - u[2]) / np.sqrt(3)


# %%
def complex2abc(u: Any) -> np.ndarray:
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
    >>> from motulator.common.utils import complex2abc
    >>> y = complex2abc(1-.5j)
    >>> y
    array([ 1.       , -0.9330127, -0.0669873])

    """
    return np.array(
        [
            u.real,
            0.5 * (-u.real + np.sqrt(3) * u.imag),
            0.5 * (-u.real - np.sqrt(3) * u.imag),
        ]
    )


# %%
class SequenceGenerator:
    """
    Sequence generator.

    The time array must be increasing. The output values are interpolated between the
    data points.

    Parameters
    ----------
    times : ndarray
        Time values.
    values : ndarray
        Output values.
    periodic : bool, optional
        Enables periodicity, defaults to False.

    """

    def __init__(self, times: Any, values: Any, periodic: bool = False) -> None:
        self.times = times
        self.values = values
        if periodic is True:
            self._period = times[-1] - times[0]
        else:
            self._period = None

    def __call__(self, t: float) -> Any:
        """Interpolate the output."""
        return np.interp(t, self.times, self.values, period=self._period)


# %%
class Step:
    """
    Step function.

    Parameters
    ----------
    step_time : float
        Time of the step.
    step_value : float
        Value of the step.
    initial_value : float, optional
        Initial value, defaults to 0.

    """

    def __init__(
        self, step_time: float, step_value: float, initial_value: float = 0.0
    ) -> None:
        self.step_time = step_time
        self.step_value = step_value
        self.initial_value = initial_value

    def __call__(self, t: float) -> float:
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
        return self.initial_value + (t >= self.step_time) * self.step_value


# %%
def wrap(theta: float) -> Any:
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

    return np.mod(theta + np.pi, 2 * np.pi) - np.pi


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
    tau : float, optional
        Torque (Nm), defaults to 0.

    """

    U: float
    I: float  # noqa: E741
    f: float
    P: float
    tau: float = 0.0


# %%
@dataclass
class BaseValues:
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
    C : float
        Capacitance (F).
    tau : float, optional
        Torque (Nm), defaults to 0.
    n_p : int, optional
        Number of pole pairs, defaults to 0.
    w_M : float, optional
        Mechanical angular frequency (rad/s), defaults to 0.

    """

    u: float
    i: float
    w: float
    psi: float
    p: float
    Z: float
    L: float
    C: float
    tau: float = 0.0
    n_p: int = 0
    w_M: float = 0.0

    @classmethod
    def from_nominal(cls, nom: NominalValues, n_p: int | None = None) -> "BaseValues":
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
        n_p : int | None, optional
            Number of pole pairs, defaults to None.

        Returns
        -------
        BaseValues
            Base values.

        Notes
        -----
        Notice that the nominal torque is larger than the base torque due to the power
        factor and efficiency being less than unity.

        """
        u = np.sqrt(2 / 3) * nom.U
        i = np.sqrt(2) * nom.I
        w = 2 * np.pi * nom.f
        psi = u / w
        p = 1.5 * u * i
        Z = u / i
        L = Z / w
        C = 1 / (Z * w)

        if n_p is None:
            return cls(u=u, i=i, w=w, psi=psi, p=p, Z=Z, L=L, C=C)
        tau = n_p * p / w
        w_M = w / n_p
        return cls(
            u=u, i=i, w=w, psi=psi, p=p, Z=Z, L=L, C=C, tau=tau, n_p=n_p, w_M=w_M
        )

    @classmethod
    def unity(cls) -> "BaseValues":
        """Create base values with all values set to 1."""
        return cls(u=1, i=1, w=1, psi=1, p=1, Z=1, L=1, C=1, tau=1, n_p=1, w_M=1)


# %%
def get_value(u: Any | Callable[[Any], Any] | None, x: Any) -> Any:
    """
    Helper to get the value of an object that is either callable or float.

    Parameters
    ----------
    u : Any | Callable[[Any], Any] | None
        Input object.
    x : Any
        Argument to the callable object.

    Returns
    -------
    Any
        Values of `u(x)` if callable, otherwise `u`.

    """
    if u is None:
        raise ValueError("Input value cannot be None")
    return u(x) if callable(u) else u


# %%
def empty_array() -> np.ndarray:
    """Return an empty array."""
    return np.array([])


# %%
def clip(value: float, min_value: float, max_value: float) -> float:
    """Clip a value between minimum and maximum."""
    return max(min(value, max_value), min_value)


# %%
def sign(x: float) -> int:
    """Return the sign of x: -1 for negative, 0 for zero, 1 for positive."""
    return -1 if x < 0 else (1 if x > 0 else 0)
