"""AC port of a load and the voltage of a converter with blocked gates."""

from collections.abc import Callable
from dataclasses import dataclass
from math import inf
from typing import Any

import numpy as np

from motulator.common.utils._utils import complex2abc

# Time constant (s) at which the residual current of a floating leg decays
_TAU = 1e-4
# Tolerances of the diode-state transitions: current (A) and voltage (V)
_I_TOL = 1e-9
_U_TOL = 1e-9
# Transformations between the phase quantities and the real and imaginary parts of
# the space vectors, as matrices
_A = np.array([[1.0, 0.0], [-0.5, np.sqrt(3) / 2], [-0.5, -np.sqrt(3) / 2]])
_B = 2 / 3 * _A.T


# %%
@dataclass
class ACPort:
    """
    Relation between the terminal current and voltage of a load.

    The current derivative of the load at the converter terminals is ``di/dt = Y @ (u
    - e)`` in stationary coordinates, where `u` is the converter voltage. The matrix
    `Y` acts on the real and imaginary parts of the space vectors. The converter uses
    the port to compute its voltage when its gates are blocked.

    Parameters
    ----------
    i : complex
        Current (A).
    e : complex
        Voltage (V) at which the current derivative is zero.
    Y : ndarray, shape (2, 2)
        Inverse differential inductance matrix (1/H).

    """

    i: complex
    e: complex
    Y: np.ndarray


def jacobian(fcn: Callable[[complex], Any], x: complex, h: float = 1e-4) -> np.ndarray:
    """
    Jacobian of a complex-valued function of a complex variable.

    Parameters
    ----------
    fcn : Callable[[complex], Any]
        Function, whose value is converted to a complex number.
    x : complex
        Point at which the Jacobian is evaluated.
    h : float, optional
        Step of the central differences, defaults to 1e-4.

    Returns
    -------
    ndarray, shape (2, 2)
        Jacobian acting on the real and imaginary parts.

    """
    d_x = (complex(fcn(x + h)) - complex(fcn(x - h))) / (2 * h)
    d_y = (complex(fcn(x + 1j * h)) - complex(fcn(x - 1j * h))) / (2 * h)
    return np.array([[d_x.real, d_y.real], [d_x.imag, d_y.imag]])


# %%
def leg_voltages(port: ACPort, u_dc: float, diodes: np.ndarray) -> np.ndarray:
    """
    Compute the leg voltages of a converter with blocked gates.

    The diode state of each leg is 1 if its lower diode conducts (the leg is clamped
    to the negative rail), -1 if its upper diode conducts (positive rail), and 0 if
    the leg floats. The voltage of a floating leg keeps its current at zero, i.e., it
    drives the residual current to zero.

    Parameters
    ----------
    port : ACPort
        Relation between the terminal current and voltage of the load.
    u_dc : float
        Voltage (V) between the rails.
    diodes : ndarray, shape (3,)
        Diode states of the legs.

    Returns
    -------
    ndarray, shape (3,)
        Leg voltages (V) with respect to the negative rail.

    """
    floating = diodes == 0
    if np.all(floating):
        i = np.array([port.i.real, port.i.imag])
        u = port.e - complex(*np.linalg.solve(port.Y, i)) / _TAU
        u_abc = complex2abc(u)
        return u_abc + (u_dc - u_abc.max() - u_abc.min()) / 2  # Centered
    u_abc = np.where(diodes < 0, u_dc, 0.0)
    if np.any(floating):
        # The current derivatives are affine in the leg voltages, M @ u_abc + c, from
        # which the voltages of the floating legs are solved
        M = _A @ port.Y @ _B
        c = -_A @ (port.Y @ [port.e.real, port.e.imag])
        f, g = floating, ~floating
        rhs = -complex2abc(port.i)[f] / _TAU - c[f] - M[np.ix_(f, g)] @ u_abc[g]
        u_abc[f] = np.linalg.solve(M[np.ix_(f, f)], rhs)
    return u_abc


def diode_margin(
    port: ACPort, u_dc: float, diodes: np.ndarray, u_abc: np.ndarray
) -> float:
    """
    Compute the margin of the diode states.

    The margin is positive while the diode states are valid, and it reaches zero
    when the current of a conducting leg reverses or the voltage of a floating leg
    reaches a rail, i.e., at a transition of the diode states.

    Parameters
    ----------
    port : ACPort
        Relation between the terminal current and voltage of the load.
    u_dc : float
        Voltage (V) between the rails.
    diodes : ndarray, shape (3,)
        Diode states of the legs, see `leg_voltages`.
    u_abc : ndarray, shape (3,)
        Leg voltages (V).

    Returns
    -------
    float
        Margin (A or V).

    """
    conducting = diodes != 0
    margin = inf
    if np.any(conducting):
        i_abc = complex2abc(port.i)
        margin = np.min((diodes * i_abc)[conducting]) + _I_TOL
    if not np.all(conducting):
        u_margin = np.minimum(u_abc, u_dc - u_abc)[~conducting]
        margin = min(margin, np.min(u_margin) + _U_TOL)
    return float(margin)


def update_diodes(
    port: ACPort, u_dc: float, diodes: np.ndarray | None = None
) -> np.ndarray:
    """
    Update the diode states at a transition or latch them when the gates are blocked.

    A conducting leg whose current has reversed starts to float, and a floating leg
    whose voltage is outside the rails is clamped to the rail by its diode. When the
    gates are blocked, the legs carrying current conduct in the direction of the
    current, and the other legs float.

    Parameters
    ----------
    port : ACPort
        Relation between the terminal current and voltage of the load.
    u_dc : float
        Voltage (V) between the rails.
    diodes : ndarray, shape (3,), optional
        Diode states of the legs, see `leg_voltages`. Defaults to None, which latches
        the states from the directions of the currents.

    Returns
    -------
    ndarray, shape (3,)
        Updated diode states.

    """
    i_abc = complex2abc(port.i)
    if diodes is None:
        diodes = np.where(np.abs(i_abc) > _I_TOL, np.sign(i_abc), 0.0)
    else:
        diodes = np.where(diodes * i_abc <= 0, 0.0, diodes)
    # The current circulates between the legs conducting in opposite directions.
    # Otherwise, the conducting legs carry only the residual currents of the floating
    # legs, so they float too.
    if not (np.any(diodes > 0) and np.any(diodes < 0)):
        diodes = np.zeros(3)
    for _ in range(3):  # Each round clamps at least one leg
        u_abc = leg_voltages(port, u_dc, diodes)
        clamp = (diodes == 0) & ((u_abc < 0) | (u_abc > u_dc))
        if not np.any(clamp):
            break
        diodes[clamp] = np.where(u_abc[clamp] < 0, 1, -1)
    return diodes
