"""AC and DC ports of loads and the voltages of a blocked converter."""

from collections.abc import Callable
from dataclasses import dataclass
from math import inf
from typing import Any

import numpy as np

from motulator.common.utils._utils import abc2complex, complex2abc

# Time constant (s) at which the residual current of a floating leg decays
_TAU = 1e-4
# Tolerances of the diode-state transitions: current (A) and voltage (V). They exceed
# the numerical noise of the solution, e.g., in the residual currents of floating legs,
# so that the margin of the diode states is positive after a transition.
_I_TOL = 1e-4
_U_TOL = 1e-3
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


@dataclass
class DCPort:
    """
    Relation between the DC current and the terminal voltage of a current-fed load.

    The terminal voltage of the load is ``u = e + R*i + L*di/dt``, where `i` is the
    current fed by the converter. A current-fed converter (e.g., a diode bridge with
    a DC choke) uses the port to solve its DC voltage, see `dc_voltage`.

    Parameters
    ----------
    e : float
        Voltage (V) at zero current and zero current derivative, e.g., the EMF of the
        load.
    R : float
        Series resistance (Ω).
    L : float
        Series inductance (H), e.g., the DC choke.

    """

    e: float
    R: float
    L: float


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
        # Inverse of Y applied to the current, in closed form
        (a, b), (c, d) = port.Y
        i = port.i
        Y_inv_i = complex(d * i.real - b * i.imag, a * i.imag - c * i.real)
        u_abc = complex2abc(port.e - Y_inv_i / (a * d - b * c) / _TAU)
        return u_abc + (u_dc - u_abc.max() - u_abc.min()) / 2  # Centered
    u_abc = np.where(diodes < 0, u_dc, 0.0)
    if np.any(floating):
        # The current derivatives of the floating legs are affine in the leg voltages,
        # A_f @ Y @ (B @ u_abc - e), from which their voltages are solved. They are
        # zero in u_abc here, and usually only one leg floats.
        AY = _A[floating] @ port.Y
        M = AY @ _B
        rhs = AY @ [port.e.real, port.e.imag] - M @ u_abc
        rhs -= complex2abc(port.i)[floating] / _TAU
        M = M[:, floating]
        u_abc[floating] = rhs / M[0, 0] if M.size == 1 else np.linalg.solve(M, rhs)
    return u_abc


def dc_voltage(port: ACPort, dc: DCPort, diodes: np.ndarray) -> float:
    """
    Solve the voltage between the rails of a current-fed bridge.

    The DC current of the bridge is the current of the legs clamped to the positive
    rail (equally, the negative of the current of the legs clamped to the negative
    rail). Its derivative follows from the AC port and the leg voltages, which depend
    on the rail voltage, see `leg_voltages`. The rail voltage satisfying the DC port
    ``u_dc = e + R*i_dc + L*di_dc/dt`` is solved from this linear relation.

    Parameters
    ----------
    port : ACPort
        Relation between the terminal current and voltage of the AC-side load.
    dc : DCPort
        Relation between the DC current and the voltage of the DC-side load.
    diodes : ndarray, shape (3,)
        Diode states of the legs, see `leg_voltages`.

    Returns
    -------
    float
        Voltage (V) between the rails.

    """
    # The DC current is half the sum of the absolute values of the phase currents,
    # whose directions in the conducting legs equal the diode states
    i_dc = 0.5 * np.sum(np.abs(complex2abc(port.i)))
    e = dc.e + dc.R * i_dc
    # The DC current changes only if legs are clamped to both rails
    if dc.L == 0 or not (np.any(diodes > 0) and np.any(diodes < 0)):
        return float(e)

    def d_i_dc(u_dc: float) -> float:
        """Derivative of the DC current for a trial rail voltage."""
        u = abc2complex(leg_voltages(port, u_dc, diodes)) - port.e
        d_i = port.Y @ [u.real, u.imag]
        return 0.5 * float(np.sum(diodes * complex2abc(complex(d_i[0], d_i[1]))))

    # The derivative is affine in the rail voltage
    a = d_i_dc(0.0)
    b = d_i_dc(1.0) - a
    return float((e + dc.L * a) / (1 - dc.L * b))


def diode_margin(
    port: ACPort,
    u_dc: float,
    diodes: np.ndarray,
    u_abc: np.ndarray,
    gated: np.ndarray | None = None,
) -> float:
    """
    Compute the margin of the diode states.

    The margin is positive while the diode states are valid, and it reaches zero
    when the current of a conducting leg reverses or the voltage of a floating leg
    reaches a rail through a gated device, i.e., at a transition of the diode states.

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
    gated : ndarray, shape (2, 3), optional
        Gate signals of the upper and lower devices of the legs, see `update_diodes`.
        Defaults to None, i.e., all the devices are gated.

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
        gate_up, gate_lo = (True, True) if gated is None else (gated[0], gated[1])
        u_lo = np.where(gate_lo, u_abc, inf)
        u_up = np.where(gate_up, u_dc - u_abc, inf)
        u_margin = np.minimum(u_lo, u_up)[~conducting]
        margin = min(margin, np.min(u_margin) + _U_TOL)
    return float(margin)


def current_direction(
    i_abc: np.ndarray, sign: Callable[[np.ndarray], np.ndarray] = np.sign
) -> np.ndarray:
    """
    Directions of the phase currents, given by `sign`.

    A current within the tolerance of the diode-state transitions has the direction
    zero, so that a leg without current, e.g., a floating leg of a blocked converter,
    has no direction.

    Parameters
    ----------
    i_abc : ndarray, shape (3,)
        Phase currents (A).
    sign : Callable[[np.ndarray], np.ndarray], optional
        Function of the phase currents giving their directions, defaults to `np.sign`.

    Returns
    -------
    ndarray, shape (3,)
        Directions of the currents.

    """
    return np.where(np.abs(i_abc) > _I_TOL, sign(i_abc), 0.0)


def update_diodes(
    port: ACPort,
    dc: float | DCPort,
    diodes: np.ndarray | None = None,
    gated: np.ndarray | None = None,
) -> np.ndarray:
    """
    Update the diode states at a transition or latch them when the gates are blocked.

    A conducting leg whose current has reversed starts to float, and a floating leg
    whose voltage is outside the rails is clamped to the rail by its diode. When the
    gates are blocked, the legs carrying current conduct in the direction of the
    current, and the other legs float. A device with a gate signal (e.g., a thyristor)
    clamps the leg only if it is gated, and it conducts until its current reverses.

    Parameters
    ----------
    port : ACPort
        Relation between the terminal current and voltage of the load.
    dc : float | DCPort
        Voltage (V) between the rails, or the DC port of a current-fed bridge, from
        which the voltage is solved, see `dc_voltage`.
    diodes : ndarray, shape (3,), optional
        Diode states of the legs, see `leg_voltages`. Defaults to None, which latches
        the states from the directions of the currents.
    gated : ndarray, shape (2, 3), optional
        Gate signals of the upper and lower devices of the legs. Defaults to None,
        i.e., all the devices are gated (diodes).

    Returns
    -------
    ndarray, shape (3,)
        Updated diode states.

    """
    i_abc = complex2abc(port.i)
    if diodes is None:
        diodes = current_direction(i_abc)
    else:
        diodes = np.where(diodes * i_abc <= 0, 0.0, diodes)
    # The current circulates between the legs conducting in opposite directions.
    # Otherwise, the conducting legs carry only the residual currents of the floating
    # legs, so they float too.
    if not (np.any(diodes > 0) and np.any(diodes < 0)):
        diodes = np.zeros(3)
    gate_up, gate_lo = (True, True) if gated is None else (gated[0], gated[1])
    for _ in range(3):  # Each round clamps at least one leg
        u_dc = dc_voltage(port, dc, diodes) if isinstance(dc, DCPort) else dc
        u_abc = leg_voltages(port, u_dc, diodes)
        floating = diodes == 0
        clamp_lo = floating & (u_abc < 0) & gate_lo
        clamp_up = floating & (u_abc > u_dc) & gate_up
        if not np.any(clamp_lo | clamp_up):
            break
        diodes = np.where(clamp_lo, 1.0, np.where(clamp_up, -1.0, diodes))
    return diodes
