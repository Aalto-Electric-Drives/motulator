"""AC-port current dynamics and passive inverter terminal voltages."""

from dataclasses import dataclass

import numpy as np

from motulator.common.utils._utils import abc2complex, complex2abc


@dataclass
class ACPort:
    """
    Local load relation ``di/dt = Y @ (u - e)`` in stationary coordinates.

    With ``algebraic=True``, the relation instead gives the current itself. This
    covers loads with direct feedthrough, such as the machine core-loss branch.
    Parameters
    ----------
    i : complex
        Instantaneous AC current space vector (A).
    e : complex
        Voltage space vector (V) giving zero current derivative, or zero current
        for an algebraic port.
    Y : ndarray, shape (2, 2)
        Symmetric positive-definite differential inverse inductance matrix (1/H),
        or conductance matrix (S) for an algebraic port. The matrix acts on the
        real and imaginary components of stationary-coordinate space vectors.
    algebraic : bool, optional
        Use ``i = Y @ (u - e)`` instead of current dynamics, defaults to False.

    """

    i: complex
    e: complex
    Y: np.ndarray
    algebraic: bool = False


def blocked_voltage(port: ACPort, u_dc: float) -> complex:
    """Solve ideal diode constraints, including floating inverter legs.

    Currents below 10 microamperes are treated as zero. Inside this numerical
    boundary layer, a 100-microsecond stabilization brings the residual current
    to zero; outside it, conducting diodes clamp the corresponding poles exactly.
    """
    i_abc = complex2abc(port.i)
    floating = np.abs(i_abc) <= 1e-5
    if port.algebraic or np.count_nonzero(floating) >= 2:
        # At zero current, the feasible terminal voltages form a hexagon. The
        # admittance metric ensures that only forward-biased diodes can turn on.
        e = np.array([port.e.real, port.e.imag])
        if not port.algebraic:
            e -= np.linalg.solve(port.Y, [port.i.real, port.i.imag]) / 1e-4
        e_ab = complex(e[0], e[1])
        if np.ptp(complex2abc(e_ab)) <= u_dc:
            return e_ab
        vertices = (2 / 3) * u_dc * np.exp(1j * np.arange(7) * np.pi / 3)
        points = np.array([vertices.real, vertices.imag]).T
        a, d = points[:-1], np.diff(points, axis=0)
        denom = np.einsum("ni,ij,nj->n", d, port.Y, d)
        numer = np.einsum("ni,ij,nj->n", e - a, port.Y, d)
        fraction = np.divide(numer, denom, out=np.zeros(6), where=denom > 0)
        candidates = a + np.clip(fraction, 0, 1)[:, None] * d
        residual = candidates - e
        cost = np.einsum("ni,ij,nj->n", residual, port.Y, residual)
        u = candidates[np.argmin(cost)]
        return complex(u[0], u[1])

    poles = np.where(i_abc < 0, u_dc, 0.0)
    if np.any(floating):
        # One floating pole: choose its voltage to keep its phase current zero,
        # unless a rail is reached and its diode has to conduct.
        k = int(np.flatnonzero(floating)[0])
        poles[k] = 0
        u0 = abc2complex(poles)
        delta = u0 - port.e
        di0 = port.Y @ [delta.real, delta.imag]
        d0 = complex2abc(complex(di0[0], di0[1]))[k]
        pole_unit = np.eye(3)[k]
        du = abc2complex(pole_unit)
        di = port.Y @ [du.real, du.imag]
        slope = complex2abc(complex(di[0], di[1]))[k]
        poles[k] = np.clip((-i_abc[k] / 1e-4 - d0) / slope, 0, u_dc)
    return complex(abc2complex(poles))
