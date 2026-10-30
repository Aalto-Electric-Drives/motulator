"""Three-phase voltage source model with signal injection."""

from collections.abc import Callable
from math import pi, sqrt
from typing import Any

import numpy as np

from motulator.common.model import ThreePhaseSource
from motulator.common.utils._utils import get_value


class ThreePhaseSourceWithSignalInjection(ThreePhaseSource):
    """
    Three-phase source model with signal injection in synchronous coordinates.

    The excitation ``u_e*cos(2*pi*f_e*t)`` is added to the positive-sequence component
    in the synchronous coordinates of the source. This model is intended for converter
    output admittance measurement.

    Parameters
    ----------
    w_g : float | Callable[[float], float], optional
        Angular frequency (rad/s), defaults to 2*pi*50.
    e_g : float | Callable[[float], float], optional
        Peak-valued magnitude of positive-sequence component, defaults to sqrt(2/3)*400.
    phi : float | Callable[[float], float], optional
        Phase shift (rad) of positive-sequence component, defaults to 0.
    u_e : complex, optional
        Excitation (V) in synchronous coordinates, defaults to 0.
    f_e : float, optional
        Excitation frequency (Hz), defaults to 0.

    """

    def __init__(
        self,
        w_g: float | Callable[[float], float] = 2 * pi * 50,
        e_g: float | Callable[[float], float] = sqrt(2 / 3) * 400,
        phi: float | Callable[[float], float] = 0.0,
        u_e: complex = 0j,
        f_e: float = 0.0,
    ) -> None:
        super().__init__(w_g, e_g, phi)
        self.u_e = u_e
        self.f_e = f_e

    def generate_space_vector(self, t: Any, exp_j_theta_g: Any) -> Any:
        """Generate the space vector in stationary coordinates."""
        u_e = self.u_e * np.cos(2 * pi * self.f_e * t)
        exp_j_theta = exp_j_theta_g * np.exp(1j * get_value(self.phi, t))
        return super().generate_space_vector(t, exp_j_theta_g) + u_e * exp_j_theta
