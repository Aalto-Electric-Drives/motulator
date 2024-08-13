"""Common dataclasses usable in models and control of grid converters."""

# %%
from abc import ABC
from dataclasses import dataclass


@dataclass
class GridPars(ABC):
    """
    Class for grid parameters

    Parameters
    ----------
    u_gN : float
        Nominal grid voltage, phase-to-ground peak value (V).
    w_gN : float
        Nominal grid angular frequency (rad/s).
    L_g : float, optional
        Grid inductance (H). The default is 0.
    R_g : float, optional
        Grid resistance (Ω). The default is 0.

    """
    u_gN: float = None
    w_gN: float = None
    L_g: float = 0
    R_g: float = 0
