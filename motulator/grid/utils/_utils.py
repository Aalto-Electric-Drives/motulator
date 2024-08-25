"""Common dataclasses usable in models and control of grid converters."""

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


# %%
@dataclass
class FilterPars(ABC):
    """
    Filter parameters

    Parameters
    ----------
    L_fc : float
        Converter-side inductance of the filter (H).
    L_fg : float, optional
        Grid-side inductance of the filter (H). The default is 0.
    C_f : float, optional
        Filter capacitance (F). The default is 0.
    R_fc : float, optional
        Converter-side series resistance (Ω). The default is 0.
    R_fg : float, optional
        Grid-side series resistance (Ω). The default is 0.

    """
    L_fc: float
    L_fg: float = 0
    C_f: float = 0
    R_fc: float = 0
    R_fg: float = 0
