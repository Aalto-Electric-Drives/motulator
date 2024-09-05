"""Common dataclasses usable in models and control of grid converters."""

from dataclasses import dataclass


# %%
@dataclass
class ACFilterPars:
    """
    AC filter and grid impedance parameters.

    Parameters
    ----------
    L_fc : float
        Converter-side filter inductance (H).
    L_fg : float, optional
        Grid-side filter inductance (H). The default is 0.
    C_f : float, optional
        Filter capacitance (F). The default is 0.
    R_fc : float, optional
        Series resistance (Ω) of the converter-side inductor. The default is 0.
    R_fg : float, optional
        Series resistance (Ω) of the grid-side inductor. The default is 0.
    L_g : float, optional
        Grid inductance (H). The default is 0.
    R_g : float, optional
        Grid resistance (Ω). The default is 0.
    u_fs0 : float, optional
        Initial value of the filter capacitor voltage (V). Needed in the case
        of an LCL filter.

    """
    L_fc: float
    L_fg: float = 0
    C_f: float = 0
    R_fc: float = 0
    R_fg: float = 0
    L_g: float = 0
    R_g: float = 0
    u_fs0: float = None
