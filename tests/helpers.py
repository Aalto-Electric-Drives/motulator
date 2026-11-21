"""Machine parameters and helper functions shared by the tests."""

import numpy as np
from scipy.integrate import trapezoid

from motulator.drive import model


def ipmsm_par(G_c: float = 0.0) -> model.SynchronousMachinePars:
    """Parameters of the 2.2-kW IPMSM."""
    return model.SynchronousMachinePars(
        n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545, G_c=G_c
    )


def im_par(G_c: float = 0.0) -> model.InductionMachineInvGammaPars:
    """Inverse-Γ parameters of the 2.2-kW induction machine."""
    return model.InductionMachineInvGammaPars(
        n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224, G_c=G_c
    )


def time_average(x: np.ndarray, t: np.ndarray, t_start: float) -> float:
    """Average over the unevenly spaced samples after `t_start`."""
    end = t > t_start
    return float(trapezoid(x[end], t[end]) / (t[end][-1] - t[end][0]))
