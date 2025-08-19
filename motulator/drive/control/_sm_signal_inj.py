"""Sensorless control with signal injection for synchronous machine drives."""

from cmath import exp, pi
from dataclasses import dataclass

from motulator.common.control._base import TimeSeries
from motulator.common.utils._utils import wrap
from motulator.drive.control._sm_current_vector import (
    CurrentVectorController,
    CurrentVectorControllerCfg,
    References,
)
from motulator.drive.control._sm_observers import ObserverOutputs
from motulator.drive.utils._parameters import (
    SaturatedSynchronousMachinePars,
    SynchronousMachinePars,
)


# %%
@dataclass
class PLLStates:
    """State estimates."""

    theta_m: float = 0.0  # Electrical rotor angle
    w_m: float = 0.0  # Electrical angular speed of the rotor


class PhaseLockedLoop:
    """
    Simple phase-locked loop for rotor-position estimation.

    Parameters
    ----------
    par : SynchronousMachinePars | SaturatedSynchronousMachinePars
        Machine model parameters.
    alpha_o : float
        Pole location (rad/s).
    U_inj : float, optional
        Injected voltage amplitude (V).
    T_s : float
      Sampling period (s).

    """

    def __init__(
        self,
        par: SynchronousMachinePars | SaturatedSynchronousMachinePars,
        alpha_o: float,
        U_inj: float,
        T_s: float,
    ) -> None:
        self._T_s = T_s
        self.k_p = 2 * alpha_o
        self.k_i = alpha_o**2
        # Constant error gain based on the unsaturated inductances
        L_s = par.incr_ind_mat(0j)
        self.k = 0.5 * L_s[0, 0] / (L_s[1, 1] - L_s[0, 0])
        self.U_inj = U_inj
        self.u_sd_inj = U_inj
        self.state = PLLStates()
        self._old_psi_sq: float = 0.0
        self._older_psi_sq: float = 0.0
        self.par = par
        self.sensorless = True  # For compatibility reasons

    def _compute_demodulation_error(self, i_s: complex) -> float:
        """Compute demodulation error signal based on flux-mapped q-axis current."""
        # Apply the flux map to get q-axis flux linkage
        psi_sq = complex(self.par.psi_s_dq(i_s)).imag

        # Compute second derivative using finite differences
        d_psi_sq = psi_sq - 2.0 * self._old_psi_sq + self._older_psi_sq

        # Compute error signal if injection is active
        if abs(self.u_sd_inj) > 0:
            eps = self.k * d_psi_sq / (self.u_sd_inj * self._T_s)
        else:
            eps = 0.0

        # Update stored values for the next sampling period
        self._older_psi_sq = self._old_psi_sq
        self._old_psi_sq = psi_sq

        return eps

    def compute_output(self, u_s_ab: complex, i_s_ab: complex) -> ObserverOutputs:
        """Compute output."""
        out = ObserverOutputs(
            theta_m=self.state.theta_m, theta_c=self.state.theta_m, w_m=self.state.w_m
        )
        out.w_M = out.w_m / self.par.n_p

        # Current and voltage vectors in (estimated) rotor coordinates
        out.i_s = exp(-1j * out.theta_m) * i_s_ab
        out.u_s = exp(-1j * out.theta_m) * u_s_ab

        # Demodulation (based on flux-mapped q-current to avoid cross-saturation error)
        out.eps = self._compute_demodulation_error(out.i_s)

        # Coordinate system angular frequency
        out.w_c = out.w_m + self.k_p * out.eps

        return out

    def update(self, T_s: float, out: ObserverOutputs) -> None:
        """Update the states."""
        self.state.theta_m = wrap(self.state.theta_m + T_s * out.w_c)
        self.state.w_m += T_s * self.k_i * out.eps
        self.u_sd_inj = (-1 if self.u_sd_inj > 0 else 1) * self.U_inj
        self._T_s = T_s


# %%
class SignalInjectionController(CurrentVectorController):
    """
    Sensorless controller with signal injection for synchronous machine drives.

    This class implements a square-wave signal injection for low-speed operation
    according to [#Kim2012]_. Cross-saturation errors are compensated for using flux
    maps [#You2018]_. A simple phase-locked loop is used to track the rotor position.
    For wider speed range, signal injection could be combined to a model-based observer.

    Parameters
    ----------
    par : SynchronousMachinePars | SaturatedSynchronousMachinePars
        Machine model parameters.
    cfg : CurrentVectorControllerCfg
        Current-vector control configuration.
    alpha_o : float, optional
        Pole location (rad/s) of the phase-locked loop, defaults to 2*pi*40.
    U_inj : float, optional
        Injected voltage amplitude (V), defaults to 250.
    T_s : float, optional
        Sampling period (s), defaults to 125e-6.

    References
    ----------
    .. [#Kim2012] Kim, Ha, Sul, "PWM switching frequency signal injection sensorless
       method in IPMSM," IEEE Trans. Ind. Appl., 2012,
       https://doi.org/10.1109/TIA.2012.2210175

    .. [#You2018] Yousefi-Talouki, Pescetto, Pellegrino, Boldea, "Combined active flux
       and high-frequency injection methods for sensorless direct-flux vector control of
       synchronous reluctance machines," IEEE Trans. Power Electron., 2018,
       https://doi.org/10.1109/TPEL.2017.2697209

    """

    def __init__(
        self,
        par: SynchronousMachinePars | SaturatedSynchronousMachinePars,
        cfg: CurrentVectorControllerCfg,
        alpha_o: float = 2 * pi * 40,
        U_inj: float = 250,
        T_s: float = 125e-6,
    ) -> None:
        super().__init__(par, cfg, True, T_s)
        self.observer = PhaseLockedLoop(par, alpha_o, U_inj, T_s)

    def compute_output(self, tau_M_ref: float, fbk: ObserverOutputs) -> References:
        ref = References(T_s=self.T_s, tau_M=tau_M_ref)
        ref.psi_s, ref.tau_M = self.reference_gen.compute_flux_and_torque_refs(
            ref.tau_M, fbk.w_m, fbk.u_dc
        )
        ref.i_s = self.reference_gen.compute_current_ref(ref.psi_s, ref.tau_M)
        ref.u_s = (
            self.current_ctrl.compute_output(ref.i_s, fbk.i_s) + self.observer.u_sd_inj
        )
        return ref

    def post_process(self, ts: TimeSeries) -> None:
        """Post-process controller time series."""
