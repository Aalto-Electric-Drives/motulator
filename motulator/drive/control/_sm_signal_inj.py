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
from motulator.drive.utils._parameters import SynchronousMachinePars


# %%
@dataclass
class PLLOutputSignals(ObserverOutputs):
    """Feedback signals for signal-injection control."""

    i_s_flt: complex = 0j


@dataclass
class PLLStates:
    """State estimates."""

    theta_m: float = 0.0
    w_m: float = 0.0


@dataclass
class PLLWorkspace:
    """Workspace variables."""

    eps: float = 0.0
    d_theta_m: float = 0.0
    old_i_s: complex = 0j
    older_i_s: complex = 0j


class PhaseLockedLoop:
    """
    Simple phase-locked loop for rotor-position estimation.

    Parameters
    ----------
    par : SynchronousMachinePars
        Machine model parameters.
    alpha_o : float
        Pole location (rad/s).
    U_inj : float, optional
        Injected voltage amplitude (V).
    T_s : float
      Sampling period (s).

    """

    def __init__(
        self, par: SynchronousMachinePars, alpha_o: float, U_inj: float, T_s: float
    ) -> None:
        self.T_s = T_s
        self.k_p = 2 * alpha_o
        self.k_i = alpha_o**2
        L_s = par.incr_ind_mat(0j)
        self.k = 0.5 * L_s[0, 0] * L_s[1, 1] / (L_s[1, 1] - L_s[0, 0])  # Error gain
        self.u_sd_inj = U_inj
        self.state = PLLStates()
        self._work = PLLWorkspace()
        self.par = par
        self.sensorless = True  # For compatibility reasons

    def compute_output(self, u_s_ab: complex, i_s_ab: complex) -> PLLOutputSignals:
        """Compute output."""
        out = PLLOutputSignals(
            theta_m=self.state.theta_m, theta_c=self.state.theta_m, w_m=self.state.w_m
        )
        out.w_M = out.w_m / self.par.n_p

        # Current and voltage vectors in (estimated) rotor coordinates
        out.i_s = exp(-1j * out.theta_m) * i_s_ab
        out.u_s = exp(-1j * out.theta_m) * u_s_ab

        # Filter the current signal
        out.i_s_flt = 0.5 * (out.i_s + self._work.old_i_s)
        d_i_sq = (out.i_s - 2 * self._work.old_i_s + self._work.older_i_s).imag

        # Internal variables, not needed elsewhere, safe to update here
        self._work.older_i_s = self._work.old_i_s
        self._work.old_i_s = out.i_s

        # Coordinate system angular frequency
        out.w_c = out.w_m + self.k_p * self._work.eps
        self._work.d_theta_m = out.w_c

        # Error signal
        self._work.eps = (
            self.k * d_i_sq / (self.u_sd_inj * self.T_s)
            if abs(self.u_sd_inj) > 0
            else 0
        )

        return out

    def update(self, T_s: float) -> None:
        """Update the states."""
        self.state.theta_m = wrap(self.state.theta_m + T_s * self._work.d_theta_m)
        self.state.w_m += T_s * self.k_i * self._work.eps
        self.u_sd_inj = -self.u_sd_inj  # Reverse the d-axis injection voltage
        self.T_s = T_s


# %%
class SignalInjectionController(CurrentVectorController):
    """
    Sensorless controller with signal injection for synchronous machine drives.

    This class implements a square-wave signal injection for low-speed operation
    according to [#Kim2012]_. A simple phase-locked loop is used to track the rotor
    position. For a wider speed range, signal injection could be combined to a model-
    based observer. The effects of magnetic saturation are not compensated for in this
    version.

    Parameters
    ----------
    par : SynchronousMachinePars
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

    """

    def __init__(
        self,
        par: SynchronousMachinePars,
        cfg: CurrentVectorControllerCfg,
        alpha_o: float = 2 * pi * 40,
        U_inj: float = 250,
        T_s: float = 125e-6,
    ) -> None:
        super().__init__(par, cfg, True, T_s)
        self.observer = PhaseLockedLoop(par, alpha_o, U_inj, T_s)

    def compute_output(self, tau_M_ref: float, fbk: ObserverOutputs) -> References:
        if not isinstance(fbk, PLLOutputSignals):
            raise TypeError
        ref = References(T_s=self.T_s, tau_M=tau_M_ref)
        ref.psi_s, ref.tau_M = self.reference_gen.compute_flux_and_torque_refs(
            ref.tau_M, fbk.w_m, fbk.u_dc
        )
        ref.i_s = self.reference_gen.compute_current_ref(ref.psi_s, ref.tau_M)
        ref.u_s = (
            self.current_ctrl.compute_output(ref.i_s, fbk.i_s_flt)
            + self.observer.u_sd_inj
        )
        ref.u_s_ab = exp(1j * fbk.theta_c) * ref.u_s
        return ref

    def post_process(self, ts: TimeSeries) -> None:
        """Post-process controller time series."""
