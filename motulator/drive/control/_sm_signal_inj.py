"""Sensorless control with signal injection for synchronous machine drives."""

from cmath import exp
from collections import deque
from typing import cast

from motulator.common.control._base import TimeSeries
from motulator.common.utils._utils import wrap
from motulator.drive.control._common import SpeedObserver
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
class SquareWaveInjection:
    """
    Square-wave signal injection with demodulation.

    This injects a square-wave voltage in the estimated d-axis direction and computes
    the mechanical position error signal by demodulating the current response. Cross-
    saturation errors are compensated for using flux maps.

    Parameters
    ----------
    par : SynchronousMachinePars | SaturatedSynchronousMachinePars
        Machine model parameters.
    U_inj : float
        Injected voltage amplitude (V).
    T_s : float
        Sampling period (s).
    N_inj : int, optional
        Number of sampling periods per injection voltage half-period, defaults to 1.

    """

    def __init__(
        self,
        par: SynchronousMachinePars | SaturatedSynchronousMachinePars,
        U_inj: float,
        T_s: float,
        N_inj: int = 1,
    ) -> None:
        if N_inj < 1:
            raise ValueError("N_inj must be a positive integer")
        self.par = par
        self.U_inj = U_inj
        self.N_inj = N_inj
        # Constant error gain based on the unsaturated inductances
        L_s = par.incr_ind_mat(0j)
        self.k = 0.5 * L_s[0, 0] / (L_s[1, 1] - L_s[0, 0]) / par.n_p
        self.u_sd_inj: float = U_inj
        self._sign: float = 1.0
        self._T_s = T_s
        self._psi_sq_history: deque[float] = deque(maxlen=2 * N_inj + 1)
        self._eps: float = 0.0
        self._inj_counter: int = 0

    def compute_error(self, i_s_ab: complex, theta_m: float) -> float:
        """Compute mechanical position error signal."""
        # Apply the flux map to compensate the cross saturation
        i_s = exp(-1j * theta_m) * i_s_ab
        psi_sq = complex(self.par.psi_s_dq(i_s)).imag
        self._psi_sq_history.append(psi_sq)

        if len(self._psi_sq_history) < 2 * self.N_inj + 1:
            return self._eps
        if self._inj_counter != 0:
            return self._eps

        # Compute the second difference over two injection half-periods
        d_psi_sq = (
            self._psi_sq_history[-1]
            - 2.0 * self._psi_sq_history[-1 - self.N_inj]
            + self._psi_sq_history[-1 - 2 * self.N_inj]
        )

        # Update the error signal at injection voltage transitions
        if abs(self.u_sd_inj) > 0:
            self._eps = self.k * d_psi_sq / (self.u_sd_inj * self.N_inj * self._T_s)
        else:
            self._eps = 0.0

        return self._eps

    def update(self, T_s: float, scale: float = 1.0) -> None:
        """Toggle the injection voltage, whose amplitude is scaled by `scale`."""
        self._inj_counter = (self._inj_counter + 1) % self.N_inj
        if self._inj_counter == 0:
            self._sign = -self._sign
        self.u_sd_inj = self._sign * self.U_inj * scale
        self._T_s = T_s


# %%
class SignalInjectionObserver:
    """
    Signal injection observer for synchronous machine drives.

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
    J : float | None, optional
        Inertia (kgm²), if not None, a speed observer is used.
    N_inj : int, optional
        Number of sampling periods per injection voltage half-period, defaults to 1.

    """

    def __init__(
        self,
        par: SynchronousMachinePars | SaturatedSynchronousMachinePars,
        alpha_o: float,
        U_inj: float,
        T_s: float,
        J: float | None = None,
        N_inj: int = 1,
    ) -> None:
        # Configure observer gains for critically damped dynamics
        if J is None:
            self.k_theta = 2 * par.n_p * alpha_o
            k_w = alpha_o**2
            k_tau = 0.0
        else:
            self.k_theta = 3 * par.n_p * alpha_o
            k_w = 3 * alpha_o**2
            k_tau = J * alpha_o**3

        self.speed_observer = SpeedObserver(k_w, k_tau, J)
        self.injection = SquareWaveInjection(par, U_inj, T_s, N_inj)
        self.theta_m: float = 0.0  # State
        self.par = par

    def compute_output(
        self, u_s_ab: complex, i_s_ab: complex, theta_M_meas: float | None = None
    ) -> ObserverOutputs:
        """Compute output."""
        # Unpack and initialize the output signals
        par = self.par
        out = ObserverOutputs()
        out.w_M, out.tau_L = self.speed_observer.compute_output()
        out.theta_m = out.theta_c = self.theta_m
        out.w_m = par.n_p * out.w_M

        # Current and voltage vectors in (estimated) rotor coordinates
        out.i_s = exp(-1j * out.theta_m) * i_s_ab
        out.u_s = exp(-1j * out.theta_m) * u_s_ab

        # Compute the mechanical position error signal
        out.eps = self.injection.compute_error(i_s_ab, out.theta_m)

        # Coordinate system angular frequency
        out.w_c = out.w_m + self.k_theta * out.eps

        # Torque estimate based on the measured current and the flux map
        psi_s = complex(par.psi_s_dq(out.i_s))
        out.tau_M = 1.5 * par.n_p * (out.i_s * psi_s.conjugate()).imag

        return out

    def update(self, T_s: float, out: ObserverOutputs) -> None:
        """Update the states."""
        self.speed_observer.update(T_s, out.eps, out.tau_M)
        self.theta_m = wrap(self.theta_m + T_s * out.w_c)
        self.injection.update(T_s)


# %%
class SignalInjectionController(CurrentVectorController):
    """
    Sensorless controller with signal injection for synchronous machine drives.

    This class implements a square-wave signal injection for low-speed operation
    according to [#Kim2012]_. Cross-saturation errors are compensated for using flux
    maps [#You2018]_. A related adjustable-frequency method is presented in [#Yu2022]_.
    If the inertia of the mechanical system is provided, the speed is estimated using
    the speed observer based on the mechanical model [#Kim2003]_, otherwise the phase-
    locked loop is used.

    Parameters
    ----------
    par : SynchronousMachinePars | SaturatedSynchronousMachinePars
        Machine model parameters.
    cfg : CurrentVectorControllerCfg
        Current-vector control configuration.
    U_inj : float, optional
        Injected voltage amplitude (V), defaults to 250.
    N_inj : int, optional
        Number of sampling periods per injection voltage half-period, defaults to 1. The
        injection frequency is `1 / (2 * N_inj * cfg.T_s)`. Reducing this frequency
        requires reducing `cfg.alpha_o` accordingly.

    References
    ----------
    .. [#Kim2012] Kim, Ha, Sul, "PWM switching frequency signal injection sensorless
       method in IPMSM," IEEE Trans. Ind. Appl., 2012,
       https://doi.org/10.1109/TIA.2012.2210175

    .. [#You2018] Yousefi-Talouki, Pescetto, Pellegrino, Boldea, "Combined active flux
       and high-frequency injection methods for sensorless direct-flux vector control of
       synchronous reluctance machines," IEEE Trans. Power Electron., 2018,
       https://doi.org/10.1109/TPEL.2017.2697209

    .. [#Yu2022] Yu, Wang, "Position sensorless control of IPMSM using adjustable
       frequency setting square-wave voltage injection," IEEE Trans. Power Electron.,
       2022, https://doi.org/10.1109/TPEL.2022.3179611

    .. [#Kim2003] Kim, Harke, Lorenz, "Sensorless control of interior permanent-magnet
       machine drives with zero-phase lag position estimation," IEEE Trans. Ind. Appl.,
       2003, https://doi.org/10.1109/TIA.2003.818966

    """

    def __init__(
        self,
        par: SynchronousMachinePars | SaturatedSynchronousMachinePars,
        cfg: CurrentVectorControllerCfg,
        U_inj: float = 250,
        N_inj: int = 1,
    ) -> None:
        super().__init__(par, cfg)
        self.observer = SignalInjectionObserver(
            par, cast(float, cfg.alpha_o), U_inj, cfg.T_s, cfg.J, N_inj
        )

    def compute_output(self, tau_M_ref: float, fbk: ObserverOutputs) -> References:
        ref = References(T_s=self.T_s, tau_M=tau_M_ref)
        ref.psi_s, ref.tau_M = self.reference_gen.compute_flux_and_torque_refs(
            ref.tau_M, fbk.w_m, fbk.u_dc
        )
        ref.i_s = self.reference_gen.compute_current_ref(ref.tau_M)
        ref.u_s = (
            self.current_ctrl.compute_output(ref.i_s, fbk.i_s)
            + self.observer.injection.u_sd_inj
        )
        return ref

    def post_process(self, ts: TimeSeries) -> None:
        """Post-process controller time series."""
