"""Sensorless control with signal injection for synchronous machine drives."""

from cmath import exp

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

    """

    def __init__(
        self,
        par: SynchronousMachinePars | SaturatedSynchronousMachinePars,
        alpha_o: float,
        U_inj: float,
        T_s: float,
        J: float | None = None,
    ) -> None:
        self._T_s = T_s
        # Configure observer gains for critically damped dynamics
        if J is None:
            self.k_theta = 2 * par.n_p * alpha_o
            k_w = alpha_o**2
            k_tau = 0.0
        else:
            self.k_theta = 3 * par.n_p * alpha_o
            k_w = 3 * alpha_o**2
            k_tau = J * alpha_o**3

        # Create speed observer
        self.speed_observer = SpeedObserver(k_w, k_tau, J)
        # State
        self.theta_m: float = 0.0
        # Constant error gain based on the unsaturated inductances
        L_s = par.incr_ind_mat(0j)
        self.k = 0.5 * L_s[0, 0] / (L_s[1, 1] - L_s[0, 0]) / par.n_p
        self.U_inj = U_inj
        self.u_sd_inj = U_inj
        self._old_psi_sq: float = 0.0
        self._older_psi_sq: float = 0.0
        self.par = par
        self.sensorless = True  # For compatibility reasons

    def _compute_demodulation_error(self, i_s: complex) -> float:
        """Compute mechanical position error signal."""
        # Apply the flux map to compensate the cross saturation
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

    def compute_output(
        self,
        u_s_ab: complex,
        i_s_ab: complex,
        w_M: float | None,
        theta_M_meas: float | None,
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
        out.eps = self._compute_demodulation_error(out.i_s)

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
        self.u_sd_inj = (-1 if self.u_sd_inj > 0 else 1) * self.U_inj
        self._T_s = T_s


# %%
class SignalInjectionController(CurrentVectorController):
    """
    Sensorless controller with signal injection for synchronous machine drives.

    This class implements a square-wave signal injection for low-speed operation
    according to [#Kim2012]_. Cross-saturation errors are compensated for using flux
    maps [#You2018]_. If the inertia of the mechanical system is provided, the speed is
    estimated using the speed observer based on the mechanical model [#Kim2003]_,
    otherwise the phase-locked loop is used.

    Parameters
    ----------
    par : SynchronousMachinePars | SaturatedSynchronousMachinePars
        Machine model parameters.
    cfg : CurrentVectorControllerCfg
        Current-vector control configuration.
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

    .. [#Kim2003] Kim, Harke, Lorenz, "Sensorless control of interior permanent-magnet
       machine drives with zero-phase lag position estimation," IEEE Trans. Ind. Appl.,
       2003, https://doi.org/10.1109/TIA.2003.818966

    """

    def __init__(
        self,
        par: SynchronousMachinePars | SaturatedSynchronousMachinePars,
        cfg: CurrentVectorControllerCfg,
        U_inj: float = 250,
        T_s: float = 125e-6,
    ) -> None:
        super().__init__(par, cfg, True, T_s)
        assert cfg.alpha_o is not None
        self.observer = SignalInjectionObserver(par, cfg.alpha_o, U_inj, T_s, cfg.J)

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
