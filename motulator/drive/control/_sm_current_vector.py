"""Current-vector control methods for synchronous machine drives."""

from cmath import exp
from dataclasses import dataclass
from math import inf, pi
from typing import Callable, cast

from motulator.common.control import ComplexPIController
from motulator.common.control._base import TimeSeries
from motulator.drive.control._sm_observers import (
    ObserverOutputs,
    create_speed_flux_observer,
    position_error,
)
from motulator.drive.control._sm_reference_gen import (
    ReferenceGenerator,
    ReferenceGeneratorOnline,
)
from motulator.drive.utils._parameters import (
    SaturatedSynchronousMachinePars,
    SynchronousMachinePars,
)


# %%
@dataclass
class References:
    """Reference signals."""

    T_s: float = 0.0
    tau_M: float = 0.0
    psi_s: float = 0.0
    i_s: complex = 0j
    u_s: complex = 0j


# %%
class CurrentController(ComplexPIController):
    """
    Current controller for synchronous machines.

    This provides an interface of a current controller for synchronous machines
    [#Awa2019a]_. The gains are initialized based on the desired closed-loop bandwidth
    and the inductances (or nonlinear flux linkage maps).

    Parameters
    ----------
    par : SynchronousMachinePars | SaturatedSynchronousMachinePars
        Machine model parameters.
    alpha_c : float
        Reference-tracking bandwidth (rad/s).
    alpha_i : float, optional
        Integral-action bandwidth (rad/s), defaults to `alpha_c`.

    References
    ----------
    .. [#Awa2019a] Awan, Saarakkala, Hinkkanen, "Flux-linkage-based current control of
       saturated synchronous motors," IEEE Trans. Ind. Appl. 2019,
       https://doi.org/10.1109/TIA.2019.2919258

    """

    def __init__(
        self,
        par: SynchronousMachinePars | SaturatedSynchronousMachinePars,
        alpha_c: float,
        alpha_i: float | None = None,
    ) -> None:
        self.par = par
        alpha_i = alpha_c if alpha_i is None else alpha_i
        k_t = alpha_c
        k_i = alpha_c * alpha_i
        k_p = alpha_c + alpha_i
        super().__init__(k_p, k_i, k_t)

    def compute_output(self, i_ref: complex, i: complex, u_ff: complex = 0j) -> complex:
        # Extends the base class method by mapping the currents to the flux linkages,
        # which is a simple way to take saliency and magnetic saturation into account.
        psi_ref = complex(self.par.psi_s_dq(i_ref)) - self.par.psi_f
        psi = complex(self.par.psi_s_dq(i)) - self.par.psi_f
        return super().compute_output(psi_ref, psi, u_ff)


# %%
class DiscreteCurrentController:
    """
    Direct discrete-time current controller for synchronous machines.

    This implements the flux-linkage-based current controller designed directly in
    discrete time [#Awa2019a]_. As in `CurrentController`, the currents are mapped to
    the flux linkages, which takes saliency and magnetic saturation into account. The
    design is based on the hold-equivalent machine model in the controller coordinates,
    i.e., the stator voltage is constant in stationary coordinates over the sampling
    period, and the computational delay of one sampling period is included. The stator
    resistance is omitted from the model, and the integral action compensates for the
    resistive voltage drop. The complex-vector design gives the reference-tracking
    dynamics ``psi(k) = (1 - beta)/(z*(z - beta))*psi_ref(k)``, where ``beta =
    exp(-alpha_c*T_s)``.

    The voltage reference is applied as such, so the PWM should be configured with
    `k_comp=0` and `average=False`, the realized voltage then being that of the ongoing
    sampling period. The anti-windup uses this voltage, which becomes available one
    sampling period after the voltage reference is computed.

    Parameters
    ----------
    par : SynchronousMachinePars | SaturatedSynchronousMachinePars
        Machine model parameters.
    alpha_c : float
        Reference-tracking bandwidth (rad/s).
    T_s : float
        Sampling period (s).

    """

    def __init__(
        self,
        par: SynchronousMachinePars | SaturatedSynchronousMachinePars,
        alpha_c: float,
        T_s: float,
    ) -> None:
        self.par = par
        self.T_s = T_s
        self.beta = exp(-alpha_c * T_s).real
        # States
        self.u_i: complex = 0j  # Integral state
        self.u_ref_old: complex = 0j  # Voltage reference of the previous period
        self.w_c: float = 0.0  # Angular speed of the coordinates
        # Workspace variables
        self._e: complex = 0j
        self._u_ref: complex = 0j

    def _gains(self, w_c: float) -> tuple[complex, complex, complex, complex]:
        """Gains of the complex-vector design, see (20) and (22) in [#Awa2019a]_."""
        T_s, beta = self.T_s, self.beta
        Phi = exp(-1j * w_c * T_s)  # Rotation of the coordinates over T_s
        k_t = (1 - beta) / (Phi**2 * T_s)
        k_i = (1 - beta) * (1 - beta * Phi) / (Phi**2 * T_s**2)
        k_1 = (1 - beta) * (1 + (1 - beta) / Phi + 1 / Phi**2) / T_s
        k_2 = (1 - beta) * (1 + Phi)
        return k_t, k_i, k_1, k_2

    def compute_output(self, i_ref: complex, i: complex) -> complex:
        """
        Compute the controller output.

        Parameters
        ----------
        i_ref : complex
            Current reference (A).
        i : complex
            Current feedback (A).

        Returns
        -------
        complex
            Voltage reference (V).

        """
        k_t, _, k_1, k_2 = self._gains(self.w_c)
        psi_ref = complex(self.par.psi_s_dq(i_ref)) - self.par.psi_f
        psi = complex(self.par.psi_s_dq(i)) - self.par.psi_f
        self._e = psi_ref - psi
        self._u_ref = k_t * psi_ref - k_1 * psi - k_2 * self.u_ref_old + self.u_i
        return self._u_ref

    def update(self, T_s: float, u: complex, w_c: float) -> None:
        """
        Update the states.

        Parameters
        ----------
        T_s : float
            Sampling period (s), which must equal the design value.
        u : complex
            Realized voltage (V) of the ongoing sampling period, i.e., the limited
            voltage reference of the previous sampling period rotated to the present
            coordinates.
        w_c : float
            Angular speed of the coordinates (rad/s).

        """
        k_t, k_i, _, _ = self._gains(self.w_c)
        # Anti-windup for the previous sampling period, whose realized voltage is known
        u_lim_old = exp(1j * self.w_c * T_s) * u
        self.u_i -= T_s * k_i / k_t * (self.u_ref_old - u_lim_old)
        # Integral action and the states for the next sampling period
        self.u_i += T_s * k_i * self._e
        self.u_ref_old = self._u_ref
        self.w_c = w_c


# %%
@dataclass
class CurrentVectorControllerCfg:
    """
    Current-vector controller configuration.

    Parameters
    ----------
    i_s_max : float
        Maximum stator current (A).
    alpha_c : float, optional
        Current-control bandwidth (rad/s), defaults to 2*pi*200.
    alpha_i : float, optional
        Current-control integral-action bandwidth (rad/s), defaults to `alpha_c`. Not
        used if `discrete` is True.
    alpha_o : float, optional
        Speed estimation poles (rad/s). Defaults to 2*pi*50 if `J` is None, otherwise
        2*pi*50/3, keeping the default speed observer gain the same.
    alpha_ref : float, optional
        Reference generation bandwidth (rad/s), defaults to 2*pi*100.
    k_o : Callable[[float], float], optional
        Observer gain as a function of the rotor angular speed.
    k_f : Callable[[float], float], optional
        PM-flux estimation gain as a function of the rotor angular speed.
    psi_s_min : float, optional
        Minimum stator flux (Vs), defaults to `par.psi_f` elsewhere.
    psi_s_max : float, optional
        Maximum stator flux (Vs), defaults to `inf`.
    k_u : float, optional
        Voltage utilization factor, defaults to 0.9.
    k_mtpv : float, optional
        MTPV margin, defaults to 0.9.
    J : float | None, optional
        Inertia (kgm²). Defaults to None, meaning the mechanical system model is not
        used in speed estimation.
    sensorless : bool, optional
        If True, sensorless control is used, defaults to True.
    online_ref : bool, optional
        If True, the online reference generation is used, defaults to False.
    T_s : float, optional
        Sampling period (s), defaults to 125e-6.
    discrete : bool, optional
        If True, the direct discrete-time current controller is used instead of the
        continuous-time design, defaults to False. The PWM should then be configured
        with `k_comp=0` and `average=False`.

    """

    i_s_max: float
    alpha_c: float = 2 * pi * 200
    alpha_i: float | None = None
    alpha_o: float | None = None
    alpha_ref: float = 2 * pi * 100
    k_o: Callable[[float], float] | None = None
    k_f: Callable[[float], float] | None = None
    psi_s_min: float | None = None
    psi_s_max: float = inf
    k_u: float = 0.9
    k_mtpv: float = 0.9
    J: float | None = None
    sensorless: bool = True
    online_ref: bool = False
    T_s: float = 125e-6
    discrete: bool = False

    def __post_init__(self) -> None:
        """Set alpha_o default based on J value."""
        if self.alpha_o is None:
            # To keep the speed observer gain k_w the same
            alpha = 2 * pi * 50
            self.alpha_o = alpha if self.J is None else alpha / 3.0


# %%
class CurrentVectorController:
    """
    Current vector controller for synchronous machine drives.

    Parameters
    ----------
    par : SynchronousMachinePars | SaturatedSynchronousMachinePars
        Machine model parameters.
    cfg : CurrentVectorControllerCfg
        Current-vector control configuration.

    """

    def __init__(
        self,
        par: SynchronousMachinePars | SaturatedSynchronousMachinePars,
        cfg: CurrentVectorControllerCfg,
    ) -> None:
        reference_generator = (
            ReferenceGeneratorOnline if cfg.online_ref else ReferenceGenerator
        )
        self.reference_gen = reference_generator(
            par,
            cfg.i_s_max,
            cfg.psi_s_min,
            cfg.psi_s_max,
            cfg.k_u,
            cfg.k_mtpv,
            cfg.alpha_ref,
        )
        self.current_ctrl: CurrentController | DiscreteCurrentController
        if cfg.discrete:
            self.current_ctrl = DiscreteCurrentController(par, cfg.alpha_c, cfg.T_s)
        else:
            self.current_ctrl = CurrentController(par, cfg.alpha_c, cfg.alpha_i)
        self.observer = create_speed_flux_observer(
            par, cast(float, cfg.alpha_o), cfg.k_o, cfg.k_f, cfg.sensorless, cfg.J
        )
        self.par = par
        self.cfg = cfg
        self.sensorless = cfg.sensorless
        self.T_s = cfg.T_s

    def get_feedback(
        self,
        u_s_ab: complex,
        i_s_ab: complex,
        w_M_meas: float | None,
        theta_M_meas: float | None,
    ) -> ObserverOutputs:
        """Get the feedback signals."""
        if self.sensorless:
            return self.observer.compute_output(u_s_ab, i_s_ab)
        if theta_M_meas is None:
            raise ValueError("Rotor angle must be provided in sensored mode")
        eps = position_error(self.par.n_p, theta_M_meas, self.observer.theta_m)
        return self.observer.compute_output(u_s_ab, i_s_ab, eps, 1.0)

    def compute_output(self, tau_M_ref: float, fbk: ObserverOutputs) -> References:
        """Compute references."""
        ref = References(T_s=self.T_s, tau_M=tau_M_ref)
        ref.psi_s, ref.tau_M = self.reference_gen.compute_flux_and_torque_refs(
            ref.tau_M, fbk.w_m, fbk.u_dc
        )
        ref.i_s = self.reference_gen.compute_current_ref(ref.tau_M)
        ref.u_s = self.current_ctrl.compute_output(ref.i_s, fbk.i_s)
        return ref

    def update(self, ref: References, fbk: ObserverOutputs) -> None:
        """Update states."""
        self.observer.update(ref.T_s, fbk)
        self.current_ctrl.update(ref.T_s, fbk.u_s, fbk.w_c)
        self.reference_gen.update(ref.T_s)

    def post_process(self, ts: TimeSeries) -> None:
        """Post-process controller time series."""
