"""Current-vector control methods for synchronous machine drives."""

from collections.abc import Callable
from dataclasses import dataclass
from math import inf, pi

from motulator.common.control import ComplexPIController, DiscreteComplexController
from motulator.common.control._base import TimeSeries
from motulator.drive.control._common import References
from motulator.drive.control._sm_observers import (
    ObserverOutputs,
    compute_feedback,
    create_speed_flux_observer,
)
from motulator.drive.control._sm_reference_gen import (
    ReferenceGenerator,
    ReferenceGeneratorOnline,
)
from motulator.drive.utils._parameters import BaseSynchronousMachineControlPars


# %%
@dataclass(kw_only=True)
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
    alpha_o : float | None, optional
        Speed estimation poles (rad/s). Defaults to 2*pi*50 if `sensorless` is True,
        otherwise 2*pi*400. If `J` is given, the default is divided by sqrt(3), which
        keeps the speed estimation gain approximately the same.
    alpha_ref : float, optional
        Reference generation bandwidth (rad/s), defaults to 2*pi*100.
    k_o : Callable[[float], float] | None, optional
        Observer gain as a function of the electrical rotor angular speed. If None,
        defaults to ``lambda w_m: 0.25*R_s*(1/L_d + 1/L_q) + 0.2*abs(w_m)`` if
        `sensorless` is True (incremental inductances at zero current), otherwise
        ``lambda w_m: 2*pi*15``.
    k_f : Callable[[float], float] | None, optional
        PM-flux estimation gain as a function of the electrical rotor angular speed. If
        None, the gain is zero, i.e., the PM-flux linkage is not estimated.
    psi_s_min : float | None, optional
        Minimum stator flux (Vs). If None, `par.psi_f` is used. It must be positive for
        machines without PMs.
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
        continuous-time design, defaults to False.

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


# %%
class CurrentVectorController:
    """
    Current vector controller for synchronous machine drives.

    The currents are mapped to the flux linkages by the flux linkage map, and the flux
    linkages are controlled by the 2DOF PI controller or, if `discrete` is True, by the
    direct discrete-time controller. The saliency and the magnetic saturation are thus
    taken into account [#Awa2019a]_.

    Parameters
    ----------
    par : BaseSynchronousMachineControlPars
        Machine model parameters.
    cfg : CurrentVectorControllerCfg
        Current-vector control configuration.

    References
    ----------
    .. [#Awa2019a] Awan, Saarakkala, Hinkkanen, "Flux-linkage-based current control of
       saturated synchronous motors," IEEE Trans. Ind. Appl., 2019,
       https://doi.org/10.1109/TIA.2019.2919258

    """

    def __init__(
        self, par: BaseSynchronousMachineControlPars, cfg: CurrentVectorControllerCfg
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
        # Controllers of the flux linkages
        self.current_ctrl: ComplexPIController | DiscreteComplexController
        if cfg.discrete:
            self.current_ctrl = DiscreteComplexController(cfg.alpha_c, cfg.T_s)
        else:
            alpha_i = cfg.alpha_c if cfg.alpha_i is None else cfg.alpha_i
            self.current_ctrl = ComplexPIController(
                k_p=cfg.alpha_c + alpha_i, k_i=cfg.alpha_c * alpha_i, k_t=cfg.alpha_c
            )
        self.observer = create_speed_flux_observer(
            par, cfg.alpha_o, cfg.T_s, cfg.k_o, cfg.k_f, cfg.sensorless, cfg.J
        )
        self.par = par
        self.cfg = cfg
        self.sensorless = cfg.sensorless
        # The discrete-time current controller compensates for the delays itself
        self.k_comp = 0.0 if cfg.discrete else 1.5
        self.T_s = cfg.T_s

    def get_feedback(
        self,
        u_s_ab: complex | None,
        i_s_ab: complex,
        w_M_meas: float | None,
        theta_M_meas: float | None,
    ) -> ObserverOutputs:
        """Get the feedback signals."""
        fbk = compute_feedback(
            self.observer, self.sensorless, u_s_ab, i_s_ab, theta_M_meas
        )
        if fbk.caught:
            # In standby, the states tracked the speed estimate held by the observer,
            # so they restart from the zero-torque operating point of the caught speed
            # (zero current, since the catch requires magnets)
            self.reference_gen.i_s_ref = 0j
            self.current_ctrl.preset(1j * fbk.w_m * fbk.psi_f, fbk.w_c)
        return fbk

    def compute_output(self, tau_M_ref: float, fbk: ObserverOutputs) -> References:
        """Compute references."""
        ref = References(T_s=self.T_s, tau_M=tau_M_ref)
        ref.psi_s, ref.tau_M = self.reference_gen.compute_flux_and_torque_refs(
            ref.tau_M, fbk.w_m, fbk.u_dc
        )
        ref.i_s = self.reference_gen.compute_current_ref(ref.tau_M)
        # Map the currents to the flux linkages
        psi_s_ref = complex(self.par.psi_s_dq(ref.i_s)) - self.par.psi_f
        psi_s = complex(self.par.psi_s_dq(fbk.i_s)) - self.par.psi_f
        ref.u_s = self.current_ctrl.compute_output(psi_s_ref, psi_s)
        return ref

    def update(self, ref: References, fbk: ObserverOutputs) -> None:
        """Update states."""
        self.observer.update(ref.T_s, fbk)
        # The discrete-time controller takes its limited output, except with blocked
        # gates, when the open-circuit voltage of the observer is the realized one
        u_s = ref.u_s_lim if self.cfg.discrete and ref.enabled else fbk.u_s
        self.current_ctrl.update(ref.T_s, u_s, fbk.w_c)
        self.reference_gen.update(ref.T_s)

    def post_process(self, ts: TimeSeries) -> None:
        """Post-process controller time series."""
