"""Current-vector control methods for synchronous machine drives."""

from cmath import exp
from dataclasses import dataclass, field
from math import inf, pi
from typing import Callable, Sequence

from motulator.common.control import ComplexPIController
from motulator.common.control._pwm import PWM
from motulator.drive.control._base import Measurements
from motulator.drive.control._sm_observers import (
    ObserverOutputs,
    create_sensored_observer,
    create_sensorless_observer,
)
from motulator.drive.control._sm_reference_gen import ReferenceGenerator
from motulator.drive.utils._parameters import (
    SaturatedSynchronousMachinePars,
    SynchronousMachinePars,
)


# %%
@dataclass
class References:
    """Reference signals."""

    T_s: float = 0.0
    d_abc: Sequence[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
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
        Current-control integral-action bandwidth (rad/s), defaults to `alpha_c`.
    alpha_o : float, optional
        Speed estimation bandwidth (rad/s), defaults to 2*pi*100.
    k_o : Callable[[float], float], optional
        Observer gain as a function of the rotor angular speed.
    k_f : Callable[[float], float], optional
        PM-flux estimation gain as a function of the rotor angular speed.
    psi_s_min : float, optional
        Minimum stator flux (Vs), defaults to `par.psi_f`.
    psi_s_max : float, optional
        Maximum stator flux (Vs), defaults to `inf`.
    k_u : float, optional
        Voltage utilization factor, defaults to 0.9.
    k_mtpv : float, optional
        MTPV margin, defaults to 0.9.

    """

    i_s_max: float
    alpha_c: float = 2 * pi * 200
    alpha_i: float | None = None
    alpha_o: float = 2 * pi * 100
    k_o: Callable[[float], float] | None = None
    k_f: Callable[[float], float] | None = None
    psi_s_min: float | None = None
    psi_s_max: float = inf
    k_u: float = 0.9
    k_mtpv: float = 0.9


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
    sensorless : bool, optional
        If True, sensorless control is used, defaults to True.
    T_s : float, optional
        Sampling period (s), defaults to 125e-6.

    """

    def __init__(
        self,
        par: SynchronousMachinePars | SaturatedSynchronousMachinePars,
        cfg: CurrentVectorControllerCfg,
        sensorless: bool = True,
        T_s: float = 125e-6,
    ) -> None:
        self.pwm = PWM()
        self.reference_gen = ReferenceGenerator(
            par, cfg.i_s_max, cfg.psi_s_min, cfg.psi_s_max, cfg.k_u, cfg.k_mtpv
        )
        self.current_ctrl = CurrentController(par, cfg.alpha_c, cfg.alpha_i)
        if sensorless:
            self.observer = create_sensorless_observer(
                par, cfg.alpha_o, cfg.k_o, cfg.k_f
            )
        else:
            self.observer = create_sensored_observer(
                par, 2 * cfg.alpha_o, cfg.k_o, cfg.k_f
            )
        self.sensorless = sensorless
        self.T_s = T_s

    def get_feedback(self, meas: Measurements) -> ObserverOutputs:
        """Get feedback signals."""
        u_c_ab = self.pwm.get_realized_voltage()
        if self.observer.sensorless:
            fbk = self.observer.compute_output(u_c_ab, meas.i_c_ab)
        else:
            fbk = self.observer.compute_output(
                u_c_ab, meas.i_c_ab, meas.w_M, meas.theta_M
            )
        fbk.u_dc = meas.u_dc
        return fbk

    def compute_output(self, tau_M_ref: float, fbk: ObserverOutputs) -> References:
        """Compute references."""
        ref = References(T_s=self.T_s, tau_M=tau_M_ref)
        ref.psi_s, ref.tau_M = self.reference_gen.compute_flux_and_torque_refs(
            ref.tau_M, fbk.w_m, fbk.u_dc
        )
        ref.i_s = self.reference_gen.compute_current_ref(ref.psi_s, ref.tau_M)
        ref.u_s = self.current_ctrl.compute_output(ref.i_s, fbk.i_s)
        u_s_ref_ab = exp(1j * fbk.theta_m) * ref.u_s
        ref.d_abc = self.pwm(ref.T_s, u_s_ref_ab, fbk.u_dc, fbk.w_s)
        return ref

    def update(self, ref: References, fbk: ObserverOutputs) -> None:
        """Update states."""
        self.observer.update(ref.T_s)
        self.current_ctrl.update(ref.T_s, fbk.u_s, fbk.w_s)
