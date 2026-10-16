"""Simulation environment."""

import os
import warnings
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from types import SimpleNamespace
from typing import Any

import numpy as np
from scipy.integrate import solve_ivp
from tqdm import tqdm

from motulator.common.control._base import ControlSystem
from motulator.common.model._base import Converter, Model, ModelTimeSeries
from motulator.common.model._composite import CompositeModel, CompositeModelTimeSeries

# Tolerance (s) for treating the sampling and switching instants of different
# converters as simultaneous
_T_TOL = 1e-12
# Maximum number of diode transitions in an integration interval
_MAX_TRANSITIONS = 100


# %%
@dataclass
class SolverCfg:
    """
    Solver configuration parameters.

    Parameters
    ----------
    max_step : float, optional
        Maximum step size for the integrator, defaults to `inf`.
    method : str, optional
        Integration method, defaults to "RK45".
    rtol : float, optional
        Relative tolerance, defaults to 1e-3.
    atol : float, optional
        Absolute tolerance, defaults to 1e-6.

    """

    max_step: float = np.inf
    method: str = "RK45"
    rtol: float = 1e-3
    atol: float = 1e-6

    @property
    def solver(self) -> dict[str, Any]:
        """Return the solver configuration."""
        return {
            "max_step": self.max_step,
            "method": self.method,
            "rtol": self.rtol,
            "atol": self.atol,
        }


# %%
@dataclass
class SimulationResults:
    """
    Container for simulation results.

    Attributes
    ----------
    mdl : ModelTimeSeries | CompositeModelTimeSeries
        Results from the continuous-time model. For a composite model, the time series
        of each model is an attribute by the model name.
    ctrl : Any
        Results from the digital control system. For a composite model, the results of
        each control system is an attribute by the model name.
    success : bool
        False if the simulation stopped before the stop time due to a numerical
        failure, in which case the results cover the time until the failure.

    """

    mdl: ModelTimeSeries | CompositeModelTimeSeries
    ctrl: Any
    success: bool = True


class Simulation:
    """
    Simulation environment.

    Each control system is called at its own sampling instants, which gives the
    sampling period of its next control cycle. The control systems of a composite
    model can have different sampling periods, and their sampling instants need not
    coincide. The switching states of the converters are held constant between the
    sampling and switching instants of all the converters.

    Parameters
    ----------
    mdl : Model | CompositeModel
        Continuous-time system model.
    ctrl : ControlSystem | Mapping[str, ControlSystem]
        Discrete-time control system. For a composite model, the control systems of
        the models by the model names.
    show_progress : bool, optional
        Show progress during simulation, defaults to True.
    cfg : SolverCfg, optional
        Solver configuration parameters.
    t_start : float | Mapping[str, float], optional
        First sampling instant (s) of the control system or, for a composite model,
        of the control systems by the model names. The time `ctrl.t` of the control
        system is set to this value. Defaults to the initial time of the model, as do
        the control systems not included in the mapping, in which case `ctrl.t` is
        not changed.

    """

    def __init__(
        self,
        mdl: Model | CompositeModel,
        ctrl: ControlSystem | Mapping[str, ControlSystem],
        show_progress: bool = True,
        cfg: SolverCfg | None = None,
        t_start: float | Mapping[str, float] | None = None,
    ) -> None:
        if os.environ.get("BUILDING_DOCS") == "1":
            show_progress = False
        self.show_progress = show_progress
        self.cfg = cfg if cfg is not None else SolverCfg()
        self.mdl = mdl
        self.ctrl = ctrl
        # Pairs of the models and their control systems, and the first sampling
        # instants given for the control systems
        if isinstance(mdl, CompositeModel):
            if not isinstance(ctrl, Mapping) or ctrl.keys() != mdl.models.keys():
                raise ValueError("A control system must be given for each model")
            if t_start is not None and (
                not isinstance(t_start, Mapping) or t_start.keys() - ctrl.keys()
            ):
                raise ValueError("t_start must be a mapping by the model names")
            self._pairs = [(mdl.models[name], ctrl[name]) for name in mdl.models]
            t_start = {} if t_start is None else t_start
            self._t_sample = [t_start.get(name, mdl.t0) for name in mdl.models]
            given = [(ctrl[name], t) for name, t in t_start.items()]
        else:
            if isinstance(ctrl, Mapping) or isinstance(t_start, Mapping):
                raise ValueError("A single model takes a single control system")
            self._pairs = [(mdl, ctrl)]
            self._t_sample = [mdl.t0 if t_start is None else t_start]
            given = [] if t_start is None else [(ctrl, t_start)]
        if not all(mdl.t0 <= t < np.inf for t in self._t_sample):
            raise ValueError("t_start must not be earlier than the model time")
        for ctrl_k, t in given:
            ctrl_k.t = t
        # Pending switching events (instant, gate signals, blanking signals) of each
        # converter
        self._events: list[list[tuple[float, np.ndarray, np.ndarray]]] = [
            [] for _ in self._pairs
        ]

    def simulate(self, t_stop: float = 1.0, N_eval: int = 0) -> SimulationResults:
        """
        Solve continuous-time system model and call control system.

        The simulation runs until the first sampling instant after `t_stop`. A
        subsequent call continues the simulation.

        Parameters
        ----------
        t_stop : float, optional
            Simulation stop time, defaults to 1.
        N_eval : int, optional
            Number of evenly spaced data points to be returned by the solver for each
            integration interval, i.e., for each interval between the sampling and
            switching instants. Defaults to 0, in which case the number and spacing of
            points is selected by the solver.

        """
        progress_bar = None
        if self.show_progress:
            progress_bar = tqdm(
                total=t_stop,
                desc="Simulation",
                unit="s",
                bar_format="{l_bar}{bar}| {n:.2f}/{total:.2f} {unit}",
            )

        def update_progress() -> None:
            if progress_bar is not None:
                progress_bar.n = min(self.mdl.t0, t_stop)
                progress_bar.refresh()

        error = None
        try:
            # Initialize outputs based on the current states
            self.mdl.set_outputs(self.mdl.t0)

            # Main simulation loop
            self._run_simulation_loop(t_stop, update_progress, N_eval)
        except FloatingPointError as err:
            error = err
        finally:
            update_progress()
            if progress_bar is not None:
                progress_bar.close()
        if error is not None:
            msg = f"Simulation stopped at {self.mdl.t0:.2f} s: {error}"
            warnings.warn(msg, RuntimeWarning, stacklevel=2)

        # Post-process the solution data
        if isinstance(self.mdl, CompositeModel) and isinstance(self.ctrl, Mapping):
            mdl_ts = CompositeModelTimeSeries(self.mdl)
            ctrl_ts = SimpleNamespace(
                **{name: ctrl.post_process() for name, ctrl in self.ctrl.items()}
            )
        else:
            mdl_ts = ModelTimeSeries(self._pairs[0][0])
            ctrl_ts = self._pairs[0][1].post_process()
        return SimulationResults(mdl_ts, ctrl_ts, success=error is None)

    @np.errstate(invalid="raise")
    def _run_simulation_loop(
        self, t_stop: float, update_progress: Callable[[], None], N_eval: int
    ) -> None:
        """Run the main simulation loop."""
        t_sample, events = self._t_sample, self._events
        while True:
            t0 = self.mdl.t0
            due = [k for k, t in enumerate(t_sample) if t <= t0 + _T_TOL]
            if due and t0 > t_stop:
                break
            for k, (mdl, ctrl) in enumerate(self._pairs):
                # Control, computational delay, and carrier comparison
                if k in due:
                    T_s, ref_duty_ratio = ctrl(mdl)
                    if T_s <= _T_TOL:
                        raise ValueError("The sampling period must be positive")
                    t_steps, q_abc, b_abc = mdl.modulate(
                        T_s, ref_duty_ratio, ctrl.enabled
                    )
                    # Switching instants, accumulated from the interval durations
                    t_sw = [t0]
                    for t_step in t_steps[:-1]:
                        t_sw.append(t_sw[-1] + t_step)
                    events[k] = list(zip(t_sw, q_abc, b_abc, strict=True))
                    # The next sampling instant is based on the current instant, so a
                    # sampling instant merged with an instant of another converter
                    # within the tolerance shifts the subsequent instants accordingly
                    t_sample[k] = t_sw[-1] + t_steps[-1]
                    update_progress()
                # Set the switching state due at this instant
                while events[k] and events[k][0][0] <= t0 + _T_TOL:
                    _, q_abc, b_abc = events[k].pop(0)
                    mdl.converter.set_gate_signals(q_abc, b_abc)
            # Integrate until the next sampling or switching instant
            t1 = min(t_sample + [ev[0][0] for ev in events if ev])
            self._integrate((t0, t1), N_eval)

    def _integrate(self, t_span: tuple[float, float], N_eval: int) -> None:
        """Integrate over the time span and save the solution."""
        t0, t1 = t_span
        if t1 <= t0:
            return

        # Create array of evaluation times if N_eval is given, including the end point
        # for the final state
        t_eval = np.linspace(t0, t1, N_eval + 1) if N_eval != 0 else None

        # The transitions of the diode states of the converters with blocked gates
        # are located as events, at which the integration is restarted
        blocked = [c for c in self.mdl.converters if not c.inp.enabled]
        events = [_DiodeEvent(self.mdl, c) for c in blocked]

        for _ in range(_MAX_TRANSITIONS):
            state0 = self.mdl.get_initial_values()

            # Explicit Runge-Kutta methods try the whole interval as the first step,
            # which skips the estimation of the initial step size
            first_step = None
            if self.cfg.method in ("RK23", "RK45", "DOP853"):
                first_step = min(t1 - t0, self.cfg.max_step)

            sol = solve_ivp(
                self.mdl.rhs,
                (t0, t1),
                state0,
                t_eval=None if t_eval is None else t_eval[t_eval >= t0],
                events=events if events else None,
                first_step=first_step,
                **self.cfg.solver,
            )
            if not sol.success:
                raise FloatingPointError(sol.message)

            # Set the final state, since the last call to rhs may be at an earlier
            # instant (e.g., DOP853 with t_eval), or the state at the first event
            if sol.status == 1:
                fired = [k for k, t_e in enumerate(sol.t_events) if t_e.size > 0]
                k = min(fired, key=lambda k: sol.t_events[k][0])
                t0, state = sol.t_events[k][0], sol.y_events[k][0]
            else:
                t0, state = t1, sol.y[:, -1]
            self.mdl.t0 = t0
            self.mdl.set_states(state)
            self.mdl.set_outputs(t0)

            # Save the solution (excluding the end point if N_eval is given). With
            # t_eval, the solution may contain no points before the event.
            if sol.status != 1 and N_eval != 0:
                sol.t, sol.y = sol.t[:-1], sol.y[:, :-1]
            if len(sol.t) > 0:
                self.mdl.save(sol)
            if sol.status != 1:
                return
            for converter in blocked:
                converter.update_diodes()
        raise FloatingPointError("Too many diode transitions in an interval")


class _DiodeEvent:
    """Event function for the transitions of the diode states of a converter."""

    terminal = True
    direction = -1  # The margin decreases to zero at a transition

    def __init__(self, mdl: Model | CompositeModel, converter: Converter) -> None:
        self.mdl = mdl
        self.converter = converter

    def __call__(self, t: float, state_list: list[complex]) -> float:
        self.mdl.set_states(state_list)
        self.mdl.set_outputs(t)
        return self.converter.margin
