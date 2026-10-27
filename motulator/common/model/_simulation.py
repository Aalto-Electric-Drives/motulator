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
from motulator.common.model._base import (
    ControlledConverter,
    Converter,
    Model,
    ModelTimeSeries,
)
from motulator.common.model._composite import CompositeModel, CompositeModelTimeSeries
from motulator.common.model._converter import DCBus

# Tolerance (s) for treating the sampling and switching instants of different
# converters as simultaneous, and diode transitions as stuck
_T_TOL = 1e-12
# Maximum number of consecutive diode transitions at the same instant
_MAX_STUCK = 10
# Default maximum step (s) while a converter is blocked or without control systems, of
# the order of a sampling period
_MAX_STEP = 1e-4


# %%
@dataclass
class SolverCfg:
    """
    Solver configuration parameters.

    Parameters
    ----------
    max_step : float, optional
        Maximum step size (s) for the integrator. Defaults to None, i.e., 1e-4 s while
        the gates of a converter are blocked, so that the solver locates the
        transitions of the diodes driven by the AC voltages, and without control
        systems, so that the solver resolves the waveforms. Otherwise, the step size is
        not limited, since the sampling and switching instants limit the integration
        intervals.
    method : str, optional
        Integration method, defaults to "RK45".
    rtol : float, optional
        Relative tolerance, defaults to 1e-3.
    atol : float, optional
        Absolute tolerance, defaults to 1e-6.

    """

    max_step: float | None = None
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
        Results from the digital control system, None for a model without a control
        system. For a composite model, the results of each control system are an
        attribute by the model name (None for a model without a control system).
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
    mdl : Model
        Continuous-time system model.
    ctrl : ControlSystem | Mapping[str, ControlSystem], optional
        Discrete-time control system of a model with a controlled converter. For a
        composite model, the control systems of the models with controlled converters
        by the model names. Defaults to None, i.e., no control systems.
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
        mdl: Model,
        ctrl: ControlSystem | Mapping[str, ControlSystem] | None = None,
        show_progress: bool = True,
        cfg: SolverCfg | None = None,
        t_start: float | Mapping[str, float] | None = None,
    ) -> None:
        if os.environ.get("BUILDING_DOCS") == "1":
            show_progress = False
        self.show_progress = show_progress
        self.cfg = cfg if cfg is not None else SolverCfg()
        self.mdl = mdl
        # Models and control systems by name, a single model by the name ""
        if isinstance(mdl, CompositeModel):
            if not isinstance(ctrl, Mapping | None):
                raise ValueError("A composite model takes the control systems by name")
            if not isinstance(t_start, Mapping | None):
                raise ValueError("t_start must be a mapping by the model names")
            models: Mapping[str, Model] = mdl.models
            ctrls = dict(ctrl or {})
            starts = dict(t_start or {})
        else:
            if isinstance(ctrl, Mapping) or isinstance(t_start, Mapping):
                raise ValueError("A single model takes a single control system")
            models = {"": mdl}
            ctrls = {} if ctrl is None else {"": ctrl}
            starts = {} if t_start is None else {"": t_start}
        controlled = [
            name
            for name, model in models.items()
            if isinstance(model.converter, ControlledConverter)
        ]
        if ctrls.keys() != set(controlled):
            raise ValueError(
                "A control system must be given for each model with a controlled "
                "converter, and only for these models"
            )
        if starts.keys() - ctrls.keys():
            raise ValueError("t_start must be given for the control systems")
        # A DC bus computes the currents of all its converters, so their models must be
        # simulated too
        for dc_bus in mdl.subsystems:
            if isinstance(dc_bus, DCBus) and not all(
                c in mdl.converters for c in dc_bus.converters
            ):
                raise ValueError("A converter fed from the DC bus is not in the model")
        # Pairs of the models and their control systems, and their sampling instants
        self._models = models
        self._ctrls = ctrls
        self._pairs = [(models[name], ctrls[name]) for name in controlled]
        self._t_sample = [starts.get(name, mdl.t0) for name in controlled]
        if not all(mdl.t0 <= t < np.inf for t in self._t_sample):
            raise ValueError("t_start must not be earlier than the model time")
        for name, t in starts.items():
            ctrls[name].t = t
        # Pending switching events (instant, gate signals, blanking signals) of each
        # converter
        self._events: list[list[tuple[float, np.ndarray, np.ndarray]]] = [
            [] for _ in self._pairs
        ]

    def simulate(self, t_stop: float = 1.0, N_eval: int = 0) -> SimulationResults:
        """
        Solve continuous-time system model and call control system.

        The simulation runs until the first sampling instant after `t_stop`, or
        until `t_stop` without control systems. A subsequent call continues the
        simulation.

        Parameters
        ----------
        t_stop : float, optional
            Simulation stop time, defaults to 1.
        N_eval : int, optional
            Number of evenly spaced data points to be returned by the solver for each
            integration interval, i.e., for each interval between the sampling and
            switching instants, or for the whole simulation without control systems.
            Defaults to 0, in which case the number and spacing of points is selected
            by the solver.

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
        ctrl_ts = {name: ctrl.post_process() for name, ctrl in self._ctrls.items()}
        if isinstance(self.mdl, CompositeModel):
            mdl_ts = CompositeModelTimeSeries(self.mdl)
            names = self.mdl.models
            ctrl_ts = SimpleNamespace(**{name: ctrl_ts.get(name) for name in names})
            return SimulationResults(mdl_ts, ctrl_ts, success=error is None)
        mdl_ts = ModelTimeSeries(self.mdl)
        return SimulationResults(mdl_ts, ctrl_ts.get(""), success=error is None)

    @np.errstate(invalid="raise")
    def _run_simulation_loop(
        self, t_stop: float, update_progress: Callable[[], None], N_eval: int
    ) -> None:
        """Run the main simulation loop."""
        t_sample, events = self._t_sample, self._events
        while True:
            t0 = self.mdl.t0
            due = [k for k, t in enumerate(t_sample) if t <= t0 + _T_TOL]
            if (due and t0 > t_stop) or (not self._pairs and t0 >= t_stop):
                break
            for k, (mdl, ctrl) in enumerate(self._pairs):
                # Control, computational delay, and carrier comparison
                if k in due:
                    # The gate enable takes effect at the sampling instant, so that
                    # the control system measures the converter with blocked gates
                    enabled = ctrl.get_enable()
                    if enabled != mdl.converter.inp.enabled:
                        mdl.converter.set_enable(enabled)
                        self.mdl.set_outputs(t0)
                    T_s, ref_duty_ratio = ctrl(mdl)
                    if T_s <= _T_TOL:
                        raise ValueError("The sampling period must be positive")
                    t_steps, q_abc, b_abc = mdl.modulate(T_s, ref_duty_ratio)
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
            t1 = min(t_sample + [ev[0][0] for ev in events if ev], default=t_stop)
            self._integrate((t0, t1), N_eval)

    def _integrate(self, t_span: tuple[float, float], N_eval: int) -> None:
        """Integrate over the time span and save the solution."""
        t0, t1 = t_span
        if t1 <= t0:
            return

        # Create array of evaluation times if N_eval is given, including the end point
        # for the final state
        t_eval = np.linspace(t0, t1, N_eval + 1) if N_eval != 0 else None

        # The transitions of the diode states of the blocked converters are located as
        # events, at which the integration is restarted
        blocked = [c for c in self.mdl.converters if c.blocked]
        events = [_DiodeEvent(self.mdl, c) for c in blocked]
        solver = self.cfg.solver
        if solver["max_step"] is None:
            solver["max_step"] = _MAX_STEP if blocked or not self._pairs else np.inf

        n_stuck = 0  # Consecutive transitions at the same instant
        while True:
            state0 = self.mdl.get_initial_values()

            # Explicit Runge-Kutta methods try the whole interval as the first step,
            # which skips the estimation of the initial step size
            explicit = self.cfg.method in ("RK23", "RK45", "DOP853")
            first_step = min(t1 - t0, solver["max_step"]) if explicit else None

            sol = solve_ivp(
                self.mdl.rhs,
                (t0, t1),
                state0,
                t_eval=None if t_eval is None else t_eval[t_eval >= t0],
                events=events if events else None,
                first_step=first_step,
                **solver,
            )
            if not sol.success:
                raise FloatingPointError(sol.message)

            # Set the final state, since the last call to rhs may be at an earlier
            # instant (e.g., DOP853 with t_eval), or the state at the first event
            if sol.status == 1:
                fired = [k for k, t_e in enumerate(sol.t_events) if t_e.size > 0]
                k = min(fired, key=lambda k: sol.t_events[k][0])
                t_e, state = sol.t_events[k][0], sol.y_events[k][0]
                n_stuck = n_stuck + 1 if t_e <= t0 + _T_TOL else 0
                t0 = t_e
            else:
                t0, state = t1, sol.y[:, -1]
            # The models of a composite model have their own clocks, which advance
            # with the integration, regardless of the saved points
            self.mdl.t0 = t0
            for mdl in self._models.values():
                mdl.t0 = t0
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
            if n_stuck > _MAX_STUCK:
                raise FloatingPointError(f"Diode transitions stuck at {t0} s")
            for converter in blocked:
                converter.update_diodes()


class _DiodeEvent:
    """Event function for the transitions of the diode states of a converter."""

    terminal = True
    direction = -1  # The margin decreases to zero at a transition

    def __init__(self, mdl: Model, converter: Converter) -> None:
        self.mdl = mdl
        self.converter = converter

    def __call__(self, t: float, state_list: list[complex]) -> float:
        self.mdl.set_states(state_list)
        self.mdl.set_outputs(t)
        return self.converter.margin
