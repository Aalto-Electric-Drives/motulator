"""Simulation environment."""

import os
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import numpy as np
from scipy.integrate import solve_ivp
from tqdm import tqdm

from motulator.common.control._base import ControlSystem
from motulator.common.model._base import Model, ModelTimeSeries


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
    mdl : ModelTimeSeries
        Results from the continuous-time model.
    ctrl : Any
        Results from the digital control system.
    success : bool
        False if the simulation stopped before the stop time due to a numerical
        failure, in which case the results cover the time until the failure.

    """

    mdl: ModelTimeSeries
    ctrl: Any
    success: bool = True


class Simulation:
    """
    Simulation environment.

    Parameters
    ----------
    mdl : Model
        Continuous-time system model.
    ctrl : ControlSystem
        Discrete-time control system.
    show_progress : bool, optional
        Show progress during simulation, defaults to True.
    cfg : SolverCfg, optional
        Solver configuration parameters.

    """

    def __init__(
        self,
        mdl: Model,
        ctrl: ControlSystem,
        show_progress: bool = True,
        cfg: SolverCfg | None = None,
    ) -> None:
        if os.environ.get("BUILDING_DOCS") == "1":
            show_progress = False
        self.show_progress = show_progress
        self.cfg = cfg if cfg is not None else SolverCfg()
        self.mdl = mdl
        self.ctrl = ctrl

    def simulate(self, t_stop: float = 1.0, N_eval: int = 0) -> SimulationResults:
        """
        Solve continuous-time system model and call control system.

        Parameters
        ----------
        t_stop : float, optional
            Simulation stop time, defaults to 1.
        N_eval : int, optional
            Number of evenly spaced data points to be returned by the solver for each
            integration interval, i.e., for each sampling period or, if carrier
            comparison is used, for each switching interval. Defaults to 0, in which
            case the number and spacing of points is selected by the solver.

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
            print(f"Simulation stopped at {self.mdl.t0:.2f} s: {error}")

        # Post-process the solution data
        mdl_ts = ModelTimeSeries(self.mdl)
        ctrl_ts = self.ctrl.post_process()
        return SimulationResults(mdl_ts, ctrl_ts, success=error is None)

    @np.errstate(invalid="raise")
    def _run_simulation_loop(
        self, t_stop: float, update_progress: Callable[[], None], N_eval: int
    ) -> None:
        """Run the main simulation loop."""
        while self.mdl.t0 <= t_stop:
            # Control, computational delay, and carrier comparison
            T_s, ref_duty_ratio = self.ctrl(self.mdl)
            duty_ratio = self.mdl.delay(ref_duty_ratio)
            t_steps, q_abc, b_abc = self.mdl.pwm(T_s, duty_ratio)

            # Loop over the sampling period T_s
            for i, t_step in enumerate(t_steps):
                # Integration time span, skipped if empty
                t_span = (self.mdl.t0, self.mdl.t0 + t_step)
                if t_span[1] > t_span[0]:
                    # Set the switching state and get initial values
                    self.mdl.converter.set_gate_signals(q_abc[i], b_abc[i])
                    state0 = self.mdl.get_initial_values()

                    # Explicit Runge-Kutta methods try the whole interval as the first
                    # step, which skips the estimation of the initial step size
                    first_step = None
                    if self.cfg.method in ("RK23", "RK45", "DOP853"):
                        first_step = min(t_span[1] - t_span[0], self.cfg.max_step)

                    # Create array of evaluation times if N_eval is given, including
                    # the end point for the final state
                    if N_eval != 0:
                        t_eval = np.linspace(t_span[0], t_span[1], N_eval + 1)
                    else:
                        t_eval = None

                    # Integrate over t_span
                    sol = solve_ivp(
                        self.mdl.rhs,
                        t_span,
                        state0,
                        t_eval=t_eval,
                        first_step=first_step,
                        **self.cfg.solver,
                    )
                    if not sol.success:
                        raise FloatingPointError(sol.message)

                    # Set the final state, since the last call to rhs may be at an
                    # earlier instant (e.g., DOP853 with t_eval)
                    self.mdl.t0 = t_span[-1]
                    self.mdl.set_states(sol.y[:, -1])
                    self.mdl.set_outputs(self.mdl.t0)

                    # Save the solution (excluding the end point if N_eval is given)
                    if N_eval != 0:
                        sol.t, sol.y = sol.t[:-1], sol.y[:, :-1]
                    self.mdl.save(sol)

            # Update progress after each control step
            update_progress()
