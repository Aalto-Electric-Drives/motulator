"""Simulation environment."""

import os
from dataclasses import dataclass
from typing import Any, Callable

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

    """

    mdl: ModelTimeSeries
    ctrl: Any


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
        N_eval : int | None, optional
            Number of evenly spaced data points to be returned by the solver for each
            sampling period. Defaults to 0, in which case the number and spacing of
            points is selected by the solver.

        """
        try:
            # Initialize outputs based on initial states
            self.mdl.set_outputs(0.0)

            # Main simulation loop
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

            self._run_simulation_loop(t_stop, update_progress, N_eval)

            if progress_bar is not None:
                progress_bar.n = t_stop
                progress_bar.refresh()
                progress_bar.close()

        except FloatingPointError:
            print(f"Invalid value encountered at {self.mdl.t0:.2f} s.")

        # Post-process the solution data
        mdl_ts = ModelTimeSeries(self.mdl)
        ctrl_ts = self.ctrl.post_process()
        return SimulationResults(mdl_ts, ctrl_ts)

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
                if t_step > 0:
                    # Set the switching state and get initial values
                    self.mdl.converter.set_gate_signals(q_abc[i], b_abc[i])
                    state0 = self.mdl.get_initial_values()

                    # Set the integration time span
                    t_span = (self.mdl.t0, self.mdl.t0 + t_step)

                    # Create array of evaluation times if N_eval is given
                    if N_eval != 0:
                        t_eval = np.linspace(
                            t_span[0], t_span[1], N_eval, endpoint=False
                        )
                    else:
                        t_eval = None

                    # Integrate over t_span
                    sol = solve_ivp(
                        self.mdl.rhs, t_span, state0, t_eval=t_eval, **self.cfg.solver
                    )

                    # Set the new initial time and save the solution
                    self.mdl.t0 = t_span[-1]
                    self.mdl.save(sol)

            # Update progress after each control step
            update_progress()
