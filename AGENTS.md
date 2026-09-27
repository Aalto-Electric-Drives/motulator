# AGENTS.md

This file provides guidance to AI coding agents working with code in this repository.

## Commands

Development uses a virtual environment in `.venv` (pyright is configured to use it). Install with `pip install -e .[dev,doc]`.

- Lint, format, and type check (the same checks as CI): `pre-commit run --all-files`
- Single file: `ruff check --fix <file>`, `ruff format <file>`, `pyright <file>`
- Build the docs (runs every example in `examples/`, slow): `make html` in `docs/`, or `sphinx-build -b html docs/source docs/build/html`
- Run one example: `python examples/drive/flux_vector/plot_2kw_ipmsm_fvc.py`
- Run the tests (about 30 s): `pytest`. A single test: `pytest tests/test_drive.py -k ipmsm_fvc_sensorless`

The tests in `tests/` run short closed-loop simulations and check physical properties of the response (e.g., speed tracking and torque balance) with loose tolerances, so they tolerate retuning of the controllers. The docstring examples in `motulator/` are run as doctests. The docs build additionally executes all examples, but it only checks that they run.

## Architecture

The package is split into `common` (shared base classes), `drive` (machine drives), and `grid` (grid converters). Each has `model/` (continuous-time system models), `control/` (discrete-time control algorithms), and `utils/`. Modules are private (`_name.py`), and the public API is re-exported through the `__init__.py` files. Drive controllers are grouped by machine type: `motulator.drive.control.sm` and `motulator.drive.control.im`.

### Simulation loop (`common/model/_simulation.py`)

`Simulation.simulate()` alternates between the control system and the ODE solver:

1. The control system is called once per sampling period: `ctrl(mdl)` returns `(T_s, d_abc)`. The controller can change `T_s` at every step.
1. `mdl.delay` applies the computational delay, and `mdl.pwm` converts the duty ratios into switching intervals: `ZOH` (averaged, default) or `CarrierComparison` (`pwm=True` in the model constructor).
1. `solve_ivp` integrates `mdl.rhs` over each switching interval, with the switching state held as a ZOH input.

### System models (`common/model/_base.py`)

A `Model` (e.g., `drive.model.Drive`, `grid.model.GridConverterSystem`) is a list of subsystems plus a wiring table. Each subsystem has `inp`, `out`, and `state` dataclasses and implements `set_outputs(t)` and `rhs(t)`. The model's `connections` dict maps `(target, input_attr)` to `(source, output_attr)`, and `zoh_connections` maps inputs to ZOH signals such as `sw_state`. States are gathered from the `state` dataclass fields in declaration order, so the field order defines the solver's state vector. New subsystems follow this pattern and are added to the model's `subsystems` and `connections`.

### Control systems (`common/control/_base.py`)

`ControlSystem.run_control_loop()` calls `get_measurement(mdl)` → `get_feedback(meas)` → `compute_output(fbk)` → `save(...)` → `update(ref, fbk)`. The `ref` and `fbk` dataclasses are logged automatically at every step and turned into numpy arrays by `post_process()`, so a signal becomes available in the results simply by adding a field to them. Drive control systems (`VectorControlSystem`, `VHzControlSystem`) wrap an inner controller (e.g., `FluxVectorController`) plus an optional `SpeedController`. Controllers take a parameter object (e.g., `SynchronousMachinePars`) and a `...Cfg` dataclass.

### GradNet (`drive/gradnet/`)

Neural-network flux and current maps (PyTorch) learned from measured or FEM data, used by the saturated machine models and controllers. Trained models and datasets are in `examples/drive/gradnet/`.

## Conventions

- Peak-valued complex space vectors. Suffixes mark the coordinates: `_ab` is stationary and `_dq` synchronous (e.g., `i_s_ab`, `psi_s_dq`). Reference values end in `_ref`. Lowercase `m` denotes electrical and uppercase `M` mechanical quantities (e.g., `w_m = n_p*w_M`, `tau_M`). Signals inside controllers are estimates without a special suffix, while estimated parameter objects are named `est_par`.
- SI units. Per-unit `BaseValues` are only used for plotting.
- NumPy-style docstrings on public classes and functions (rendered by sphinx-autoapi). Line length is 88.
- `BUILDING_DOCS=1` is set during the docs build and disables progress bars.

### Examples

Sphinx-Gallery renders `examples/**/plot_*.py` into the documentation, so their format matters: a module docstring with a title and `=` underline, and `# %%` cell separators followed by comment text. Scripts not starting with `plot_` (e.g., `train_*.py`) are not executed during the docs build. Section order and in-section sorting (by code length) are set in `sphinx_gallery_conf` in `docs/source/conf.py`. Citations such as `[#Qu2012]_` refer to footnotes at the end of the example.
