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

`Simulation.simulate()` alternates between the control systems and the ODE solver:

1. Each control system is called at its own sampling instants (the first one at `mdl.t0`, or at `t_start` given to `Simulation`): `ctrl(mdl)` returns `(T_s, command)`, where the command consists of the duty ratios `d_abc` for a voltage-source converter. The controller can change `T_s` at every step. Before the call, the gate enable `ctrl.get_enable()` is applied to the converter, so that the measurements see a converter with blocked gates; the diode transitions of a blocked converter are located as solver events.
1. `converter.modulate(T_s, command)` converts the command into the gate states and their durations through the modulator of the converter (`converter.modulator`), which includes the computational delay: `ZOH` (averaged, default) or `CarrierComparison`, both with the dead time `t_d`. The simulation sets the gate states at the switching instants with `converter.set_gates`.
1. `solve_ivp` integrates `mdl.rhs` between the sampling and switching instants of all the converters, with the switching states `converter.inp.q_c_ab` held constant. The integration restarts at each diode transition of the blocked converters (`converter.blocked`), and repeated transitions at the same instant stop the simulation as a numerical failure.

A `Model` has at most one converter. A controlled converter (`ControlledConverter`, e.g., `VoltageSourceConverter`) is driven by a control system, while a passive converter (`DiodeBridge`) and a model without a converter (`drive.model.GridConnectedMachine`) have none. The diode states and the DC bus are optional capabilities of the `Converter` protocol with defaults (never blocked, no DC bus), so a converter without them implements only the subsystem interface and the gate control. Without control systems, the simulation integrates until `t_stop` as a single interval, split only by the diode transitions. A `CompositeModel` combines several models (e.g., a `GridConverterSystem` with a `DiodeBridge` or an active front end and a `Drive` sharing a `DCBus`), each with its own converter and modulator; `Simulation` then takes a dict of control systems keyed by the names of the models with controlled converters, and the results are grouped by the model names (`res.mdl.drive`, `res.ctrl.drive`, which is None without a control system). The `DCBus` computes the DC currents of its converters from their inputs in `rhs()` instead of through the connections, since a converter output computed in `set_outputs()` would be one evaluation stale.

### System models (`common/model/_base.py`)

A `Model` (e.g., `drive.model.Drive`, `grid.model.GridConverterSystem`) is a dict of named subsystems plus a wiring table, with an optional converter. Each subsystem has `inp`, `out`, and `state` dataclasses and implements `set_outputs(t)` and `rhs(t)`. The converter (or None), the subsystems by name, and the `connections` dict, which maps `(target_name, input_attr)` to `(source_name, output_attr)`, are passed to `Model.__init__`, optionally with a `DCBus`, which is added under the name `dc_bus` and registers the converter and gives the connections of its DC terminals (`DCBus.connect`). The names of the subsystems name their time series in the results (`res.mdl.machine`). The outputs are computed in the order of the `subsystems` dict and passed to the connected inputs immediately, so a subsystem whose outputs depend directly on its inputs (e.g., a machine with core losses) must come after its input sources. The voltage of a blocked converter depends on its load through the AC port (`ACPort`), so it is computed after the other subsystems. A converter keeps its signals that are not states (e.g., the switching state and the voltage of a blocked converter, evaluated at the solution points) in its history through `extend_history`. The post-processing (`ModelTimeSeries`) computes the output time series in the same order (`set_outputs(t, subsystem)` of the time series), and then the other signals that depend on the inputs (`compute_derived_signals`). States are gathered from the `state` dataclass fields in declaration order, so the field order defines the solver's state vector.

### Control systems (`common/control/_base.py`)

`ControlSystem.run_control_loop()` calls `get_measurement(mdl)` → `get_feedback(meas)` → `compute_output(fbk)` → `save(...)` → `update(ref, fbk)` → `get_command(ref)`, which the drive and grid control systems implement as `(ref.T_s, ref.d_abc)`. The `ref` and `fbk` dataclasses are logged automatically at every step and turned into numpy arrays by `post_process()`, so a signal becomes available in the results simply by adding a field to them. Drive control systems (`VectorControlSystem`, `VHzControlSystem`) wrap an inner controller (e.g., `FluxVectorController`) plus an optional `SpeedController`. Controllers take a parameter object (e.g., `SynchronousMachinePars`) and a `...Cfg` dataclass.

### GradNet (`drive/gradnet/`)

Neural-network flux and current maps learned from measured or FEM data, used by the saturated machine models and controllers. The networks are trained with PyTorch in the related package motulator-gradnet and saved as npz files (`GradNetFunction`), which the maps evaluate with NumPy in double precision. The npz format and the activation functions are the interface between the packages. Trained models are in `examples/drive/flux_vector/trained_models/`.

## Conventions

- Peak-valued complex space vectors. Suffixes mark the coordinates: `_ab` is stationary and `_dq` synchronous (e.g., `i_s_ab`, `psi_s_dq`). Reference values end in `_ref`. Lowercase `m` denotes electrical and uppercase `M` mechanical quantities (e.g., `w_m = n_p*w_M`, `tau_M`). Signals inside controllers are estimates without a special suffix, while estimated parameter objects are named `est_par`.
- SI units. Per-unit `BaseValues` are only used for plotting.
- NumPy-style docstrings on public classes and functions (rendered by sphinx-autoapi). Line length is 88.
- `BUILDING_DOCS=1` is set during the docs build and disables progress bars.
- Avoid hacks. Do not attach undeclared attributes to objects (e.g., `vars(obj)[...] = ...` or `setattr`), monkey-patch objects, or bypass the type checker to get around an interface. Extend through the existing interfaces instead (subclass methods, dataclass fields, signal groups passed to `ControlSystem.save()`). If a clean solution needs a change in a base class, propose the change rather than working around it.

### Examples

Sphinx-Gallery renders `examples/**/plot_*.py` into the documentation, so their format matters: a module docstring with a title and `=` underline, and `# %%` cell separators followed by comment text. Scripts not starting with `plot_` are not executed during the docs build. Section order and in-section sorting (by code length) are set in `sphinx_gallery_conf` in `docs/source/conf.py`. Citations such as `[#Qu2012]_` refer to footnotes at the end of the example.
