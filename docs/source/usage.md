# Usage

After {doc}`installation`, *motulator* can be used by creating a continuous-time system model, a discrete-time control system, and a simulation object.

## Quick Start

```{include} ../../README.md
---
start-after: '## Quick start'
end-before: '## From simulation to the laboratory'
---
```

## Simulation Results

The {class}`motulator.common.model.SimulationResults` object returned by the simulation contains the time series of the system model in `res.mdl` and those of the control system in `res.ctrl`, see {doc}`/control/common/control_system`. The plotting functions, such as {func}`motulator.drive.utils.plot` and {func}`motulator.grid.utils.plot_control_signals`, plot the main signals in SI units or, if base values are given, in per-unit values.

## Conventions

- Space vectors are peak-valued complex numbers, see {doc}`/model/common/space_vectors`. The suffix `_ab` marks stationary coordinates and `_dq` synchronous (rotor) coordinates, e.g., `i_s_ab` and `psi_s_dq`. Inside the controllers, the vectors are in the controller coordinates and have no suffix.
- Reference values end in `_ref`, e.g., `tau_M_ref`. Signals inside the controllers are estimates without a special suffix.
- Lowercase `m` denotes electrical and uppercase `M` mechanical quantities, e.g., the electrical angular speed `w_m = n_p*w_M` and the mechanical angular speed `w_M`.
- SI units are used. The per-unit base values {class}`motulator.common.utils.BaseValues` are only used for plotting.
