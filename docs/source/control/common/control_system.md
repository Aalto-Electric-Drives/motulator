# Control System Framework

## Main Control Loop

A protocol for discrete-time control systems is available in {class}`motulator.common.control.ControlSystem` class. The main control loop has the following steps:

1. Get the measurement signals `meas` from the outputs of the continuous-time system model `mdl`.
1. Get the feedback signals `fbk` for the controllers. The feedback signals `fbk` may contain the raw measurement signals `meas`. If the control system has an observer (or phase-locked loop etc.), the feedback signals `fbk` contain the observer output signal.
1. Compute the reference signals `ref` based on the feedback signals `fbk`.
1. Save the feedback signals `fbk` and the reference signals `ref` so they can be accessed after the simulation.
1. Update the states of the control system for the next sampling instant.
1. Return the sampling period `T_s` and the duty ratios `d_abc` for the carrier comparison.

Using this protocol is not compulsory, but it may simplify the implementation of new control systems.

## Data Flow and Storage

[Figure 1](fig:overall_system) illustrates the structure and data flow in a typical simulation model. [Figure 2](fig:discrete_control_system) exemplifies an internal structure of a typical control system. The text in italics refers to the default object names used in the software.

In discrete-time control systems, the signals are collected into these key objects during the simulation:

`meas`
: Contains raw measured signals from sensors (e.g., `meas.u_dc` for measured DC-bus voltage).

`fbk`
: Contains signals used by controllers, including both measured quantities (`fbk.u_dc`) and estimated values (such as `fbk.psi_s` for stator flux linkage). The `fbk` object is saved at each controller time step.

`ref`
: Contains reference signals, both external references (such as `ref.w_m` for speed reference) and internally generated control outputs (such as `ref.d_abc` for converter duty ratios). The `ref` object is saved at each controller time step.

The objects `fbk` and `ref` may propagate through multiple control blocks (implemented as classes), with each block potentially adding or modifying signals.

The states and inputs of the continuous-time system model are saved at every solver time step.

```{figure} ../figs/overall_system.svg
---
name: fig:overall_system
class: only-light
width: 100%
align: center
alt: Block diagram of the control system (light mode)
---
*Figure 1:* Block diagram illustrating the structure and data flow in a typical simulation model. The lower part of the figure illustrates how the data is saved. The post-processing is automatically done after the simulation. The internal structure of a typical control system is exemplified in the figure below.
```

```{figure} ../figs/overall_system.svg
---
class: invert-colors-dark only-dark
width: 100%
align: center
alt: Block diagram of the control system (dark mode)
---
*Figure 1:* Block diagram illustrating the structure and data flow in a typical simulation model. The lower part of the figure illustrates how the data is saved. The post-processing is automatically done after the simulation. The internal structure of a typical control system is exemplified in the figure below.
```

```{figure} ../figs/discrete_control_system.svg
---
name: fig:discrete_control_system
class: only-light
width: 100%
align: center
alt: Block diagram of the discrete control system (light mode)
---
*Figure 2:* Block diagram exemplifying the internal structure of a typical cascade control system. The object `ref` at the control system output should contain the sampling period `T_s` and the converter duty ratios `d_abc` for the carrier comparison. The observer does not necessarily exist in all control systems (or it can be replaced with, e.g., a phase-locked loop).
```

```{figure} ../figs/discrete_control_system.svg
---
class: invert-colors-dark only-dark
width: 100%
align: center
alt: Block diagram of the discrete control system (dark mode)
---
*Figure 2:* Block diagram exemplifying the internal structure of a typical cascade control system. The object `ref` at the control system output should contain the sampling period `T_s` and the converter duty ratios `d_abc` for the carrier comparison. The observer does not necessarily exist in all control systems (or it can be replaced with, e.g., a phase-locked loop).
```

## Accessing the Data

Time series of simulation results are available as NumPy arrays in the {class}`motulator.common.model.SimulationResults` object, named `res` in the following:

- Feedback signals: `res.ctrl.fbk`
- Reference signals: `res.ctrl.ref`
- System model signals: `res.mdl`

The {doc}`/drive_examples/index` and {doc}`/grid_examples/index` example scripts demonstrate how to access and visualize the data.
