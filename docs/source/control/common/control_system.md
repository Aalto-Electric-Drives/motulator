# Control System Framework

## Main Control Loop

The base class {class}`motulator.common.control.ControlSystem` implements the main control loop of discrete-time control systems, which has the following steps:

1. Sample the measurement signals `meas` from the sensors of the continuous-time system model `mdl`, see {class}`motulator.common.model.Sensor`.
1. Get the feedback signals `fbk` for the controllers. The feedback signals `fbk` may contain the raw measurement signals `meas`. If the control system has an observer (or phase-locked loop etc.), the feedback signals `fbk` contain the observer output signal.
1. Compute the reference signals `ref` based on the feedback signals `fbk`.
1. Save the feedback signals `fbk` and the reference signals `ref` so they can be accessed after the simulation.
1. Update the states of the control system for the next sampling instant.
1. Return the sampling period `T_s` and the duty ratios `d_abc` for the modulator of the converter.

Using this base class is not compulsory, but it simplifies the implementation of new control systems.

## Gate Enable

The control system can block the gates of the converter with `set_enable`, constantly or as a function of time, which is evaluated at the sampling instants. While the gates are blocked, the control loop runs as usual, but the reference signals are computed by `compute_standby_output` instead of `compute_output`, and the outer controllers (the speed controller and the DC-bus voltage controller) are not updated. The converter voltage is not measured. The grid converter control systems use the measured PCC voltage in its place, which it approximately equals once the current has decayed, so that the phase-locked loop or the observer keeps tracking the grid. In the drive control systems, the observers, if any, run on their open-circuit model, and the vector control systems compute the output for zero torque. A sensorless synchronous machine drive catches the rotating machine after the release with a flying start, see {class}`motulator.drive.control.sm.VectorControlSystem`. The model of the converter with blocked gates is described in {doc}`/model/common/converters`.

The drive control systems and the grid converter control systems implement the standby output, except with the power-synchronization controller, which cannot synchronize with the grid without power. A sensorless induction machine drive holds its speed estimate while the gates are blocked, and its decayed rotor flux does not allow a flying start, so a speed search, not included in this package, is needed if the speed changes.

## Data Flow and Storage

[Figure 1](fig:overall_system) illustrates the structure and data flow in a typical simulation model. [Figure 2](fig:discrete_control_system) exemplifies an internal structure of a typical control system. The text in italics refers to the default object names used in the software.

In discrete-time control systems, the signals are collected into these key objects during the simulation:

`meas`
: Contains raw measured signals from sensors (e.g., `meas.u_dc` for measured DC-bus voltage).

`fbk`
: Contains signals used by controllers, including both measured quantities (`fbk.u_dc`) and estimated values (such as `fbk.psi_s` for stator flux linkage). The `fbk` object is saved at each controller time step.

`ref`
: Contains reference signals, both external references (such as `ref.w_M` for speed reference) and internally generated control outputs (such as `ref.d_abc` for converter duty ratios). The `ref` object is saved at each controller time step.

The objects `fbk` and `ref` may propagate through multiple control blocks (implemented as classes), with each block potentially adding or modifying signals.

The states of the continuous-time system model and the switching states of the converters are saved at the solution points of the solver. The other signals, such as the inputs and outputs of the subsystems, are computed from them after the simulation.

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
*Figure 2:* Block diagram exemplifying the internal structure of a typical cascade control system. The object `ref` at the control system output should contain the sampling period `T_s` and the converter duty ratios `d_abc` for the modulator of the converter. The observer does not necessarily exist in all control systems (or it can be replaced with, e.g., a phase-locked loop).
```

```{figure} ../figs/discrete_control_system.svg
---
class: invert-colors-dark only-dark
width: 100%
align: center
alt: Block diagram of the discrete control system (dark mode)
---
*Figure 2:* Block diagram exemplifying the internal structure of a typical cascade control system. The object `ref` at the control system output should contain the sampling period `T_s` and the converter duty ratios `d_abc` for the modulator of the converter. The observer does not necessarily exist in all control systems (or it can be replaced with, e.g., a phase-locked loop).
```

## Accessing the Data

Time series of simulation results are available as NumPy arrays in the {class}`motulator.common.model.SimulationResults` object, named `res` in the following:

- Feedback signals: `res.ctrl.fbk`
- Reference signals: `res.ctrl.ref`
- System model signals: `res.mdl`

The {doc}`/drive_examples/index` and {doc}`/grid_examples/index` example scripts demonstrate how to access and visualize the data.
