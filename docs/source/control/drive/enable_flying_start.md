# Enable and Flying-Start

Drive control systems accept a constant enable command or a function of controller
time, using the same interface as external references:

```python
ctrl.set_enable(False)
ctrl.set_enable(lambda t: t >= 0.2)
```

The command is sampled when the controller runs. The converter gates are enabled
only when both `ctrl.set_enable(...)` and `mdl.set_enable(...)` permit operation.
Neither command overrides a disable command from the other side. The default
command is True on both sides.

While disabled, the drive's power regulators and reference generators stop
integrating. Observers continue running using the measured converter terminal
voltage instead of reconstructing it from the blocked PWM commands. The voltage
measurement is ideal. Diode conduction and passive rectification remain part of the
physical converter model, see {doc}`/model/common/converters`.
No torque or speed reference is needed while disabled.

## Sensored Flying-Start

Flying-start is an explicit option of `VectorControlSystem`:

```python
import motulator.drive.control.sm as control

cfg = control.CurrentVectorControllerCfg(i_s_max=10, sensorless=False)
inner = control.CurrentVectorController(par, cfg)
ctrl = control.VectorControlSystem(
    inner, flying_start=control.FlyingStartCfg(T_ramp=0.05)
)
ctrl.set_enable(lambda t: t >= 0.2)
ctrl.set_torque_ref(3)
```

Both synchronous and induction machine vector controllers support sensored
flying-start, including current-vector control with continuous or discrete current
regulators and flux-vector control. Pass a speed controller as usual to use speed
control instead of torque control. Flying-start is disabled by default; sensorless
controllers are rejected when this option is requested. V/Hz controllers support
`set_enable` but do not implement flying-start. Flying-start currently requires a
direct connection between converter and machine, without an output LC filter.

On initial startup and each re-enable, the controller:

1. Synchronizes the observer from measured shaft speed and position. For synchronous
   machines, the estimated flux is initialized using the measured current and the
   controller's magnetic map. Induction-machine residual flux is preserved and
   tracked by its observer, rather than inferred from mechanical position.
1. Keeps the gates blocked while feeding synchronized voltage commands through the
   computational delay for `delay + 1` sampling periods.
1. Initializes current or flux/torque regulators at the observed operating point,
   and initializes the optional speed regulator for zero initial torque demand.
1. Enables the gates and ramps the requested torque from zero over `T_ramp` seconds.
   Existing current, torque, flux, and voltage limits continue to apply. An initially
   unmagnetized induction motor builds flux before it can produce the requested
   torque.

The algorithm uses sensor interfaces and controller parameter estimates; it does
not read the model's rotor speed, position, or flux states directly. A synchronous
machine whose generated voltage exceeds the available DC voltage can still
produce passive diode currents. Flying-start cannot remove those currents; the
current controller's field-weakening and voltage limits govern active takeover.

`res.ctrl.ref.enabled` records the effective gate command and
`res.ctrl.ref.flying_start` identifies PWM priming and the torque ramp. For a
composite model, these signals appear under its controller name, e.g.,
`res.ctrl.drive.ref.enabled`. The mechanical-system
{doc}`/drive_examples/current_vector/plot_ipmsm_drive_enable` example demonstrates
coasting, passive rectification, and sensored torque takeover.
The matching {doc}`/drive_examples/current_vector/plot_im_drive_enable` example
uses the same speed reference, gate-enable schedule, and mechanical system for an
induction motor, including residual flux decay and flux rebuilding at takeover.
For disabling and re-enabling an IPMSM above base speed, see
{doc}`/drive_examples/current_vector/plot_ipmsm_drive_enable_field_weakening`.
