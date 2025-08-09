# Current Control

Synchronous-frame two-degrees-of-freedom (2DOF) proportional-integral (PI) current control can be used in grid converters {cite}`Har2015`. This control structure allows to compensate for the cross-coupling originating from rotating coordinates as well as to improve disturbance rejection. A 2DOF PI current controller is available in the {class}`motulator.grid.control.CurrentController` class, whose base class is {class}`motulator.common.control.ComplexPIController`.

```{note}
This controller design assumes an L filter, but it can also be applied with LCL filters (see the {doc}`/grid_examples/grid_following/plot_10kva_lcl_gfl` example). If LCL-resonance damping and very low sampling frequencies are needed, the controller could be designed directly in the discrete-time domain taking the LCL filter dynamics into account {cite}`Rah2021`.
```

## 2DOF PI Controller

The design of synchronous-frame 2DOF PI current control is considered in the continuous-time domain, even though the actual implementation is discrete. Two typical gain selections for this control type are known as the internal-model-control (IMC) design {cite}`Har1998` and the complex-vector design {cite}`Bri1999`. Here, only the complex-vector design is considered, see {ref}`complex-vector-2dof-pi-controller`, which is compatible with the {class}`motulator.common.control.ComplexPIController` base class. The controller can be expressed in a state-space form as

```{math}
---
label: grid_cc
---
\frac{\D \mathbf{u}_\mathrm{i}}{\D t} &= (\kI + \jj\omegac\kT )\left(\icref - \ic\right) \\
\ucref &= \kT\icref - \kP\ic + \mathbf{u}_\mathrm{i}
```

where $\ucref$ is the output of the controller, i.e., the converter voltage reference, $\ic$ is the measured converter current, $\icref$ is the converter current reference, $\mathbf{u}_\mathrm{i}$ is the integral state, and $\omegac$ is the angular speed of the coordinate system. Furthermore, $\kT$ is the reference-feedforward gain, $\kP$ is the proportional gain, and $\kI$ is the integral gain.

## Closed-Loop System

Consider the grid model in synchronous coordinates

```{math}
---
label: L_filt_sync
---
L\frac{\D\ic}{\D t} = \uc - \ug - \jj \omegac L \ic
```

where $\uc$ is the converter output voltage, $\ug$ is the grid (or PCC) voltage, and $L$ is the inductance. Ideal converter voltage production is assumed, $\uc = \ucref$. Using {eq}`grid_cc` and {eq}`L_filt_sync`, the closed-loop system in the Laplace domain becomes

```{math}
---
label: closed_loop_current_control_grid
---
\ic = \mathbf{G}_\mathrm{c}(s)\icref - \mathbf{Y}_\mathrm{c}(s)\ug
```

The closed-loop poles can be arbitrarily placed by means of the gains. The reference-tracking transfer function is

```{math}
---
label: Gc_grid
---
\mathbf{G}_\mathrm{c}(s) = \frac{(s + \jj\omegac) \kT + \kI }{L s^2 + (\kP + \jj\omegac L) s + \kI + \jj\omegac \kT}
```

whose zero can be placed by means of the reference-feedforward gain $\kT$. The disturbance rejection depends on the closed-loop admittance

```{math}
---
label: Yc_grid
---
\mathbf{Y}_\mathrm{c}(s) = \frac{s}{L s^2 + (\kP + \jj\omegac L) s + \kI + \jj\omegac \kT}
```

## Gain Selection

Consider the gains

```{math}
---
label: grid_cc_gain_selection
---
\kP = (\alphac + \alphai) \hat L \qquad
\kI = \alphac \alphai \hat L \qquad
\kT = \alphac \hat L
```

where $\alpha_\mathrm{s}$ is the closed-loop reference-tracking bandwidth, $\alphai$ is the integral action bandwidth, and $\hat L$ is the inductance estimate. Assuming accurate parameter estimates, the closed-loop transfer functions {eq}`Gc_grid` and {eq}`Yc_grid` reduce to

```{math}
---
label: Gc_Yc_grid
---
\mathbf{G}_\mathrm{c}(s) = \frac{\alphac}{s + \alphac} \qquad
\mathbf{Y}_\mathrm{c}(s) = \frac{s}{L (s + \alphac)(s + \alphai + \jj\omegac)}
```

It can be seen that this design results in the first-order reference-tracking dynamics. Furthermore, one pole is placed at the real axis at $s = -\alphac$ and another pole at $s= -\alphai - \jj\omegac$. This gain selection is used in the {class}`motulator.grid.control.CurrentController` class.

The converter output voltage is limited in practice due to the limited DC-bus voltage of the converter. Consequently, the realized (limited) voltage reference is

```{math}
---
label: limited_voltage_grid
---
\ucreflim = \mathrm{sat}(\ucref)
```

where $\mathrm{sat}(\cdot)$ is the saturation function. The limited voltage can be obtained from a pulse-width modulation (PWM) algorithm (see the {class}`motulator.common.control.PWM` class). The anti-windup of the integrator is included in the implementation of the {class}`motulator.common.control.ComplexPIController` base class.
