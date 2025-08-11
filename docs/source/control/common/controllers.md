# Controllers

## 2DOF PI Controller

Proportional-integral (PI) control is widely used in various applications. A standard one-degree-of-freedom (1DOF) PI controller manipulates only the control error, i.e., it has single input and single output. Its two-degrees-of-freedom (2DOF) variants have two inputs (reference signal and feedback signal), which allows to design disturbance rejection and reference tracking separately {cite}`Sko1996`. The 2DOF PI controller is available in the {class}`motulator.common.control.PIController` class, which is the base class for the {class}`motulator.drive.control.im.SpeedController` and {class}`motulator.grid.control.DCBusVoltageController` classes.

### Typical Structure

[Figure 1](fig:2dof_pi) shows a 2DOF PI controller with an optional feedforward term. Its equivalent state-space form is given by

```{math}
---
label: 2dof_pi
---
    \frac{\D \ui}{\D t} &= \ki\left(r - y\right) \\
    u &= \kt r - \kp y + \ui + \uff
```

where $r$ is the reference signal, $y$ is the measured (or estimated) feedback signal, $\ui$ is the the integral state, and $\uff$ is the optional feedforward signal. Furthermore, $\kt$ is the reference-feedforward gain, $\kp$ is the proportional gain, and $\ki$ is the integral gain. Setting $\kt = \kp$ and $\uff = 0$ results in the standard PI controller. This 2DOF PI controller can also be understood as a state-feedback controller with integral action and reference feedforward {cite}`Fra1997`.

```{figure} ../figs/2dof_pi.svg
---
name: fig:2dof_pi
class: only-light
width: 100%
align: center
alt: 2DOF PI controller
---
*Figure 1:* 2DOF PI controller with an optional feedforward term. The operator $1/s$ refers to integration. A discrete-time variant of this controller with the integrator anti-windup is implemented in the {class}`motulator.common.control.PIController` class.
```

```{figure} ../figs/2dof_pi.svg
---
class: invert-colors-dark only-dark
width: 100%
align: center
alt: 2DOF PI controller
---
*Figure 1:* 2DOF PI controller with an optional feedforward term. The operator $1/s$ refers to integration. A discrete-time variant of this controller with the integrator anti-windup is implemented in the {class}`motulator.common.control.PIController` class.
```

(disturbance-observer-structure)=

### Disturbance-Observer Structure

The controller {eq}`2dof_pi` can be equally represented using the disturbance-observer structure as

```{math}
---
label: 2dof_pi_disturbance_observer
---
    \frac{\D \ui}{\D t} &= \alphai\left(u - \hat v\right) \\
    \hat v &= \ui - (\kp - \kt)y + \uff \\
    u &= \kt\left(r - y\right) + \hat v
```

where $\alphai = \ki/\kt$ is the redefined integral gain and $\hat v$ is the input-equivalent disturbance estimate. This structure is convenient to prevent the integral windup that originates from the actuator saturation {cite}`Fra1997`. The actuator output is limited in practice due to physical constraints. Consequently, the realized actuator output is

```{math}
    \bar{u} = \mathrm{sat}(u)
```

where $\mathrm{sat}(\cdot)$ is the saturation function. If this saturation function is known, the anti-windup of the integrator can be implemented simply as

```{math}
---
label: anti_windup
---
    \frac{\D \ui}{\D t} = \alphai\left(\bar{u} - \hat v \right)
```

The other parts of the above controller are not affected by the saturation.

### Discrete-Time Algorithm

The discrete-time variant of the controller {eq}`2dof_pi_disturbance_observer` with the anti-windup in {eq}`anti_windup` is given by

```{math}
---
label: discrete_2dof_pi
---
    \ui(k+1) &= \ui(k) + \Ts \alphai \left[\bar{u}(k) - \hat v(k) \right] \\
    \hat v(k) &= \ui(k) - (\kp - \kt)y(k) + \uff(k) \\
    u(k) &= \kt\left[r(k) - y(k)\right] + \hat v(k) \\
    \bar{u}(k) &= \mathrm{sat}[u(k)]
```

where $\Ts$ is the sampling period and $k$ is the discrete-time index. This algorithm corresponds to the actual implementation in the {class}`motulator.common.control.PIController` class.

(complex-vector-2dof-pi-controller)=

## Complex-Vector 2DOF PI Controller

As shown in [Figure 2](fig:complex_vector_2dof_pi), the 2DOF PI controller presented above can be extended for the control of complex-valued space vectors in a coordinate system rotating at $\omegac$ {cite}`Bri1999`. Depending on the control task, the controlled quantity is typically either a current vector or a flux linkage vector. In the continuous-time domain, the controller in the state-space form is given by

```{math}
---
label: complex_vector_2dof_pi
---
    \frac{\D \uI}{\D t} &= (\kI + \jj\omegac \kT)\left(\mathbf{r} - \mathbf{y}\right) \\
    \mathbf{u} &= \kT\mathbf{r} - \kP\mathbf{y} + \uI + \uFF
```

where $\mathbf{u}$ is the output of the controller, $\mathbf{r}$ is the reference signal, $\uI$ is the the integral state, and $\uFF$ is the optional feedforward signal. Furthermore, $\kT$ is the reference-feedforward gain, $\kP$ is the proportional gain, and $\kI$ is the integral gain.

```{figure} ../figs/complex_vector_2dof_pi.svg
---
name: fig:complex_vector_2dof_pi
class: only-light
width: 100%
align: center
alt: 2DOF complex-vector PI controller with feedforward
---
*Figure 2:* 2DOF complex-vector PI controller with an optional feedforward term.
```

```{figure} ../figs/complex_vector_2dof_pi.svg
---
class: invert-colors-dark only-dark
width: 100%
align: center
alt: 2DOF complex-vector PI controller with feedforward
---
*Figure 2:* 2DOF complex-vector PI controller with an optional feedforward term.
```

The discrete-time implementation of {eq}`complex_vector_2dof_pi` with the anti-windup is given in the {class}`motulator.common.control.ComplexPIController` class, which is the base class for {class}`motulator.drive.control.sm.CurrentController`, {class}`motulator.drive.control.im.CurrentController`, and {class}`motulator.grid.control.CurrentController` classes. The algorithm is similar to the real-valued case given in {eq}`discrete_2dof_pi`.

The converter output voltage is limited due to the limited DC-bus voltage. The realized switching-cycle averaged voltage is obtained from the pulse-width modulation (PWM) algorithm. The {class}`motulator.common.control.PWM` class implements the space-vector modulation {cite}`Hav1999` with different overmodulation strategies (minimum phase error, minimum amplitude error, six-step {cite}`Bol1997`). In the computation of the realized voltage, the effect of the computational and PWM delays on the realized voltage angle are also compensated for {cite}`Bae2003`.
