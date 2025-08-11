# Speed Control

A speed controller is implemented in the {class}`motulator.drive.control.im.SpeedController` class, whose base class is {class}`motulator.common.control.PIController`. In the following, the tuning of the speed controller is discussed. The presented approach can be extended to many other control tasks as well.

(2dof-pi-controller)=

## 2DOF PI Controller

Even if controllers operate in the discrete-time domain, they are often designed and analyzed in the continuous-time domain. The state-space form of a simple 2DOF PI speed controller is given by {cite}`Hin2024`

```{math}
---
label: speed_ctrl
---
    \frac{\D \tau_\mathrm{i}}{\D t} &= \ki\left(\omegaMref - \omegaM\right) \\
    \tauMref &= \kt\omegaMref - \kp\omegaM + \tau_\mathrm{i}
```

where $\omegaM$ is the measured (or estimated) mechanical angular speed of the rotor, $\omegaMref$ is the reference angular speed, and $\tau_\mathrm{i}$ is the integral state. Furthermore, $\kt$ is the reference feedforward gain, $\kp$ is the proportional gain, and $\ki$ is the integral gain. Setting $\kt = \kp$ results in the standard PI controller. This 2DOF PI controller can also be understood as a state feedback controller with integral action and reference feedforward {cite}`Fra1997`.

## Closed-Loop System Analysis

For simplicity, let us assume ideal torque control ($\tauM = \tauMref$) and a stiff mechanical system

```{math}
---
label: stiff_mech
---
    J\frac{\D\omegaM}{\D t} = \tauM - \tauL
```

where $\tauM$ is the electromagnetic torque, $\tauL$ is the load torque, and $J$ is the total moment of inertia. In the Laplace domain, the closed-loop system resulting from {eq}`speed_ctrl` and {eq}`stiff_mech` is given by

```{math}
---
label: speed_ctrl_closed_loop_system
---
    \omegaM(s) = \frac{\kt s + \ki}{J s^2 + \kp s + \ki} \omegaMref(s) - \frac{s}{J s^2 + \kp s + \ki} \tauL(s)
```

where it can be seen that the gain $\kt$ allows to place the reference-tracking zero.

## Gain Selection

The gain selection {cite}`Har2013`

```{math}
---
label: speed_ctrl_gain_selection
---
    \kt = \alphas \hat{J} \qquad
    \kp = (\alphas + \alphai) \hat{J} \qquad
    \ki = \alphas\alphai \hat{J}
```

results in

```{math}
---
label: speed_ctrl_closed_loop_system2
---
    \omegaM(s) = \frac{\alphas}{s + \alphas} \omegaMref(s) - \frac{s}{J (s + \alphas)(s + \alphai)} \tauL(s)
```

where $\alphas$ is the closed-loop reference-tracking bandwidth and $\alphai$ is the integral action bandwidth. An accurate inertia estimate $\hat{J} = J$ is assumed in the above closed-loop system.
