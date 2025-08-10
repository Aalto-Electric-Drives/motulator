# Current Control

Synchronous-frame two-degrees-of-freedom (2DOF) proportional-integral (PI) current control is commonly used in three-phase AC machine drives {cite}`Har1998,Bri1999,Awa2019a`. This control structure allows compensating for the cross-coupling originating from rotating coordinates as well as to improve disturbance rejection.

A 2DOF PI current controller for induction machines is available in the {class}`motulator.drive.control.im.CurrentController` class and for synchronous machines in the {class}`motulator.drive.control.sm.CurrentController` class, both of which inherit from the {class}`motulator.common.control.ComplexPIController` class. In the following, current control of induction machines is first considered in detail. Then, the same principles are applied to synchronous machines.

## For Induction Machines

### Machine Model

The inverse-Γ model of an induction machine is considered (see {eq}`im_inv_gamma_ss` in {doc}`/model/drive/induction_machine`). The rotor flux linkage $\psiR$ and the rotor speed $\omegam$ change slowly as compared to the stator current. Consequently, it suffices to consider the stator current dynamics

```{math}
---
label: im_current
---
    \Lsgm \frac{\D \is}{\D t} = \us - (\Rsgm + \jj \omegac\Lsgm)\is - \es
```

where $\Rsgm = \Rs + \RR$ is the total resistance. The back-emf $\es$ is considered as a quasi-constant load disturbance. Here, the machine model is considered in the control system coordinates rotating at $\omegac = \hatomegas$.

### 2DOF PI Controller

The design of synchronous-frame 2DOF PI current control is considered in the continuous-time domain, even though the actual implementation is discrete. Two typical gain selections for this control type are known as the internal-model-control (IMC) design {cite}`Har1998` and the complex-vector design {cite}`Bri1999`. Here, only the complex-vector design is considered, see {ref}`complex-vector-2dof-pi-controller`, which is compatible with the {class}`motulator.common.control.ComplexPIController` base class. The controller can be expressed in a state-space form as

```{math}
---
label: cc
---
    \frac{\D \uI}{\D t} &= (\kI + \jj\hatomegas\kT )\left(\isref - \is\right) \\
    \usref &= \kT\isref - \kP\is + \uI
```

where $\usref$ is the stator voltage reference, $\isref$ is the stator current reference, and $\uI$ is the integral state. Furthermore, $\kT$ is the reference-feedforward gain, $\kP$ is the proportional gain, and $\kI$ is the integral gain.

%```{note}
%The gain definitions used in {eq}`cc` differ from {cite}`Hin2024`, where a more general controller structure is considered.
%```

### Closed-Loop System Analysis

Here, ideal voltage production is assumed, $\us = \usref$. Using {eq}`im_current` and {eq}`cc`, the closed-loop system in the Laplace domain becomes

```{math}
---
label: closed_loop_current_control
---
    \Delta\is(s) = \mathbf{G}_\mathrm{c}(s)\Delta\isref(s) - \mathbf{Y}_\mathrm{c}(s)\Delta\es(s)
```

The reference-tracking transfer function is

```{math}
---
label: Gc
---
    \mathbf{G}_\mathrm{c}(s) = \frac{(s + \jj\omegaso) \kT + \kI }{\Lsgm s^2 + (\Rsgm + \jj\omegaso \Lsgm + \kP) s + \kI + \jj\omegaso \kT}
```

where $\omegaso$ is the operating-point stator angular frequency. The closed-loop poles can be arbitrarily placed by means of the gains. The zero can be placed by means of the reference-feedforward gain $\kT$. The disturbance rejection depends on the closed-loop admittance

```{math}
---
label: Yc
---
    \mathbf{Y}_\mathrm{c}(s) = \frac{s}{\Lsgm s^2 + (\Rsgm + \jj\omegaso \Lsgm + \kP) s + \kI + \jj\omegaso \kT}
```

### Gain Selection

Consider the gains

```{math}
---
label: complex_vector_gains
---
    \kP = (\alphac + \alphai) \hatLsgm - \hatRsgm \qquad
    \kI = \alphac\alphai\hatLsgm \qquad
    \kT = \alphac \hatLsgm
```

where $\alphac$ is the closed-loop reference-tracking bandwidth and $\alphai$ is the integral action bandwidth. Assuming accurate parameter estimates, the closed-loop transfer functions {eq}`Yc` and {eq}`Gc` reduce to

```{math}
---
label: Gc_Yc
---
    \mathbf{G}_\mathrm{c}(s) = \frac{\alphac}{s + \alphac} \qquad
    \mathbf{Y}_\mathrm{c}(s) = \frac{s}{\Lsgm (s + \alphac)(s + \alphai + \jj\omegaso)}
```

It can be seen that this design results in the first-order reference-tracking dynamics. Furthermore, one pole is placed at the real axis at $s = -\alphac$, while another pole moves with the angular frequency of the coordinate system, $s= -\alphai - \jj\omegaso$. The complex-vector design tends to be slightly more robust to parameter errors than the IMC design since the other closed-loop pole approximately corresponds to the open-loop pole. Notice that $\hatRsgm = 0$ can be used in practice in {eq}`complex_vector_gains`.

This gain selection is used in the {class}`motulator.drive.control.im.CurrentController` class. The stator voltage is limited in practice due to the limited DC-bus voltage of the converter. Consequently, the realized (limited) voltage reference is

```{math}
---
label: limited_voltage
---
    \usreflim = \mathrm{sat}(\usref)
```

where $\mathrm{sat}(\cdot)$ is the saturation function. The limited voltage is obtained from the pulse-width modulation (PWM) algorithm (see the {class}`motulator.common.control.PWM` class). The anti-windup of the integrator is included in the implementation of the {class}`motulator.common.control.ComplexPIController` base class.

## For Synchronous Machines

### Machine Model

Consider the synchronous machine model in rotor coordinates rotating at $\omegam$,

```{math}
---
label: sm_model
---
    \frac{\D\psis}{\D t} &= \us - \Rs\is - \jj\omegam\psis \\
    \psis &= \psisfcn(\is)
```

where $\psisfcn(\is)$ is the flux linkage map (see {ref}`synchronous_machine`). If the magnetic saturation is omitted, the flux linkage map is $\psis = \psif + \Ld\id + \jj\Lq\iq$.

### 2DOF PI Controller

A 2DOF PI controller for synchronous machines is available in the {class}`motulator.drive.control.sm.CurrentController` class. An internal change of the state variable from the stator current to the stator flux linkage is used {cite}`Awa2019a`. This choice of using the flux linkage as the internal controller state has several advantages: the gain expressions become simpler; the same control structure can be used for salient and non-salient machines; and the magnetic saturation is easy to take into account.

For simplicity, assume perfect alignment of the controller coordinate system with the rotor coordinate system (corresponding to sensored drives). The controller can be expressed in a state-space form as

```{math}
---
label: sm_cc
---
    \frac{\D \uI}{\D t} &= (\kI + \jj\omegam\kT )\left[\hatpsisfcn(\isref) - \hatpsisfcn(\is)\right] \\
    \usref &= \kT\hatpsisfcn(\isref) - \kP\hatpsisfcn(\is) + \uI
```

where both reference and actual currents are transformed using the same flux linkage map $\hatpsisfcn$. Hence, $\isref = \is$ holds in the steady state even with flux map inaccuracies. If the magnetic saturation is omitted, this flux-linkage-based current controller becomes equivalent to a regular 2DOF PI current controller (even if inductance estimates are inaccurate).

```{note}
The control law {eq}`sm_cc` omits the effect of the stator resistance for simplicity, i.e., $\hatRs = 0$ is assumed. This is a common and practical assumption. However, if needed, the resistive voltage drop term $\hatRs\is$ could be simply added to $\usref$ in {eq}`sm_cc`.
```

### Gain Selection and Closed-Loop System Analysis

Ideal voltage production $\us = \usref$, accurate flux maps $\hatpsisfcn = \psisfcn$, and $\hatRs = \Rs = 0$ are assumed for simplicity. Consider the gains

```{math}
---
label: sm_gains
---
    \kP = \alphac + \alphai \qquad
    \kI = \alphac\alphai \qquad
    \kT = \alphac
```

Using {eq}`sm_model`--{eq}`sm_gains`, the closed-loop in the Laplace domain become

```{math}
    \frac{\Delta\psis(s)}{\Delta \psisref(s)} = \frac{\alphac(s + \alphai + \jj\omegamo)}{(s + \alphac)(s + \alphai + \jj\omegamo)} 
    = \frac{\alphac}{s + \alphac}
```

where $\omegamo$ is the operating-point rotor angular speed. Assuming linear magnetics, the above dynamics are valid for the current as well, i.e., $\Delta\is(s)/\Delta \isref(s) = \Delta\psis(s)/\Delta \psisref(s)$.

This control design corresponds to the implementation in the {class}`motulator.drive.control.sm.CurrentController` class.
