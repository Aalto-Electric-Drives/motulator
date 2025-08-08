# Current Control

Synchronous-frame two-degrees-of-freedom (2DOF) proportional-integral (PI) current control is commonly used in three-phase AC machine drives {cite}`Har1998,Bri1999,Awa2019a`. This control structure allows compensating for the cross-coupling originating from rotating coordinates as well as to improve disturbance rejection.

A 2DOF PI current controller for induction machines is available in the {class}`motulator.drive.control.im.CurrentController` class and for synchronous machines in the {class}`motulator.drive.control.sm.CurrentController` class, both of which inherit from the {class}`motulator.common.control.ComplexPIController` class. In the following, current control of induction machines is first considered in detail. Then, the same principles are applied to synchronous machines.

## Induction Machines

### System Model

The inverse-Γ model of an induction machine is considered (see {eq}`im_inv_gamma_ss` in {doc}`/model/drive/induction_machine`). The rotor flux linkage $\psiR$ and the rotor speed $\omegam$ change slowly as compared to the stator current. Consequently, it suffices to consider the stator current dynamics

```{math}
    :label: im_current

    \Lsgm \frac{\D \is}{\D t} = \us - (\Rsgm + \jj \omegac\Lsgm)\is - \es
```

where $\Rsgm = \Rs + \RR$ is the total resistance and $\omegac$ is the angular speed of the synchronous coordinate system. The back-emf $\es$ is considered as a quasi-constant load disturbance.

### 2DOF PI Controller

The design of synchronous-frame 2DOF PI current control is considered in the continuous-time domain, even though the actual implementation is discrete. Two typical gain selections for this control type are known as the internal-model-control (IMC) design {cite}`Har1998` and the complex-vector design {cite}`Bri1999`. Here, only the complex-vector design is considered, see {ref}`complex-vector-2dof-pi-controller`, which is compatible with the {class}`motulator.common.control.ComplexPIController` base class. The controller can be expressed in a state-space form as

```{math}
    :label: cc

    \frac{\D \uI}{\D t} &= (\kI + \jj\omegac\kT )\left(\isref - \is\right) \\
    \usref &= \kT\isref - \kP\is + \uI
```

where $\usref$ is the stator voltage reference, $\isref$ is the stator current reference, and $\uI$ is the integral state. Furthermore, $\kT$ is the reference-feedforward gain, $\kP$ is the proportional gain, and $\kI$ is the integral gain.

%```{note}
%The gain definitions used in {eq}`cc` differ from {cite}`Hin2024`, where a more general controller structure is considered.
%```

### Closed-Loop System

Here, ideal voltage production is assumed, $\us = \usref$. Using {eq}`im_current` and {eq}`cc`, the closed-loop system in the Laplace domain becomes

```{math}
    :label: closed_loop_current_control

    \is = \mathbf{G}_\mathrm{c}(s)\isref - \mathbf{Y}_\mathrm{c}(s)\es
```

The closed-loop poles can be arbitrarily placed by means of the gains. The reference-tracking transfer function is

```{math}
    :label: Gc

    \mathbf{G}_\mathrm{c}(s) = \frac{(s + \jj\omegac) \kT + \kI }{\Lsgm s^2 + (\Rsgm + \jj\omegac \Lsgm + \kP) s + \kI + \jj\omegac \kT}
```

whose zero can be placed by means of the reference-feedforward gain $\kT$. The disturbance rejection depends on the closed-loop admittance

```{math}
    :label: Yc

    \mathbf{Y}_\mathrm{c}(s) = \frac{s}{\Lsgm s^2 + (\Rsgm + \jj\omegac \Lsgm + \kP) s + \kI + \jj\omegac \kT}
```

### Gain Selection

Consider the gains

```{math}
    :label: complex_vector_gains

    \kP = (\alphac + \alphai) \hatLsgm - \hatRsgm \qquad
    \kI = \alphac\alphai\hatLsgm \qquad
    \kT = \alphac \hatLsgm
```

where $\alphac$ is the closed-loop reference-tracking bandwidth and $\alphai$ is the integral action bandwidth. Assuming accurate parameter estimates, the closed-loop transfer functions {eq}`Yc` and {eq}`Gc` reduce to

```{math}
    :label: Gc_Yc

    \mathbf{G}_\mathrm{c}(s) = \frac{\alphac}{s + \alphac} \qquad
    \mathbf{Y}_\mathrm{c}(s) = \frac{s}{\Lsgm (s + \alphac)(s + \alphai + \jj\omegac)}
```

It can be seen that this design results in the first-order reference-tracking dynamics. Furthermore, one pole is placed at the real axis at $s = -\alphac$, while another pole moves with the angular frequency of the coordinate system, $s= -\alphai - \jj\omegac$. The complex-vector design tends to be slightly more robust to parameter errors than the IMC design since the other closed-loop pole approximately corresponds to the open-loop pole. Notice that $\hatRsgm = 0$ can be used in practice in {eq}`complex_vector_gains`.

This gain selection is used in the {class}`motulator.drive.control.im.CurrentController` class. The stator voltage is limited in practice due to the limited DC-bus voltage of the converter. Consequently, the realized (limited) voltage reference is

```{math}
    :label: limited_voltage

    \usreflim = \mathrm{sat}(\usref)
```

where $\mathrm{sat}(\cdot)$ is the saturation function. The limited voltage is obtained from the pulse-width modulation (PWM) algorithm (see the {class}`motulator.common.control.PWM` class). The anti-windup of the integrator is included in the implementation of the {class}`motulator.common.control.ComplexPIController` base class.

## Synchronous Machines

### System Model

Consider the synchronous machine model in rotor coordinates rotating at $\omegam$,

```{math}
    :label: sm_model

    \frac{\D\psis}{\D t} &= \us - \Rs\is - \jj\omegam\psis \\
    \psis &= \psisfcn(\is)
```

where $\psisfcn(\is)$ is the flux linkage map (see {ref}`synchronous_machine`). If the magnetic saturation is omitted, the flux linkage map is $\psis = \psif + \Ld\id + \jj\Lq\iq$.

### 2DOF PI Controller

A 2DOF PI controller for synchronous machines is available in the {class}`motulator.drive.control.sm.CurrentController` class. An internal change of the state variable from the stator current to the stator flux linkage is used {cite}`Awa2019a`. This choice of using the flux linkage as the internal state has several advantages: the gain expressions become simpler; the magnetic saturation is easy to take into account; and the same control structure can be used for salient and non-salient machines.

Both the reference current and the actual current are transformed using the same flux linkage map $\hatpsisfcn(\is)$. The controller can be expressed in a state-space form as

```{math}
    :label: cc_flux

    \frac{\D \uI}{\D t} &= (\kI + \jj\omegac\kT )\left[\hatpsisfcn(\isref) - \hatpsisfcn(\is)\right] \\
    \usref &= \kT\hatpsisfcn(\isref) - \kP\hatpsisfcn(\is) + \uI
```

If the magnetic saturation is not considered, this flux-linkage-based current controller becomes equivalent to a regular 2DOF PI current controller (even if inductance estimates are inaccurate). Notice that $\isref = \is$ holds in the steady state even with inductance estimate inaccuracies, since the same inductances are used to map both the reference current and the actual current to the corresponding flux linkages.

```{note}
The control law {eq}`cc_flux` omits the effect of the stator resistance for simplicity, i.e., $\hatRs = 0$ is assumed. This is a common and practical assumption. However, if needed, the resistive voltage drop term $\hatRs\is$ could be simply added to $\usref$ in {eq}`cc_flux`.
```

The gain selection analogous to {eq}`complex_vector_gains` becomes

```{math}
    :label: sm_gains

    \kP = \alphac + \alphai \qquad
    \kI = \alphac\alphai \qquad
    \kT = \alphac
```

Assume accurate flux map estimate and perfect alignment of the controller coordinate system with the rotor coordinate system. Using {eq}`sm_model` and {eq}`cc_flux`, the closed-loop system can be shown to be analogous to the induction machine case.

This control design corresponds to the implementation in the {class}`motulator.drive.control.sm.CurrentController` class.
