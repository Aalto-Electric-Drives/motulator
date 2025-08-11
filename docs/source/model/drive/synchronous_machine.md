# Synchronous Machines

This document describes continuous-time synchronous machine models of the {mod}`motulator.drive.model` package.

(synchronous_machine)=

## Synchronous Machine Model

[Figure 1](fig:sm) shows the space-vector equivalent circuit model of a synchronous machine in rotor coordinates. The model is implemented in the {class}`motulator.drive.model.SynchronousMachine` class. The synchronous machine model can be parametrized to represent permanent-magnet synchronous machines (PMSMs) and synchronous reluctance machines (SyRMs).

```{figure} ../figs/sm.svg
---
name: fig:sm
class: only-light
width: 100%
align: center
alt: Equivalent circuit model of a synchronous machine
---
*Figure 1:* Space-vector equivalent circuit model of a synchronous machine in rotor coordinates. The nonlinear stator inductance is defined by the current map $\is = \isfcn(\psis)$.
```

```{figure} ../figs/sm.svg
---
class: invert-colors-dark only-dark
width: 100%
align: center
alt: Equivalent circuit model of a synchronous machine
---
*Figure 1:* Space-vector equivalent circuit model of a synchronous machine in rotor coordinates. The nonlinear stator inductance is defined by the current map $\is = \isfcn(\psis)$.
```

The state equations in rotor coordinates are {cite}`Jah1986`

```{math}
---
label: sm_states
---
    \frac{\D\psis}{\D t} &= \us - \Rs\is - \jj\omegam\psis \\
    \frac{\D\thetam}{\D t} &= \omegam
```

where $\us$ is the stator voltage and $\is$ is the stator current. The stator resistance is denoted by $\Rs$. The rotor angular speed is $\omegam = \np \omegaM$, where $\omegaM$ is the mechanical angular speed of the rotor and $\np$ is the number of pole pairs. The current $\is = \id + \jj \iq$ depends on the flux linkage $\psis = \psid + \jj \psiq$ as

```{math}
---
label: sm_current
---
    \is &= \isfcn(\psis) \\
    &= \idfcn(\psid, \psiq) + \jj\iqfcn(\psid, \psiq)
```

where the complex function $\isfcn$ is referred to as a current map. The electromagnetic torque is

```{math}
---
label: sm_torque
---
    \tauM = \frac{3 \np}{2}\IM\left\{\is\psis^*\right\}
```

[Figure 2](fig:sm_block_rot) shows the block diagram of the machine model in rotor coordinates. The magnetic model includes the current map and the torque equation. Since the machine is fed and observed from stator coordinates, the quantities are transformed accordingly, as shown in [Figure 3](fig:sm_block_stat).

```{figure} ../figs/sm_block_rot.svg
---
name: fig:sm_block_rot
class: only-light
width: 100%
align: center
alt: Synchronous machine model
---
*Figure 2:* Block diagram of the machine model in rotor coordinates. The magnetic model includes the flux equation (or, optionally, saturation characteristics) and the torque equation.
```

```{figure} ../figs/sm_block_rot.svg
---
class: invert-colors-dark only-dark
width: 100%
align: center
alt: Synchronous machine model
---
*Figure 2:* Block diagram of the synchronous machine model in rotor coordinates. The magnetic model includes {eq}`sm_current` and {eq}`sm_torque`. Since the spatial harmonics are omitted, the current $\is$ and the torque $\tauM$ do not depend on the angle $\thetam$.
```

```{figure} ../figs/sm_block_stat.svg
---
name: fig:sm_block_stat
class: only-light
width: 100%
align: center
alt: Synchronous machine model seen from stator coordinates
---
*Figure 3:* Synchronous machine model seen from stator coordinates.
```

```{figure} ../figs/sm_block_stat.svg
---
class: invert-colors-dark only-dark
width: 100%
align: center
alt: Synchronous machine model seen from stator coordinates
---
*Figure 3:* Synchronous machine model seen from stator coordinates.
```

```{note}
Due to the spatial harmonics, the stator current $\is = \isfcn(\psis, \thetam)$ and the electromagnetic torque $\tauM = \tauM(\psis,\thetam)$ would also depend on the angle $\thetam$. This effect is typically omitted in dynamic models and not implemented here.
```

### Linear Magnetic Model

The linear magnetic model is parametrized using the {class}`motulator.drive.model.SynchronousMachinePars` class. In this case, the current is

```{math}
---
label: sm_current_linear
---
    \is = \frac{\psid - \psif}{\Ld} + \jj\frac{\psiq}{\Lq}
```

where $\Ld$ is the d-axis inductance, $\Lq$ is the q-axis inductance, and $\psif$ is the permanent-magnet (PM) flux linkage. The model represents a surface-PMSM with $\Ld = \Lq$ and SyRM with $\psif = 0$. The linear magnetic model {eq}`sm_current_linear` can be inverted, i.e., the flux linkage is $\psis = \psif + \Ld\id + \jj\Lq\iq$.

(sm_magnetic_saturation)=

### Saturation Models

Nonlinear magnetic models are parametrized using the {class}`motulator.drive.model.SaturatedSynchronousMachinePars` class. The term _magnetic model_ refers to the complete magnetic characterization of the machine. This characterization can be represented in two equivalent ways: as current maps or as flux linkage maps. These two representations are mathematically inverse to each other but describe the same physical relationship.

The magnetic model can be obtained through experimental measurements {cite}`Arm2013` or finite-element method analysis. Regardless of the source, these models can be implemented as the current map $\is = \isfcn(\psis)$ or as its inverse, the flux linkage map

```{math}
---
label: sm_flux_linkage_map
---
    \psis &= \psisfcn(\is) \\
    &= \psidfcn(\id, \iq) + \jj\psiqfcn(\id, \iq)

```

In practice, these maps are typically realized as lookup tables or explicit mathematical functions, see, e.g., {cite}`Hin2017,Lel2024`.

Methods for manipulating and plotting the magnetic models are provided in the classes {class}`motulator.drive.utils.MagneticModel`, {func}`motulator.drive.utils.plot_map`, and {func}`motulator.drive.utils.plot_flux_vs_current`. If the magnetic model is lossless and physically feasible, it can be numerically inverted between the two representations, see the {func}`motulator.drive.utils.MagneticModel.invert` method.

See also the examples {doc}`/drive_examples/flux_vector/plot_6kw_pmsyrm_sat_fvc`, {doc}`/drive_examples/flux_vector/plot_7kw_syrm_sat_fvc`, and {doc}`/drive_examples/current_vector/plot_5kw_pmsyrm_thor_sat_cvc`.

(mtpa_mtpv)=

## MTPV and MTPA Conditions

The maximum-torque-per-volt (MTPV) and maximum-torque-per-ampere (MTPA) conditions for saturable machines can be compactly presented by means of the auxiliary current and flux vectors {cite}`Var2022`. In addition to optimal reference generation, these vectors are useful in flux-vector control and sensorless observers {cite}`Tii2025a`. The {class}`motulator.drive.model.SaturatedSynchronousMachinePars` class contains methods for computing these vectors from the given magnetic model.

Consider the MTPV condition as an example. Applying the flux linkage vector $\psis = \abspsis \e^{\jj\delta}$ with a given magnitude $\abspsis$ in the torque expression {eq}`sm_torque`, the MTPV condition is obtained by setting $\partial \tauM/\partial \delta = 0$. This results in

```{math}
---
label: sm_mtpv
---
    \text{MTPV:} \quad \RE\left\{\iaux\psis^* \right\} = 0
```

where the auxiliary current vector is

```{math}
---
label: sm_mtpv_aux
---
    \iaux(\psis) = -\is + \Gqq \psid + \jj \Gdd \psiq - \jj \Gdq \psis^*
```

The current $\is$ as well as the incremental inverse inductances $\Gdd = \partial \id/\partial \psid$, $\Gqq = \partial \iq/\partial \psiq$, and $\Gdq = \partial \id/\partial \psiq$ are obtained from the current map $\isfcn(\psis)$.

The MTPA condition is obtained in a similar manner, resulting in

```{math}
---
label: sm_mtpa
---
    \text{MTPA:} \quad \RE\left\{\psiaux\is^* \right\} = 0
```

where the auxiliary flux vector is

```{math}
---
label: sm_mtpa_aux
---
    \psiaux(\is) = \psis - \Lqq \id - \jj \Ldd \iq + \jj \Ldq \is^*
```

The flux linkage $\psis$ as well as the incremental inductances $\Ldd = \partial \psid/\partial \id$, $\Lqq = \partial \psiq/\partial \iq$, and $\Ldq = \partial \psid/\partial \iq$ are obtained from the flux linkage map $\psisfcn(\is)$.

The conditions in {eq}`sm_mtpv` and {eq}`sm_mtpa` are realized in the class {class}`motulator.drive.utils.ControlLoci` that provides methods for computing the MTPA, MTPV, and constant current loci for magnetically linear as well as saturated machines. The class {class}`motulator.drive.utils.MachineCharacteristics` provides methods for visualizing these loci. See also the examples {doc}`/drive_examples/flux_vector/plot_6kw_pmsyrm_sat_fvc`, {doc}`/drive_examples/flux_vector/plot_7kw_syrm_sat_fvc`, and {doc}`/drive_examples/current_vector/plot_5kw_pmsyrm_thor_sat_cvc`.

```{note}
The auxiliary vectors follow {cite}`Hin2018` conventions and are rotated 90° relative to {cite}`Var2022`.
```

## Flux and Torque Dynamics

From the machine model {eq}`sm_states`--{eq}`sm_torque`, the stator flux and torque dynamics can be written as {cite}`Tii2025a`

```{math}
---
label: sm_flux_torque
---
    \frac{\D \abspsis}{\D t} &= \frac{1}{\abspsis} \RE \{\left(\us - \Rs \is - \jj\omegam\psis\right) \psis^* \} \\
    \frac{\D\tauM}{\D t} &= \frac{3 \np}{2} \IM \{ \left(\us - \Rs \is - \jj\omegam\psis \right) \iaux^*\}
```

where $\abspsis = |\psis|$ is the flux magnitude. These dynamics are valid also for saturated machines, when the auxiliary current $\iaux$ is defined according to {eq}`sm_mtpv_aux`.
