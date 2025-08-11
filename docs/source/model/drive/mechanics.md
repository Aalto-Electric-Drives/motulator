# Mechanics

## Stiff Mechanical System

The model of a stiff mechanical system is provided in the class {class}`motulator.drive.model.MechanicalSystem`. The dynamics are governed by

```{math}
---
label: mech_stiff
---
J\frac{\D\omegaM}{\D t} = \tauM - \tauLtot
```

where $\omegaM$ is the mechanical angular speed of the rotor, $\tauM$ is the electromagnetic torque, and $J$ is the total moment of inertia. The total load torque is

```{math}
---
label: total_load_torque
---
\tauLtot = \BL\omegaM + \tauL
```

where $\tauL = \tauL(t)$ is an external load torque as a function of time and $\BL$ is the friction coefficient. [Figure 1](fig:mech_block) shows the corresponding block diagram.

A constant friction coefficient $\BL$ models viscous friction that appears, e.g., due to laminar fluid flow in bearings. The friction coefficient is allowed to depend on the rotor speed, $\BL = \BL(\omegaM)$. As an example, the quadratic load torque profile is achieved choosing $\BL = k|\omegaM|$, where $k$ is a constant. The quadratic load torque appears, e.g., in pumps and fans as well as in vehicles moving at higher speeds due to air resistance.

The mechanical angle $\thetaM$ of the rotor is related to the mechanical angular speed as

```{math}
---
label: mech_angle
---
    \frac{\D\thetaM}{\D t} = \omegaM
```

```{figure} ../figs/mech_block.svg
---
name: fig:mech_block
class: only-light
width: 100%
align: center
alt: Block diagram of a stiff mechanical system.
---
*Figure 1:* Block diagram of a stiff mechanical system.
```

```{figure} ../figs/mech_block.svg
---
class: invert-colors-dark only-dark
width: 100%
align: center
alt: Block diagram of a stiff mechanical system.
---
*Figure 1:* Block diagram of a stiff mechanical system.
```

## Two-Mass Mechanical System

A two-mass mechanical system is modeled in the {class}`motulator.drive.model.TwoMassMechanicalSystem` class. The dynamics are governed by {cite}`Saa2015`

```{math}
---
label: mech_two_mass
---
    \JM\frac{\D\omegaM}{\D t} &= \tauM - \tauS \\
    \JL\frac{\D\omegaL}{\D t} &= \tauS - \tauL \\
    \frac{\D\thetaML}{\D t} &= \omegaM - \omegaL
```

where $\omegaL$ is the angular speed of the load, $\thetaML = \thetaM - \thetaL$ is the twist angle, $\JM$ is the moment of inertia of the machine, and $\JL$ is the moment of inertia of the load. The shaft torque is

```{math}
---
label: shaft_torque
---
    \tauS = \KS\thetaML + \CS(\omegaM - \omegaL)
```

where $\KS$ is the torsional stiffness of the shaft, and $\CS$ is the torsional damping of the shaft. [Figure 2](fig:two_mass_block) shows the block diagram of the system.

See the {doc}`/drive_examples/vhz/plot_2kw_ipmsm_2mass_ovhz` example.

```{figure} ../figs/two_mass_block.svg
---
name: fig:two_mass_block
class: only-light
width: 100%
align: center
alt: Block diagram of a two-mass mechanical system.
---
*Figure 2:* Block diagram of a two-mass mechanical system.
```

```{figure} ../figs/two_mass_block.svg
---
class: invert-colors-dark only-dark
width: 100%
align: center
alt: Block diagram of a two-mass mechanical system.
---
*Figure 2:* Block diagram of a two-mass mechanical system.
```

## Externally Specified Rotor Speed

It is also possible to omit the mechanical dynamics and directly specify the actual rotor speed $\omegaM$ as a function of time, see the class {class}`motulator.drive.model.ExternalRotorSpeed`. This feature is typically needed when torque-control mode is studied.

See the example {doc}`/drive_examples/current_vector/plot_2kw_im_cvc_tq`.
