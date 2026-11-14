# Induction Machines

This document describes continuous-time induction machine models of the {mod}`motulator.drive.model` package.

(induction_machine)=

## Induction Machine Model

The induction machine model consists of the stator resistance, the magnetic model, and the rotor cage. In general coordinates rotating at $\omegac$ (see {ref}`coordinate-transformation`), the voltage equations are

```{math}
---
label: im_voltages
---
    \frac{\D\psis}{\D t} &= \us - \Rs\is - \jj\omegac\psis \\
    \frac{\D\psir}{\D t} &= \mathbf{u}_\mathrm{r} - \Rr\ir - \jj(\omegac - \omegam)\psir
```

where $\us$ is the stator voltage, $\is$ is the stator current, $\psis$ is the stator flux linkage, and $\Rs$ is the stator resistance. The rotor quantities are defined similarly. The rotor winding of a cage machine is short-circuited, i.e., the rotor voltage is $\mathbf{u}_\mathrm{r} = 0$. The deep-bar effect can be modeled by replacing the rotor resistance $\Rr$ with the cage impedance, see {ref}`im_cage_impedance`. The electrical angular speed of the rotor is $\omegam = \np \omegaM$, where $\omegaM$ is the mechanical angular speed of the rotor and $\np$ is the number of pole pairs.

The magnetic model gives the stator and rotor currents as functions of the stator and rotor flux linkages, see {ref}`im_magnetic_model`. Without spatial harmonics, the electromagnetic torque is

```{math}
---
label: im_torque
---
    \tauM = \frac{3\np}{2}\IM \left\{\is \psis^* \right\}
```

where the star operator denotes the complex conjugate.

The model is implemented in the {class}`motulator.drive.model.InductionMachine` class in rotor coordinates ($\omegac = \omegam$), in which the cage impedance is time invariant and the magnetic model can depend on the electrical rotor angle $\thetam$, whose state equation is $\D\thetam/\D t = \omegam$. [Figure 1](fig:im_block) shows the block diagram of the model in these coordinates. Since the machine is fed and observed from stator coordinates, the stator voltage and the stator current are transformed accordingly, in the same way as in the synchronous machine model, see [Figure 3](fig:sm_block_stat) in {ref}`synchronous_machine`.

```{figure} ../figs/im_block.svg
---
name: fig:im_block
class: only-light
width: 100%
align: center
alt: Block diagram of an induction machine model
---
*Figure 1:* Block diagram of the induction machine model in rotor coordinates. The magnetic model gives the currents and the torque, and it depends on the angle $\thetam$ only if spatial harmonics are modeled. The cage impedance $Z_\mathrm{c}(s)$ reduces to the rotor resistance $\Rr$, if the deep-bar effect is omitted.
```

```{figure} ../figs/im_block.svg
---
class: invert-colors-dark only-dark
width: 100%
align: center
alt: Block diagram of an induction machine model
---
*Figure 1:* Block diagram of the induction machine model in rotor coordinates. The magnetic model gives the currents and the torque, and it depends on the angle $\thetam$ only if spatial harmonics are modeled. The cage impedance $Z_\mathrm{c}(s)$ reduces to the rotor resistance $\Rr$, if the deep-bar effect is omitted.
```

The rotor quantities can be scaled freely, since the rotor of a cage machine has no terminals. The class uses the scaling of its parameters, e.g., the rotor quantities are those of the inverse-Γ model if the inverse-Γ parameters are used.

(im_magnetic_model)=

## Magnetic Model

### Γ Model

[Figure 2](fig:im_gamma) shows the Γ-equivalent circuit model of an induction machine, which readily extends to include the main-flux saturation {cite}`Sle1989`. Its parameters are defined in the {class}`motulator.drive.model.InductionMachinePars` class.

```{figure} ../figs/im_gamma.svg
---
name: fig:im_gamma
class: only-light
width: 100%
align: center
alt: Gamma-model of an induction machine
---
*Figure 2:* Γ model of an induction machine in stator coordinates (denoted by the superscript s). The stator inductance can be parametrized to be a nonlinear function of the stator flux magnitude, $\Ls = \Ls(\abspsis)$.
```

```{figure} ../figs/im_gamma.svg
---
class: invert-colors-dark only-dark
width: 100%
align: center
alt: Gamma-model of an induction machine
---
*Figure 2:* Γ model of an induction machine in stator coordinates (denoted by the superscript s). The stator inductance can be parametrized to be a nonlinear function of the stator flux magnitude, $\Ls = \Ls(\abspsis)$.
```

The stator and rotor currents, respectively, are

```{math}
---
label: im_currents
---
    \is &= \frac{\psis - \gamma\psir}{\gamma L_\ell} \\
    \ir &= \frac{\psir - \psis}{L_\ell}
```

where $\Ls$ is the stator inductance, $L_\ell$ is the leakage inductance, and $\gamma$ is the magnetic coupling factor

```{math}
---
label: gamma_factor
---
    \gamma = \frac{\Ls}{\Ls + L_\ell}
```

The main-flux saturation is modeled using a nonlinear stator inductance $\Ls = \Ls(\abspsis)$ where $\abspsis = |\psis|$ {cite}`Sle1989, Qu2012`. The saturable stator inductance can be unambiguously characterized using simple no-load tests. It also properly takes into account the effect of the load variations on the saturation state. Notice that the nonlinear stator inductance only appears in {eq}`gamma_factor`. See also examples {doc}`/drive_examples/current_vector/plot_2kw_im_sat_cvc` and {doc}`/drive_examples/flux_vector/plot_2kw_im_sat_fvc`.

### Inverse-Γ Model

[Figure 3](fig:im_inv_gamma) shows the inverse-Γ model of an induction machine. This model is commonly used in control applications for more straightforward implementation. Its constant parameters are defined in the {class}`motulator.drive.model.InductionMachineInvGammaPars` class, which can be used both in control methods and to parametrize the {class}`motulator.drive.model.InductionMachine` model.

```{figure} ../figs/im_inv_gamma.svg
---
name: fig:im_inv_gamma
class: only-light
width: 100%
align: center
alt: Inverse-Gamma model of an induction machine
---
*Figure 3:* Inverse-Γ model of an induction machine in stator coordinates. If main-flux saturation is included, the parameters become nonlinear functions of the stator flux magnitude, $\Lsgm(\abspsis)$, $\RR(\abspsis)$, and $\LM(\abspsis)$.
```

```{figure} ../figs/im_inv_gamma.svg
---
class: invert-colors-dark only-dark
width: 100%
align: center
alt: Inverse-Gamma model of an induction machine
---
*Figure 3:* Inverse-Γ model of an induction machine in stator coordinates. If main-flux saturation is included, the parameters become nonlinear functions of the stator flux magnitude, $\Lsgm(\abspsis)$, $\RR(\abspsis)$, and $\LM(\abspsis)$.
```

The stator and rotor flux linkages, respectively, are

```{math}
---
label: im_inv_gamma_fluxes
---
    \psis &= \Lsgm\is + \psiR \\
    \psiR &= \LM(\is + \iR)
```

where $\Lsgm$ is the leakage inductance and $\LM$ is the magnetizing inductance. The rotor flux linkage $\psiR = \gamma\psir$ and the rotor current $\iR = \ir/\gamma$ are scaled with respect to the Γ model, and the rotor resistance is $\RR$. If the magnetic saturation is omitted, the inverse-Γ model is mathematically identical to the Γ model {cite}`Sle1989`. The parameters can be transformed as

```{math}
---
label: gamma_to_inv_gamma
---
    \Lsgm = \gamma L_\ell \qquad
    \LM = \gamma\Ls \qquad
    \RR = \gamma^2 \Rr
```

where $\gamma$ is defined in {eq}`gamma_factor`.

While the inverse-Γ model is convenient for control, incorporating magnetic saturation directly into it is challenging. A more practical approach is to map the saturation effects from the Γ model using the factor $\gamma$ in {eq}`gamma_factor`. As a result, the inverse-Γ model parameters become flux-dependent: $\Lsgm(\abspsis)$, $\RR(\abspsis)$, and $\LM(\abspsis)$. The {class}`motulator.drive.model.InductionMachinePars` class provides these inverse-Γ parameters, enabling saturation-aware control methods (see example {doc}`/drive_examples/flux_vector/plot_2kw_im_sat_fvc`).

Using the stator flux linkage and the stator current as state variables, the nonlinear state-space form becomes

```{math}
---
label: im_inv_gamma_ss
---
    \frac{\D \psis}{\D t} &= \us - \Rs\is - \jj\omegac\psis \\
    \Lsgm\frac{\D \is}{\D t} &= \us - (\Rs + \RR + \jj\omegac \Lsgm)\is
    + \left(\alpha - \jj\omegam\right)\psiR - \boldsymbol{\upepsilon} \\
    \psiR &= \psis - \Lsgm\is
```

where $\Lsgm$, $\RR$, $\LM$, $\alpha = \RR/\LM$ are nonlinear functions of the stator flux magnitude. Saturation introduces a transient voltage term

```{math}
---
label: im_inv_gamma_eps
---
    \boldsymbol{\upepsilon} = \frac{1}{2}\frac{\abspsis}{\gamma(\abspsis)}\frac{\partial\gamma(\abspsis)}{\partial\abspsis} \left[ \us - \Rs\is + \frac{\psis}{\psis^*}(\us^* - \Rs\is^*)\right]
```

This voltage term only appears during changes in the stator-flux magnitude. When the transient voltage term is included, the inverse-Γ model {eq}`im_inv_gamma_ss` is mathematically identical to the original nonlinear Γ model. However, in control methods, the transient voltage term can typically be neglected.

### General Magnetic Model

In general, the magnetic model is a current map, i.e., a map from the flux linkages to the currents. The map is lossless, if the currents are the gradient of the field energy with respect to the flux linkages, i.e.,

```{math}
---
label: im_energy
---
    \is = \frac{\partial W}{\partial \psis} \qquad
    \ir = \frac{\partial W}{\partial \psir}
```

where $W(\psis, \psir)$ is the field energy multiplied by 2/3 and the derivatives are taken with respect to the real and imaginary parts of the flux linkages. The Jacobian of the map is then symmetric, which is known as the reciprocity condition. Without spatial harmonics, $W$ is invariant under a common rotation of the flux linkages, which makes the torque {eq}`im_torque` equal to the torque $-(3\np/2)\IM\{\ir\psir^*\}$ computed from the rotor quantities. As an example, the Γ model with constant inductances corresponds to

```{math}
---
label: im_energy_gamma
---
    W = \frac{|\psis|^2}{2\Ls} + \frac{|\psir - \psis|^2}{2L_\ell}
```

A current map can be given as a function using the {class}`motulator.drive.model.SaturatedInductionMachinePars` class, e.g., for modeling the saturation of the leakage flux or the mutual saturation between the main flux and the leakage flux {cite}`Tuo2010`. With spatial harmonics (e.g., due to the slots), the field energy $W(\psis, \psir, \thetam)$ in rotor coordinates also depends on the electrical rotor angle $\thetam$, and the torque is

```{math}
---
label: im_torque_harmonics
---
    \tauM = \frac{3\np}{2}\left(\IM \left\{\is \psis^* \right\} - \frac{\partial W}{\partial \thetam}\right)
```

Such a map, including the torque, can be given using the {class}`motulator.drive.model.SpatialSaturatedInductionMachinePars` class.

(im_cage_impedance)=

## Cage Impedance

Due to the eddy currents in the rotor bars, the rotor resistance increases and the rotor leakage inductance decreases with the frequency of the rotor current (deep-bar effect). This effect can be modeled by replacing the rotor resistance with the cage impedance {cite}`Mol2020`

```{math}
---
label: im_cage
---
    Z_\mathrm{c}(s) = R_0 + \cfrac{1}{\cfrac{1}{sL_0} + \cfrac{1}{R_1 + \cfrac{1}{\cfrac{1}{sL_1} + \cdots}}}
```

in rotor coordinates, i.e., a ladder circuit of series resistances and shunt inductances, terminated with the last resistance, as shown in [Figure 4](fig:im_cage). Hence, $R_0$ is the DC resistance and $L_0$ is the DC inductance of the cage. The rotor voltage equation in {eq}`im_voltages` then becomes

```{math}
---
label: im_cage_voltage
---
    \frac{\D\psir}{\D t} = \mathbf{u}_\mathrm{r} - Z_\mathrm{c}(s)\ir
```

in rotor coordinates ($\omegac = \omegam$), where $s$ is the differential operator. The flux linkages of the inductances are additional state variables.

```{figure} ../figs/im_cage.svg
---
name: fig:im_cage
class: only-light
width: 100%
align: center
alt: Cage impedance as a ladder circuit
---
*Figure 4:* Cage impedance $Z_\mathrm{c}(s)$ as a ladder circuit of the order $N$.
```

```{figure} ../figs/im_cage.svg
---
class: invert-colors-dark only-dark
width: 100%
align: center
alt: Cage impedance as a ladder circuit
---
*Figure 4:* Cage impedance $Z_\mathrm{c}(s)$ as a ladder circuit of the order $N$.
```

For rectangular rotor bars, the elements are {cite}`Mol2020`

```{math}
---
label: im_cage_elements
---
    R_n = (4n + 1)R_0 \qquad
    L_n = \frac{3L_0}{4n + 3}
```

The cage impedance is defined in the {class}`motulator.drive.model.CageImpedance` class, which can be given as the rotor resistance to the Γ-model parameters and to the general magnetic model classes. The inverse-Γ parameters of the Γ-model class, used in control, are then those of the low-frequency equivalent $Z_\mathrm{c}(s) \approx R_0 + sL_0$, i.e., the DC inductance of the cage adds to the leakage inductance. See also the example {doc}`/drive_examples/grid_connection/plot_6kw_im_deep_bars_dol`.

(im_core_losses)=

## Core Losses

Eddy-current core losses can be optionally modeled by means of a constant core-loss conductance $\Gc$, connected between the stator resistance and the magnetic model, i.e., in parallel with the stator inductance in the Γ model {cite}`Qu2012`. In this case, the current given by the magnetic model is denoted by $\isp$ and the stator current becomes

```{math}
---
label: im_core_loss
---
    \is = \isp + \Gc(\us - \Rs\is) = \frac{\isp + \Gc\us}{1 + \Gc\Rs}
```

where $\us - \Rs\is$ is the voltage across the core-loss conductance. The corresponding core losses are $(3/2)\Gc|\us - \Rs\is|^2$. Notice that the torque is produced by the current $\isp$, i.e., the core-loss current does not contribute to the torque. By default, $\Gc = 0$, i.e., the core losses are omitted.

```{note}
For notational simplicity, $\Gc = 0$ is assumed elsewhere in this documentation, i.e., $\is = \isp$. Otherwise, the magnetic model and the expressions derived from it hold as such, if the current $\isp$ is used in place of $\is$.
```

## Flux and Torque Dynamics

Using the Γ model with the rotor resistance, the flux and torque dynamics can be written as {cite}`Tii2025b`

```{math}
---
label: im_flux_torque
---
    \frac{\D \abspsis}{\D t} &= \frac{1}{\abspsis} \RE \{\left[\us - \Rs \is - \jj(\omegam + \omegar)\psis\right] \psis^*\} \\
    \frac{\D\tauM}{\D t} &= \frac{3 \np}{2 L_\ell} \IM \{\left[\us - \Rs \is - \jj(\omegam + \omegar)\psis\right] \psir^*\} \\
    \omegar &= \frac{\omegarb \IM\{\psis\psir^*\}}{\RE\{\psis\psir^*\}}

```

where $\abspsis = |\psis|$ is the flux magnitude, $\omegarb = \Rr/L_\ell$ is the breakdown angular slip frequency, and $\omegar$ is the slip angular frequency. These dynamics are valid also for saturated machines where $\Ls = \Ls(\abspsis)$, and they are also independent of the coordinate system. In this documentation, the sum of the electrical angular speed and the slip angular frequency

```{math}
---
label: im_omegas
---
    \omegas = \omegam + \omegar

```

is referred to as the stator angular frequency, with this definition being agnostic to the coordinate system and defined also in transient states.

The flux and torque dynamics can be easily transformed to the inverse-Γ form by substituting $\psir = \psiR/\gamma$, $L_\ell = \Lsgm/\gamma$, and $\Rr = \RR/\gamma^2$. Compare also to {eq}`sm_flux_torque` in the {ref}`synchronous_machine` document.

```{note}
Substituting $\omegar = \omegarb$ in {eq}`im_flux_torque` yields the breakdown condition $\IM\{\psis\psir^*\} = \RE\{\psis\psir^*\}$. Applying this condition in the Γ model and assuming the steady state results in the breakdown torque expression

\begin{equation}
    \tau_\mathrm{b} = \frac{3\np}{2} \frac{\abspsis^2}{2 L_\ell}
\end{equation}
```
