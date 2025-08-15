# Induction Machines

This document describes continuous-time induction machine models of the {mod}`motulator.drive.model` package.

(induction_machine)=

## Induction Machine Γ Model

[Figure 1](fig:im_gamma) shows the Γ-equivalent circuit model of an induction machine. We apply it as the base model, as it readily extends to include magnetic saturation {cite}`Sle1989`. The model is implemented in {class}`motulator.drive.model.InductionMachine` class.

```{figure} ../figs/im_gamma.svg
---
name: fig:im_gamma
class: only-light
width: 100%
align: center
alt: Gamma-model of an induction machine
---
*Figure 1:* Γ model of an induction machine in stator coordinates (denoted by the superscript s). The stator inductance can be parametrized to be a nonlinear function of the stator flux magnitude, $\Ls = \Ls(\abspsis)$.
```

```{figure} ../figs/im_gamma.svg
---
class: invert-colors-dark only-dark
width: 100%
align: center
alt: Gamma-model of an induction machine
---
*Figure 1:* Γ model of an induction machine in stator coordinates (denoted by the superscript s). The stator inductance can be parametrized to be a nonlinear function of the stator flux magnitude, $\Ls = \Ls(\abspsis)$.
```

In general coordinates rotating at $\omegac$ (see {ref}`coordinate-transformation`), the voltage equations are

```{math}
---
label: im_voltages
---
    \frac{\D\psis}{\D t} &= \us - \Rs\is - \jj\omegac\psis \\
    \frac{\D\psir}{\D t} &= -\Rr\ir - \jj(\omegac - \omegam)\psir
```

where $\us$ is the stator voltage, $\is$ is the stator current, $\psis$ is the stator flux linkage, and $\Rs$ is the stator resistance. The rotor quantities are defined similarly. The electrical angular speed of the rotor is $\omegam = \np \omegaM$, where $\omegaM$ is the mechanical angular speed of the rotor and $\np$ is the number of pole pairs.

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

The electromagnetic torque is

```{math}
---
label: im_torque
---
    \tauM = \frac{3\np}{2}\IM \left\{\is \psis^* \right\}
```

where the star operator denotes the complex conjugate. [Figure 2](fig:im_block) shows the block diagram of the model in stator coordinates.

```{figure} ../figs/im_block.svg
---
name: fig:im_block
class: only-light
width: 100%
align: center
alt: Block diagram of an induction machine model
---
*Figure 2:* Block diagram of the induction machine model. The magnetic model includes the flux equations (or, optionally, saturation characteristics) and the torque equation.
```

```{figure} ../figs/im_block.svg
---
class: invert-colors-dark only-dark
width: 100%
align: center
alt: Block diagram of an induction machine model
---
*Figure 2:* Block diagram of the machine model. The magnetic model includes the flux equations (or, optionally, saturation characteristics) and the torque equation.
```

The main-flux saturation is modeled using a nonlinear stator inductance $\Ls = \Ls(\abspsis)$ where $\abspsis = |\psis|$ {cite}`Sle1989, Qu2012`. The saturable stator inductance can be unambiguously characterized using simple no-load tests. It also properly takes into account the effect of the load variations on the saturation state. Notice that the nonlinear stator inductance only appears in {eq}`gamma_factor`.

The Γ model in stator coordinates ($\omegac = 0$) is implemented in the {class}`motulator.drive.model.InductionMachine` class. Its parameters are defined in the {class}`motulator.drive.model.InductionMachinePars` class, which accepts nonlinear stator inductance $\Ls = \Ls(\abspsis)$. See also examples {doc}`/drive_examples/current_vector/plot_2kw_im_sat_cvc` and {doc}`/drive_examples/flux_vector/plot_2kw_im_sat_fvc`.

## Induction Machine Inverse-Γ Model

### Linear Magnetic Model

[Figure 3](fig:im_inv_gamma) shows the inverse-Γ model of an induction machine. This model is commonly used in control applications for more straightforward implementation. Constant parameters are defined using {class}`motulator.drive.model.InductionMachineInvGammaPars` and can be used both in control methods and to parametrize the {class}`motulator.drive.model.InductionMachine` model.

```{figure} ../figs/im_inv_gamma.svg
---
name: fig:im_inv_gamma
class: only-light
width: 100%
align: center
alt: Inverse-Gamma model of an induction machine
---
*Figure 3:* Inverse-Γ model of an induction machine. If main-flux saturation is included, the parameters become nonlinear functions of the stator flux magnitude, $\Lsgm(\abspsis)$, $\RR(\abspsis)$, and $\LM(\abspsis)$.
```

```{figure} ../figs/im_inv_gamma.svg
---
class: invert-colors-dark only-dark
width: 100%
align: center
alt: Inverse-Gamma model of an induction machine
---
*Figure 3:* Inverse-Γ model of an induction machine. If main-flux saturation is included, the parameters become nonlinear functions of the stator flux magnitude, $\Lsgm(\abspsis)$, $\RR(\abspsis)$, and $\LM(\abspsis)$.
```

If the magnetic saturation is omitted, the inverse-Γ model is mathematically identical to the Γ model {cite}`Sle1989`. The parameters can be transformed as

```{math}
---
label: gamma_to_inv_gamma
---
    \Lsgm = \gamma L_\ell \qquad
    \LM = \gamma\Ls \qquad
    \RR = \gamma^2 \Rr
```

where $\gamma$ is defined in {eq}`gamma_factor`. Furthermore, the rotor flux linkage $\psiR = \gamma\psir$ and the rotor current $\iR = \ir/\gamma$ are also scaled. Otherwise the equations remain the same as in the Γ model.

### Inverse-Γ Model With Main-Flux Saturation

While the inverse-Γ model is convenient for control, incorporating magnetic saturation directly into it is challenging. A more practical approach is to map the saturation effects from the Γ model using the factor $\gamma$ in {eq}`gamma_factor`. As a result, the inverse-Γ model parameters become flux-dependent: $\Lsgm(\abspsis)$, $\RR(\abspsis)$, and $\LM(\abspsis)$. The {class}`motulator.drive.model.InductionMachinePars` class handles the conversion from the Γ parameters to the inverse-Γ parameters, enabling saturation-aware control methods (see example {doc}`/drive_examples/flux_vector/plot_2kw_im_sat_fvc`).

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

## Flux and Torque Dynamics

Using the machine model {eq}`im_voltages`--{eq}`im_torque`, the flux and torque dynamics can be written as {cite}`Tii2025b`

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
Substituting $\omegar = \omegarb$ in {eq}`im_flux_torque` yields the breakdown condition $\IM\{\psis\psir^*\} = \RE\{\psis\psir^*\}$. Applying this condition in {eq}`im_voltages`--{eq}`im_torque` and assuming the steady state results in the breakdown torque expression

\begin{equation}
    \tau_\mathrm{b} = \frac{3\np}{2} \frac{\abspsis^2}{2 L_\ell}
\end{equation}
```
