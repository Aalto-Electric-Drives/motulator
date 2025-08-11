# Flux-Vector Control

Flux-vector control directly controls the stator flux magnitude and the electromagnetic torque, which simplifies reference generation especially in the field-weakening region {cite}`Pel2009`. We use the control law that results from feedback linearization of the flux and torque dynamics {cite}`Awa2019b`.

## For Induction Machines

### Flux and Torque Dynamics

Using the machine model \[see {eq}`im_flux_torque` in {ref}`induction_machine`\], the flux and torque dynamics can be expressed as

```{math}
---
label: im_flux_torque_fvc
---
    \frac{\D \abspsis}{\D t} &= \frac{1}{\abspsis} \RE \{\left[\us - \Rs \is - \jj\omegas\psis\right] \psis^*\} \\
    \frac{\D\tauM}{\D t} &= \frac{3 \np}{2 L_\sigma} \IM \{\left[\us - \Rs \is - \jj\omegas\psis\right] \psiR^*\}

```

where $\abspsis = |\psis|$ is the flux magnitude. The stator angular frequency is defined by $\omegas = \omegam + \omegar$, where $\omegam$ is the rotor angular speed. The slip angular frequency is

```{math}
---
label: omegar_fvc
---
    \omegar = \frac{\omegarb \IM\{\psis\psiR^*\}}{\RE\{\psis\psiR^*\}}

```

where $\omegarb = \Rr/L_\ell = [(\LM + \Lsgm)/(\LM\Lsgm)]\RR$ is the breakdown angular slip frequency. Note that the above expressions are independent of the coordinate system. Furthermore, they hold also for saturated machines, where $\Ls = \Ls(\abspsis)$.

### Proportional Control Law

Based on {eq}`im_flux_torque_fvc`, the proportional control law can be derived {cite}`Tii2025a`

```{math}
---
label: im_flux_vector_control_usref
---
    \usref = \hatRs \is + \jj\hatomegas \hatpsis + e_\uppsi \mathbf{t}_\uppsi + e_\uptau \mathbf{t}_\uptau
```

where $e_\uppsi = \alpha_\uppsi(\abspsisref - \hatabspsis)$ and $e_\uptau = \alpha_\uptau(\tauMref - \hattauM)$ are the bandwidth-scaled control errors of the stator flux linkage and torque, respectively, and the estimates are marked with the hat. The complex quantities $\mathbf{t}_\uppsi$ and $\mathbf{t}_\uptau$ are the direction vectors:

```{math}
---
label: im_flux_vector_control_dir
---
    \mathbf{t}_\uppsi = \frac{ \hatabspsis}{ \RE\{\hatpsiR\hatpsis^*\}}\hatpsiR \qquad
    \mathbf{t}_\uptau = \frac{2\Lsgm}{3 \np \RE\{\hatpsiR\hatpsis^*\}}\jj\hatpsis
```

Note that $\RE\{\hatpsiR\hatpsis^*\} > 0$ holds in the whole feasible operating region.

### Closed-Loop System Analysis

Assuming accurate estimates, linearization of the closed-loop system consisting of {eq}`im_flux_torque_fvc`--{eq}`im_flux_vector_control_dir` results in

```{math}
---
label: im_flux_vector_control_closed_lin
---
    \frac{\D\Delta\abspsis}{\D t} = \alpha_\uppsi (\Delta \abspsisref - \Delta \abspsis) \qquad
    \frac{\D\Delta\tauM}{\D t} = \alpha_\uptau (\Delta\tauMref - \Delta \tauM)
```

These closed-loop small-signal dynamics are valid also for saturated machines, if the flux observer takes the magnetic saturation into account. Furthermore, the dynamics hold also for sensorless drives, if the decoupling observer gains are used, see the {ref}`im_obs_sensorless` document and {cite}`Tii2025b`.

## For Synchronous Machines

The flux-vector control for synchronous machines is analogous to the induction machine case. Only the differences are highlighted here.

### Flux and Torque Dynamics

Using the machine model \[see {eq}`sm_mtpv_aux` and {eq}`sm_flux_torque` in {ref}`synchronous_machine`\], the flux and torque dynamics can be expressed as

```{math}
---
label: flux_torque
---
    \frac{\D \abspsis}{\D t} &= \frac{1}{\abspsis} \RE \{\left(\us - \Rs \is - \jj\omegam\psis\right) \psis^* \} \\
    \frac{\D\tauM}{\D t} &= \frac{3 \np}{2} \IM \{ \left(\us - \Rs \is - \jj\omegam\psis \right) \iaux^*\}
```

These expressions are valid also for saturated machines.

### Proportional Control Law

The proportional control law is {cite}`Tii2025a`

```{math}
---
label: sm_flux_vector_control_usref
---
    \usref = \hatRs \is + \jj\hatomegam \hatpsis + e_\uppsi \mathbf{t}_\uppsi + e_\uptau \mathbf{t}_\uptau
```

where $e_\uppsi = \alpha_\uppsi(\abspsisref - \hatabspsis)$ and $e_\uptau = \alpha_\uptau(\tauMref - \hattauM)$, and the estimates are marked with the hat. The complex direction vectors are

```{math}
---
label: sm_flux_vector_control_dir
---
    \mathbf{t}_\uppsi = \frac{ \hatabspsis}{ \RE\{\hatiaux\hatpsis^*\}}\hatiaux \qquad
    \mathbf{t}_\uptau = \frac{2}{3 \np \RE\{\hatiaux\hatpsis^*\}}\jj\hatpsis
```

Note that $\RE\{\hatiaux\hatpsis^*\} > 0$ holds in the whole feasible operating region. The closed-loop small-signal dynamics in {eq}`im_flux_vector_control_closed_lin` are valid also for synchronous machines (including saturated machines, if the flux observer takes the magnetic saturation into account). Furthermore, the dynamics hold also for sensorless drives, if the decoupling observer gains are used, see the {ref}`sm_obs_sensorless` document and {cite}`Tii2025a`.

## Inclusion of Integral Action

Optionally, the above control laws {eq}`im_flux_vector_control_usref` and {eq}`im_flux_vector_control_dir` as well as {eq}`sm_flux_vector_control_usref` and {eq}`sm_flux_vector_control_dir` can be extended with integral action. 2DOF PI control is achieved using the following integral states and control error terms,

```{math}
---
label: flux_vector_control_2dof
---
    \frac{\D x_\uppsi}{\D t} &= \alpha_\uppsi\alphai(\abspsisref - \hatabspsis) \\
    \frac{\D x_\uptau}{\D t} &= \alpha_\uptau\alphai(\tauMref - \hattauM) \\
    e_\uppsi &= \alpha_\uppsi(\abspsisref - \hatabspsis) + x_\uppsi - \alphai \hatabspsis \\
    e_\uptau &= \alpha_\uptau(\tauMref - \hattauM) + x_\uptau - \alphai \hattauM
```

where $x_\uppsi$ and $x_\uptau$ are the integral states, $\alpha_\uppsi$ and $\alpha_\uptau$ are the reference-tracking bandwidths, and $\alphai$ is the integral action bandwidth. It can be shown that the resulting linearized closed-loop reference-following dynamics remain the same as with the proportional control law. However, in this case, both the flux and torque dynamics have additional pole at $s = -\alphai$ resulting from the integral action. The integral action pole is canceled from reference-tracking dynamics (see the {ref}`2dof-pi-controller` document).

This 2DOF PI control law is a variant of {cite}`Awa2019b`, having channel-specific reference-tracking bandwidths. It has been implemented in the {class}`motulator.drive.control.im.FluxVectorController` and {class}`motulator.drive.control.sm.FluxVectorController` classes for induction and synchronous machines, respectively. The disturbance observer structure is used in the implementation to avoid the need for anti-windup mechanism (see the {ref}`disturbance-observer-structure` document).
