# Flux-Vector Control

Flux-vector control directly controls the stator flux magnitude and the electromagnetic torque, which simplifies reference generation especially in the field-weakening region {cite}`Pel2009`. We use the control law that results from feedback linearization of the flux and torque dynamics {cite}`Awa2019b`. Here, the inclusion of integral action is briefly discussed, as an extension to {cite}`Tii2025a`, {cite}`Tii2025b`. Furthermore, these notes aim to provide a link between the column-vector and complex-vector formulations of the control law. Here, a synchronous machine is considered as an example (see the {ref}`synchronous_machine` document for the model and notation), but the approach is analogous for induction machines.

## Flux and Torque Dynamics

From the machine model, the flux and torque dynamics can be written as

```{math}
---
label: flux_torque
---
    \frac{\D \abspsis}{\D t} &= \frac{1}{\abspsis} \RE \{\left(\us - \Rs \is - \jj\omegam\psis\right) \psis^* \} \\
    \frac{\D\tauM}{\D t} &= \frac{3 \np}{2} \IM \{ \left(\us - \Rs \is - \jj\omegam\psis \right) \iaux^*\}
```

where $\abspsis = |\psis|$. These expressions are valid also for saturated machines, when the auxiliary current $\iaux$ is defined according to {eq}`sm_mtpv_aux` in the {ref}`synchronous_machine` document.

## Proportional Control

Consider proportional control, resulting from the feedback linearization, {cite}`Tii2025a`

```{math}
---
label: flux_vector_control_usref
---
    \usref = \hatRs \is + \jj\omegam \hatpsis + e_\uppsi \mathbf{t}_\uppsi + e_\uptau \mathbf{t}_\uptau
```

where $\hatRs$ is the stator resistance estimate, $\hatpsis$ is the estimated flux vector, and $e_\uppsi = \alpha_\uppsi(\abspsisref - \hatabspsis)$ and $e_\uptau = \alpha_\uptau(\tauMref - \hattauM)$ are the control errors in the flux and torque, respectively, scaled by the respective bandwidths. The complex quantities $\mathbf{t}_\uppsi$ and $\mathbf{t}_\uptau$ are the direction vectors:

```{math}
---
label: flux_vector_control_dir
---
    \mathbf{t}_\uppsi = \frac{ \hatabspsis}{ \RE\{\hatiaux\hatpsis^*\}}\hatiaux \qquad
    \mathbf{t}_\uptau = \frac{2}{3 \np \RE\{\hatiaux\hatpsis^*\}}\jj\hatpsis
```

Notice that $\RE\{\hatiaux\hatpsis^*\} > 0$ in the whole feasible operating region.

Assuming accurate estimates, linearization of the closed-loop system consisting of {eq}`flux_torque`--{eq}`flux_vector_control_dir` results in

```{math}
---
label: flux_vector_control_closed_lin
---
    \frac{\D\Delta\abspsis}{\D t} = \alpha_\uppsi (\Delta \abspsisref - \Delta \abspsis) \qquad
    \frac{\D\Delta\tauM}{\D t} = \alpha_\uptau (\Delta\tauMref - \Delta \tauM)
```

These closed-loop small-signal dynamics are valid also for saturated machines, if the auxiliary current is estimated based on {eq}`sm_mtpv_aux`.

## Inclusion of Integral Action

Consider a 2DOF PI control law consisting of {eq}`flux_vector_control_usref` and {eq}`flux_vector_control_dir` and the following integral states and control error terms,

```{math}
---
label: flux_vector_control_2dof
---
    \frac{\D x_\uppsi}{\D t} &= \alpha_\uppsi\alphai(\abspsisref - \hatabspsis) \\
    \frac{\D x_\uptau}{\D t} &= \alpha_\uptau\alphai(\tauMref - \hattauM) \\
    e_\uppsi &= \alpha_\uppsi(\abspsisref - \hatabspsis) + x_\uppsi - \alphai \hatabspsis \\
    e_\uptau &= \alpha_\uptau(\tauMref - \hattauM) + x_\uptau - \alphai \hattauM
```

where $\alpha_\uppsi$ and $\alpha_\uptau$ are the reference-tracking bandwidths, $x_\uppsi$ and $x_\uptau$ are the integral states, and $\alphai$ is the integral action bandwidth. It can be shown that the resulting linearized closed-loop reference-following dynamics remain the same as {eq}`flux_vector_control_closed_lin`. However, in this case, both the flux and torque dynamics have additional pole at $s = - \alphai$ resulting from the integral action. The integral action pole is canceled from reference-tracking dynamics (see the {ref}`2dof-pi-controller` document).

This control law is a variant of {cite}`Awa2019b`, having channel-specific reference-tracking bandwidths. It has been implemented in the {class}`motulator.drive.control.sm.FluxVectorController` and {class}`motulator.drive.control.im.FluxVectorController` classes for synchronous and induction machines, respectively. The disturbance observer structure is used in the implementation to avoid the need for anti-windup mechanism (see the {ref}`disturbance-observer-structure` document).
