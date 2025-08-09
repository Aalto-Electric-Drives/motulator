# Observers: Synchronous Machines

Observers estimate the internal state of synchronous machines. For drives with a motion sensor, the main goal is estimating the flux linkage and the torque. In sensorless drives, the rotor speed and position are also estimated {cite}`Jon1989,Cap2001,Pii2008,Hin2018`.

This document describes an observer design implemented in the {class}`motulator.drive.control.sm.FluxObserver` and {class}`motulator.drive.control.sm.SpeedFluxObserver` classes, based on {cite}`Hin2018`. The observer supports both sensorless and sensored operation modes and accounts for magnetic saturation.

## Machine Model

The synchronous machine model in rotor coordinates rotating at $\omegam$ is

```{math}
---
label: sm_model_obs
---
    \frac{\D\psis}{\D t} &= \us - \Rs\is - \jj\omegam\psis \\
    \frac{\D\thetam}{\D t} &= \omegam \\
    \is &= \isfcn(\psis) \\
    \tauM &= \frac{3 \np}{2}\IM\left\{\is\psis^*\right\}
```

where $\isfcn$ is the current map (see {ref}`synchronous_machine`). If the magnetic saturation is omitted, the current is $\is = (\psid - \psif)/\Ld + \jj\psiq/\Lq$.

## Coordinate Transformation

The control system operates in estimated rotor coordinates, aligned at the rotor angle estimate $\hatthetam$. In these coordinates, the measured current and the realized voltage (obtained from the PWM algorithm), respectively, are

```{math}
---
label: is_prime
---
    \is' = \iss \e^{-\jj\hatthetam} \qquad
    \us' = \us \e^{-\jj\hatthetam}
```

Due to the estimation error $\tildethetam = \thetam - \hatthetam$, the current $\is'$ generally differs from the current $\is$ (and similarly for the voltage).

## Observer Structure

Based on {eq}`sm_model_obs`, a nonlinear state observer is formulated as

```{math}
---
label: sm_obs
---
    \frac{\D \hatpsis}{\D t} &= \us' - \hatRs\is' - \jj\omegac\hatpsis + \koa \eo + \kob \eo^* \\
    \frac{\D\hatthetam}{\D t} &= \hatomegam - \kotheta \IM\left\{ \frac{\eo}{\psiaux} \right\} = \omegac
```

where $\omegac$ is the angular speed of the coordinate system, $\eo$ is the estimation error, $\koa$, $\kob$, and $\kotheta$ are observer gains, the estimates are marked with the hat, and $^*$ marks the complex conjugate. The flux estimation error is

```{math}
---
label: sm_eo
---
    \eo = \hatpsisfcn(\is') - \hatpsis
```

where $\hatpsisfcn$ is the flux map estimate. This observer structure is used in the {class}`motulator.drive.control.sm.FluxObserver` class.

```{note}
Since the current is measured, the observer is fundamentally corrected by means of the current estimation error. However, due to the saliency and magnetic saturation, the current estimation error is convenient to map (or scale in the case of linear magnetics) to the flux linkage error.
```

```{note}
Real-valued column vectors and the corresponding $2\times 2$ gain matrix were used in {cite}`Hin2018`. The complex form in {eq}`sm_obs` has the same degrees of freedom.
```

## Gain Selection

### Sensored Drives

In sensored case, $\tildethetam = 0$ holds and $\kob = 0$ is used. Therefore, using {eq}`sm_model_obs` and {eq}`sm_obs`, the linearized estimation-error dynamics become

```{math}
---
label: sm_tilde_psis_sensored
---
    \frac{\D \Delta\tildepsis}{\D t} = -(\koa + \jj\omegamo)\Delta\tildepsis
```

where $\Delta$ marks the small-signal quantities, the subscript 0 marks the operating-point quantities, and $\tildepsis = \psis - \hatpsis$ is the estimation error. The pole can be arbitrarily placed via the gain $\koa$. Well-damped dynamics are obtained simply with a real gain, $\koa = \sigma$, resulting in the pole at $s = -\sigma - \jj\omegamo$, where $\sigma = 2\pi \cdot 15$ rad/s is used as the default value in the {class}`motulator.drive.control.sm.FluxObserver` class in sensored drives.

#### Sensorless Drives

The analysis of the sensorless case resembles that of induction machines, see {ref}`im_obs_sensorless` in {doc}`im_observer`. The following results can be derived from the linearized form of {eq}`sm_model_obs` -- {eq}`sm_obs`, see details in {cite}`Hin2018`.

To decouple the flux estimation from the rotor angle, the gains of {eq}`sm_obs` have to be of the form

```{math}
---
label: k1k2_sensorless
---
    \koa = \sigma \qquad
    \kob = \frac{\sigma\hatpsiaux}{\hatpsiaux^*}
```

where $\hatpsiaux = \hatpsiaux(\is')$ is the estimate of the auxiliary flux \[see {eq}`sm_mtpa_aux` in {ref}`synchronous_machine`\] and $\sigma$ is the attenuation, i.e., the resulting characteristic polynomial is $D(s) = s^2 + 2\sigma s + \omegamo^2$. By default, the attenuation in sensorless drives is scheduled as

```{math}
---
label: sigma_sensorless
---
    \sigma = \frac{\beta}{2} + \zeta_\infty |\hatomegam |
```

where $\zeta_\infty$ is the desired damping ratio at high speeds. At zero speed, one pole is placed at $s = 0$ and another at $s = -\beta$. Unstable double pole at $s = 0$ is avoided, enabling stable start of the machine.

## Speed Observer

The flux observer {eq}`sm_obs` is extended with the speed observer in the {class}`motulator.drive.control.sm.SpeedFluxObserver` class. The angle-estimation error signal $\varepsilon$ is extracted as

```{math}
---
label: sm_obs_eps
---
    \varepsilon = -\IM\left\{ \frac{\eo}{\hatpsiaux} \right\}
```

Considering the rotor speed to be a quasi-constant disturbance, the speed can be estimated as {cite}`Hin2018`

```{math}
---
label: sm_speed_obs_ro
---
    \frac{\D \hatomegam}{\D t} = \koomega \varepsilon
```

To avoid the lag in the speed estimate, the speed can be estimated based on the mechanical model and considering the load torque as a disturbance (see {eq}`mech_stiff` in {doc}`/model/drive/mechanics`). This approach results in the speed observer

```{math}
---
label: sm_speed_obs
---
    \frac{\D \hatomegam}{\D t} &= \frac{\np}{\hat{J}}(\hattauM - \hattauL) + \koomega \varepsilon \\
    \frac{\D\hattauL}{\D t} &= -\frac{\kotau}{\np} \varepsilon
```

where $\hattauL$ is the load torque estimate and $\koomega$ and $\kotau$ are the observer gains. Note that setting $\hat{J} = \infty$ and $\kotau = 0$ yields the estimator {eq}`im_speed_obs_ro`; clearly, the inertia estimate $\hat{J}$ can be safely overestimated. Originally {eq}`sm_speed_obs` was used in servo drives with incremental encoders {cite}`Lor1991` and signal-injection methods {cite}`Kim2003`.

### Gain Selection

The flux observer design decouples the speed and position estimation from the flux estimation. Therefore, the speed estimation dynamics can be analyzed separately. The estimator {eq}`sm_speed_obs_ro` results in the second-order estimation dynamics

```{math}
---
label: sm_speed_obs_ro_lin
---
    \frac{\Delta\hatomegam(s)}{\Delta\omegam(s)} = \frac{\alphao^2}{(s + \alphao)^2}
```

The critically damped design is obtained by setting $\kotheta = 2\alphao$ and $\koomega = \alphao^2$, where $\alphao$ is the desired pole location. The inertia estimate is avoided, but the lag limits achievable speed-control bandwidth {cite}`Tii2025a`.

For the observer {eq}`sm_speed_obs`, the linearized estimation dynamics are

```{math}
---
label: sm_speed_obs_lin
---
    \frac{\Delta\hatomegam(s)}{\Delta\omegam(s)} =
    \frac{(J/\hat{J}) s^3 + (J/\hat{J}) \kotheta s^2 + \koomega s + \kotau/\hat{J}}{s^3 + \kotheta s^2 + \koomega s + \kotau/\hat{J}}
```

where the stiff mechanical model is assumed in the derivation. The critically damped design is obtained by setting $\kotheta = 3\alphao$, $\koomega = 3\alphao^2$, and $\kotau = \alphao^3 \hat{J}$.
