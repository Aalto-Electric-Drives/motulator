# Observers

Observers estimate the internal state of electrical machines. We first present a generic rotor speed observer used in both induction and synchronous machine drives (both sensored and sensorless). Then, we describe the flux observer designs for induction and synchronous machines. These designs cover both sensored and sensorless drives and account for magnetic saturation.

## Speed Observer

The rotor speed observer described below is available in the {class}`motulator.drive.control.im.SpeedObserver` and {class}`motulator.drive.control.sm.SpeedObserver` classes for induction machines and synchronous machines, respectively.

The rotor speed can be estimated (or filtered in sensored drives) using a disturbance observer structure {cite}`Fra1997`. The starting point is the mechanical model \[see {eq}`mech_stiff` in {doc}`/model/drive/mechanics`\], in which the load torque is considered as a quasi-constant disturbance. The resulting speed observer is

```{math}
---
label: speed_obs
---
    \frac{\D \hatomegaM}{\D t} &= \frac{1}{\hat{J}}(\hattauM - \hattauL) + \koomega \varepsilon \\
    \frac{\D\hattauL}{\D t} &= -\kotau\varepsilon
```

where $\hatomegaM$ is the estimated (or filtered in sensored drives) speed, $\hattauM$ is the electromagnetic torque estimate, $\hattauL$ is the load torque estimate, and $\koomega$ and $\kotau$ are the observer gains. The error signal $\varepsilon$ depends on the machine type and on whether the drive is sensored or sensorless, as described below. In induction machines, it is based on the speed estimation error; in synchronous machines, it is based on the position estimation error.

As a special case, setting $\hat{J} = \infty$ and $\kotau = 0$ yields the commonly used reduced-order estimator {cite}`Har2001`

```{math}
---
label: speed_obs_ro
---
    \frac{\D \hatomegaM}{\D t} = \koomega \varepsilon
```

which corresponds to a first-order low-pass filter of the true speed. The full-order observer {eq}`speed_obs` uses the mechanical model to avoid the lag inherent in such filtering. Clearly, the inertia estimate $\hat{J}$ can be safely overestimated.

The observer structure {eq}`speed_obs` was originally proposed for sensored servo drives {cite}`Lor1991` and for signal-injection methods {cite}`Kim2003`. We use it also in model-based sensorless control, where the error signal comes from the flux observer instead of a measured speed or position.

## For Induction Machines

In induction machine drives, the flux linkages and torque are estimated. In sensorless drives, the rotor speed is also estimated. Sensored and sensorless reduced-order observer designs {cite}`Ver1988,Har2001,Hin2010` are available in the {class}`motulator.drive.control.im.FluxObserver` and {class}`motulator.drive.control.im.SpeedFluxObserver` classes.

### Machine Model

The inverse-Γ model of an induction machine is considered (see {eq}`im_inv_gamma_ss` in {doc}`/model/drive/induction_machine`). In a coordinate system rotating at the angular speed $\omegac$, the dynamics can be expressed as

```{math}
---
label: im_model_obs
---
    \frac{\D \psis}{\D t} &= \us - \Rs\is - \jj\omegac\psis \\
    \Lsgm\frac{\D \is}{\D t} &= \us - (\Rsgm + \jj\omegac \Lsgm)\is
    + \left(\alpha - \jj\omegam\right)\psiR \\
    \psiR &= \psis - \Lsgm\is \\
    \tauM &= \frac{3\np}{2}\IM\left\{\is \psis^* \right\}
```

where $\alpha = \RR/\LM$ and $\Rsgm = \Rs + \RR$.

### Flux Observer

A reduced-order observer is implemented in the {class}`motulator.drive.control.im.FluxObserver` class. Based on {eq}`im_model_obs`, the observer is formulated as

```{math}
---
label: im_obs
---
    \frac{\D \hatpsis}{\D t} &= \us - \hatRs\is - \jj\omegac\hatpsis + \koa\eo + \kob\eo^* \\
    \hatpsiR &= \hatpsis - \hatLsgm\is
```

where $\us$ is the realized voltage estimate obtained from the PWM algorithm, $\koa$ and $\kob$ are complex gains, the estimates are marked with the hat, and $^*$ marks the complex conjugate. The estimation error is

```{math}
---
label: im_eo
---
    \eo = \hatLsgm\frac{\D \is}{\D t} - \us + \left(\hatRsgm + \jj\omegac\hatLsgm\right)\is - \left(\hatalpha - \jj\hatomegam\right)\hatpsiR
```

where $\hatomegam = \np\hatomegaM$ represents the filtered rotor speed in sensored drives and estimated rotor speed in sensorless drives. In observer-based V/Hz control {cite}`Tii2025b`, the speed estimate is replaced with the (rate-limited) speed reference, $\hatomegam = \omegamref$. Note that the derivative of the stator current in {eq}`im_eo` is integrated, i.e., the noise is not amplified. The torque estimate is given by

```{math}
---
label: im_obs_tauM
---
    \hattauM = \frac{3\np}{2}\IM\left\{\is \hatpsis^* \right\}
```

The magnetic saturation $\hatLs = \hatLs(\hatabspsis)$ can be taken into account using the {class}`motulator.drive.model.InductionMachinePars` class.

```{note}
The angular speed $\omegac$ of the controller coordinate system can be arbitrarily selected. In the {class}`motulator.drive.control.im.FluxObserver` class, it is set to $\omegac = \hatomegam + \hatomegar$, which allows simple discretization since the DC quantities are estimated in the steady state. Estimated rotor coordinates $\omegac = \hatomegam$ could also be used, but using $\omegac = 0$ would require a more complex discretization.
```

```{note}
Real-valued column vectors and the corresponding $2\times 2$ gain matrix were used in {cite}`Hin2010`. The complex form in {eq}`im_obs` has the same degrees of freedom.
```

(im_obs_analysis)=

#### Gain Analysis and Selection

The estimation-error dynamics are obtained by subtracting {eq}`im_obs` from {eq}`im_model_obs`. The resulting system is linearized for analysis and gain selection purposes. Assuming accurate parameter estimates, linearized estimation-error dynamics are {cite}`Hin2010`

```{math}
---
label: tilde_psis
---
    \frac{\D \Delta\tildepsis}{\D t} = -\koa \Delta \eo - \kob \Delta \eo^* - \jj\omegaso \Delta\tildepsis
```

where $\Delta$ marks the small-signal quantities, the subscript 0 marks the operating-point quantities, $\tildepsis = \psis - \hatpsis$ is the estimation error, and $\omegaso$ is the stator angular frequency. The linearized estimation error is

```{math}
---
label: eo_lin
---
    \Delta\eo = \left(\alpha - \jj\omegamo \right)\Delta\tildepsis - \jj\psiRo\Delta\tildeomegam
```

Notice that the rotor flux estimation error is $\Delta\tildepsiR = \Delta\tildepsis$.

##### Sensored Drives

In sensored drives, $\Delta\tildeomegam$ decoupled from the flux estimation. Consequently, this external disturbance can be omitted in the analysis, $\Delta\tildeomegam = 0$. Thus, the gain $\kob = 0$ can be used. Under these assumptions, the estimation-error dynamics {eq}`tilde_psis` reduce to {cite}`Ver1988`

```{math}
---
label: im_tilde_psis_sensored
---
    \frac{\D \Delta\tildepsis}{\D t} = -\left[\koa \left(\alpha - \jj\omegamo \right) + \jj\omegaso \right]\Delta\tildepsis
```

The closed-loop pole can be arbitrarily placed via the gain $\koa$. The default gains for sensored drives are

```{math}
---
label: k1_sensored
---
    \koa = 1 + \frac{g |\omegam|}{\hatalpha - \jj\omegam} \qquad
    \kob = 0
```

where $g$ is a unitless positive design parameter. The corresponding pole is located at $s = -\alpha - g |\omegamo| - \jj\omegaro$, where $\omegaro = \omegaso - \omegamo$ is the slip angular frequency.

```{note}
As a special case, $\koa = 0$ yields the voltage model. As another special case, the current model is obtained choosing $\koa = 1$.
```

(im_obs_sensorless)=

##### Sensorless Drives

In sensorless drives, the rotor speed estimate $\hatomegam$ is canceled out from the observer equations by choosing {cite}`Hin2010`

```{math}
---
label: inherently
---
    \kob = \frac{\hatpsiR}{\hatpsiR^*} \koa
```

With this choice, the linearized estimation-error dynamics in {eq}`tilde_psis` become

```{math}
---
label: tilde_psis_sensorless
---
    \frac{\D}{\D t} \begin{bmatrix} \Delta\tildepsisd \\ \Delta\tildepsisq \end{bmatrix} = \begin{bmatrix} -2\kd\alpha & -2\kd\omegamo + \omegaso \\ -2\kq\alpha - \omegaso & -2\kq\omegamo
    \end{bmatrix}
    \begin{bmatrix} \Delta\tildepsisd \\ \Delta\tildepsisq \end{bmatrix}
```

where the gain components correspond to $\koa = \kd + \jj \kq$. The attenuation $\sigma$ can be assigned by choosing

```{math}
---
label: k1_sensorless
---
    \koa = \frac{\sigma}{\hat \alpha - \jj\hatomegam}
```

which results in the characteristic polynomial $D(s) = s^2 + 2\sigma s + \omegaso^2$. In the default tuning, the attenuation is scheduled as $\sigma = \hat \alpha/2 + \zeta_\infty|\hatomegam|$, where $\zeta_\infty$ is the desired damping ratio at high speeds. At zero stator frequency $\omegaso = 0$, the poles are located at $s = 0$ and $s = -\alpha$, which allows stable magnetizing and starting the machine. [Figure 1](fig:poles) shows the corresponding pole placement example.

```{figure} ../figs/poles.svg
---
name: fig:poles
class: only-light
width: 100%
align: center
alt: Pole placement example in sensorless drives
---
*Figure 1:* Example pole placement of the sensorless observer.
```

```{figure} ../figs/poles.svg
---
class: invert-colors-dark only-dark
width: 100%
align: center
alt: Pole placement example in sensorless drives
---
*Figure 1:* Example pole placement of the sensorless observer.
```

### Speed and Flux Observer

To estimate the rotor speed, the flux observer {eq}`im_obs` is extended with the speed observer {eq}`speed_obs` in the {class}`motulator.drive.control.im.SpeedFluxObserver` class. The error signal is different in sensored and sensorless drives, as described below.

#### Sensored Drives

In sensored drives, the error signal for the mechanical rotor speed is

```{math}
---
label: im_obs_eps_sensored
---
    \varepsilon = \omegaMmeas - \hatomegaM
```

where $\omegaMmeas$ is the measured speed and $\hatomegaM$ is the filtered speed. The measured speed $\omegaMmeas$ may contain a significant amount of noise (such as quantization noise from incremental encoders), which is filtered by the speed observer.

#### Sensorless Drives

In sensorless drives, the estimation error of the mechanical rotor speed is obtained from the flux observer {eq}`im_obs` and {eq}`im_eo` as

```{math}
---
label: im_obs_eps
---
    \varepsilon = -\frac{1}{\np}\IM\left\{ \frac{\eo}{\hatpsiR} \right\}
```

The reduced-order speed observer {eq}`speed_obs_ro` with the error signal {eq}`im_obs_eps` is essentially the same as the conventional slip-relation-based estimator with the first-order low-pass filter, see {cite}`Hin2010`.

#### Gain Analysis and Selection

The flux observer gain {eq}`inherently` decouples the rotor speed estimation from the flux estimation. Therefore, the speed estimation dynamics can be analyzed separately. Assuming an ideal measurement ($\omegaMmeas = \omegaM$) in the sensored case, the estimation dynamics are identical for both sensored and sensorless drives.

For the observer {eq}`speed_obs`, the linearized estimation dynamics are

```{math}
---
label: im_speed_obs_lin
---
    \frac{\Delta\hatomegaM(s)}{\Delta\omegaM(s)} = \frac{(J/\hat{J})s^2 + \koomega s + \kotau/\hat{J}}{s^2 + \koomega s + \kotau/\hat{J}}
```

where the stiff mechanical model is assumed in the derivation. The critically damped design is obtained by setting $\koomega = 2\alphao$ and $\kotau = \alphao^2 \hat{J}$, where $\alphao$ is the desired pole location.

As a special case of {eq}`speed_obs`, setting $\hat{J} = \infty$ and $\kotau = 0$ yields {eq}`speed_obs_ro`. This reduced-order estimator gives the first-order estimation dynamics

```{math}
---
label: im_speed_obs_ro_lin
---
    \frac{\Delta\hatomegaM(s)}{\Delta\omegaM(s)} = \frac{\koomega}{s + \koomega}
```

The gain $\koomega = \alphao$ determines the speed-estimation bandwidth.

## For Synchronous Machines

In synchronous machine drives, the flux linkage and the torque are estimated. In sensorless drives, the rotor speed and position are also estimated {cite}`Jon1989,Cap2001,Pii2008,Hin2018`. This document describes an observer design implemented in the {class}`motulator.drive.control.sm.FluxObserver` and {class}`motulator.drive.control.sm.SpeedFluxObserver` classes, based on {cite}`Hin2018`. The observer supports both sensorless and sensored operating modes and accounts for magnetic saturation.

### Machine Model

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

### Coordinate Transformation

The control system operates in estimated rotor coordinates, $\thetac = \hatthetam$, aligned at the rotor angle estimate. In these coordinates, the measured current and the realized voltage (obtained from the PWM algorithm), respectively, are

```{math}
---
label: is_prime
---
    \is' = \iss \e^{-\jj\hatthetam} \qquad
    \us' = \us \e^{-\jj\hatthetam}
```

Due to the estimation error $\tildethetam = \thetam - \hatthetam$, the current $\is'$ generally differs from the current $\is$ (and similarly for the voltage).

### Flux and Position Observer

Based on {eq}`sm_model_obs`, a nonlinear state observer is formulated as

```{math}
---
label: sm_obs
---
    \frac{\D \hatpsis}{\D t} &= \us' - \hatRs\is' - \jj\omegac\hatpsis + \koa \eo + \kob \eo^* \\
    \frac{\D\hatthetam}{\D t} &= \hatomegam + \kotheta \np\varepsilon = \omegac
```

where $\us'$ is the realized voltage estimate obtained from the PWM algorithm, $\omegac$ is the angular speed of the coordinate system, and $\koa$, $\kob$, and $\kotheta$ are observer gains. Furthermore, $\hatomegam = \np \hatomegaM$ is the estimated rotor speed (in electrical rad/s) and $\varepsilon$ is the rotor position estimation error (in mechanical rad). The flux estimation error is

```{math}
---
label: sm_eo
---
    \eo = \hatpsisfcn(\is') - \hatpsis
```

where $\hatpsisfcn$ is the flux map estimate. In sensored drives, the estimation error signal is

```{math}
---
label: sm_obs_eps_sensored
---
    \varepsilon = \thetaMmeas - \hatthetam/\np
```

where $\thetaMmeas$ is the measured mechanical angular position. In sensorless drives, the estimation error signal of the mechanical rotor position is obtained from {eq}`sm_obs` and {eq}`sm_eo` as {cite}`Hin2018`

```{math}
---
label: sm_obs_eps_sensorless
---
    \varepsilon = -\frac{1}{\np}\IM\left\{ \frac{\eo}{\hatpsiaux} \right\}
```

The mechanical position is used in these signals for compatibility with the generic speed observer {eq}`speed_obs`. The torque estimate is given by

```{math}
---
label: sm_obs_tauM
---
    \hattauM = \frac{3\np}{2}\IM\left\{\is' \hatpsis^* \right\}
```

This observer structure is used in the {class}`motulator.drive.control.sm.FluxObserver` class. The implementation also contains optional PM-flux adaptation {cite}`Tuo2018`, see the {doc}`/drive_examples/current_vector/plot_2kw_ipmsm_cvc_adapt` example.

```{note}
Since the current is measured, the observer is fundamentally corrected by means of the current estimation error. However, due to the saliency and magnetic saturation, the current estimation error is convenient to map (or scale in the case of linear magnetics) to the flux linkage error.
```

```{note}
Real-valued column vectors and the corresponding $2\times 2$ gain matrix were used in {cite}`Hin2018`. The complex form in {eq}`sm_obs` has the same degrees of freedom.
```

#### Gain Analysis and Selection

The analysis resembles that of induction machines, see {ref}`im_obs_analysis`. The following results can be derived from the linearized form of {eq}`sm_model_obs` -- {eq}`sm_obs_tauM`, see details in {cite}`Hin2018`.

##### Sensored Drives

In sensored case, the rotor position estimation error is decoupled from the flux estimation, i.e., it acts as an external disturbance in the flux estimation. Therefore, the gain $\kob = 0$ can be selected. Using {eq}`sm_model_obs` and {eq}`sm_obs`, the linearized estimation-error dynamics become

```{math}
---
label: sm_tilde_psis_sensored
---
    \frac{\D \Delta\tildepsis}{\D t} = -(\koa + \jj\omegamo)\Delta\tildepsis
```

where $\Delta$ marks the small-signal quantities, the subscript 0 marks the operating-point quantities, and $\tildepsis = \psis - \hatpsis$ is the estimation error. The pole can be arbitrarily placed via the gain $\koa$. Well-damped dynamics are obtained simply with a real gain, $\koa = \sigma$, resulting in the pole at $s = -\sigma - \jj\omegamo$, where $\sigma = 2\pi \cdot 15$ rad/s is used as the default value in the {class}`motulator.drive.control.sm.FluxObserver` class in sensored drives.

(sm_obs_sensorless)=

##### Sensorless Drives

To decouple the flux estimation from the rotor angle, the gains of {eq}`sm_obs` have to be of the form {cite}`Hin2018`

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

### Speed and Flux Observer

To estimate the rotor speed and position, the flux observer {eq}`sm_obs` is extended with the speed observer {eq}`speed_obs` in the {class}`motulator.drive.control.sm.SpeedFluxObserver` class. The error signals are defined above in {eq}`sm_obs_eps_sensored` and {eq}`sm_obs_eps_sensorless` for sensored and sensorless drives, respectively.

#### Gain Analysis and Selection

The flux observer design decouples the speed and position estimation from the flux estimation. Therefore, the speed estimation dynamics can be analyzed separately. Assuming an ideal measurement ($\thetaMmeas = \thetaM$) in the sensored case, the estimation dynamics are identical for both sensored and sensorless drives.

For the observer {eq}`speed_obs`, the linearized estimation dynamics are

```{math}
---
label: sm_speed_obs_lin
---
    \frac{\Delta\hatomegaM(s)}{\Delta\omegaM(s)} =
    \frac{(J/\hat{J}) s^3 + (J/\hat{J}) \kotheta s^2 + \koomega s + \kotau/\hat{J}}{s^3 + \kotheta s^2 + \koomega s + \kotau/\hat{J}}
```

where the stiff mechanical model is assumed in the derivation. The critically damped design is obtained by setting $\kotheta = 3\alphao$, $\koomega = 3\alphao^2$, and $\kotau = \alphao^3 \hat{J}$.

As a special case of {eq}`speed_obs`, setting $\hat{J} = \infty$ and $\kotau = 0$ yields {eq}`speed_obs_ro`. This reduced-order estimator gives the second-order estimation dynamics

```{math}
---
label: sm_speed_obs_ro_lin
---
    \frac{\Delta\hatomegaM(s)}{\Delta\omegaM(s)} = \frac{\alphao^2}{(s + \alphao)^2}
```

The critically damped design is obtained by setting $\kotheta = 2\alphao$ and $\koomega = \alphao^2$, where $\alphao$ is the desired pole location. The inertia estimate is avoided, but the lag limits achievable speed-control bandwidth {cite}`Tii2025a`.
