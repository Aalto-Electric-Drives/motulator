# Phase-Locked Loop

A phase-locked loop (PLL) is commonly used in grid-following converters to synchronize the converter output with the grid {cite}`Kau1997`. Here, we represent the PLL using the disturbance observer structure {cite}`Fra1997`, which may be simpler to extend than the classical PLL. Furthermore, this structure allows to see links to synchronization methods used in grid-forming converters, see {cite}`Nur2024`.

## Disturbance Model

Consider the positive-sequence voltage at the point of common coupling (PCC) in general coordinates, rotating at the angular speed $\omegac$. The dynamics of the PCC voltage $\ug$ can be modeled using a disturbance model as

```{math}
---
label: disturbance
---
\frac{\D\ug}{\D t} = \jj(\omegag - \omegac)\ug
```

where $\omegag$ is the grid angular frequency. If needed, this disturbance model could be extended, e.g., with a negative-sequence component, allowing to design the PLL for unbalanced grids.

```{note}
The disturbance model in {eq}`disturbance` can be equivalently expressed in polar form as

\begin{align}
\frac{\D \absug}{\D t} &= 0 \\
\frac{\D \delta_\mathrm{g}}{\D t} &= \omegag - \omegac \\
\ug &= \absug \e^{\jj\delta_\mathrm{g}}
\end{align}

where $\delta_\mathrm{g} = \thetag - \vartheta_\mathrm{c}$ is the angle of the grid voltage in general coordinates and the last equation is the output equation.
```

## PLL in General Coordinates

Based on {eq}`disturbance`, the disturbance observer containing the regular PLL functionality can be formulated as

```{math}
---
label: pll
---
\frac{\D\hatug}{\D t} = \jj(\hatomegag - \omegac)\hatug + \alphao (\ug - \hatug )
```

where $\hatug$ is the estimated PCC voltage, $\hatomegag$ is the grid angular frequency estimate (either constant corresponding to the nominal value or dynamic from grid-frequency tracking), and $\alphao$ is the bandwidth. If needed, the disturbance observer can be extended with the frequency tracking as

```{math}
---
label: frequency_tracking
---
\frac{\D\hatomegag}{\D t} = k_\upomega\IM\left\{ \frac{\ug - \hatug}{\hatug} \right\}
```

where $k_\upomega$ is the frequency-tracking gain. Notice that {eq}`pll` and {eq}`frequency_tracking` are both driven by the estimation error $\ug - \hatug$ of the PCC voltage.

## PLL in Estimated PCC Voltage Coordinates

The disturbance observer {eq}`pll` can be equivalently expressed in estimated PCC voltage coordinates, where $\hatug = \hatabsug$ and $\omegac = \D \hatthetag/ \D t$, resulting in

```{math}
---
label: pll_polar
---
\frac{\D\hatabsug}{\D t} &= \alphao \left(\mathrm{Re}\{\ug\} - \hatabsug \right) \\
\frac{\D \hatthetag}{\D t} &= \hatomegag + \frac{\alphao}{\hatabsug}\IM\{ \ug \} = \omegac
```

It can be seen that the first equation in {eq}`pll_polar` is low-pass filtering of the measured PCC voltage magnitude and the second equation is the conventional angle-tracking PLL. In these coordinates, the frequency tracking {eq}`frequency_tracking` reduces to

```{math}
---
label: frequency_tracking_polar
---
\frac{\D \hatomegag}{\D t} = \frac{k_\upomega}{\hatabsug } \IM\{ \ug \}
```

It can be noticed that the disturbance observer with the frequency tracking equals the conventional frequency-adaptive PLL {cite}`Kau1997`, with the additional feature of low-pass filtering the PCC voltage magnitude. The low-pass filtered PCC voltage can be used as a feedforward term in current control {cite}`Har2021`.

## Linearized Closed-Loop System

The estimation-error dynamics are analyzed by means of linearization. Using the PCC voltage as an example, the small-signal deviation about the operating point is denoted by $\Delta \ug = \ug - \ugo$, where $\ugo$ is the operating-point quantity. From {eq}`disturbance`--{eq}`frequency_tracking`, the linearized model for the estimation-error dynamics is obtained as

```{math}
---
label: linearized_model
---
\frac{\D\Delta \tildeug}{\D t} &= -\alphao\Delta \tildeug + \jj\ugo (\Delta \omegag - \Delta \hatomegag) \\
\frac{\D\Delta \hatomegag}{\D t} &= k_\omega\IM\left\{ \frac{\Delta \tildeug}{\ugo} \right\}
```

where $\Delta \tildeug = \Delta\ug - \Delta \hatug$ is the estimation error.

First, assume that the grid frequency $\omegag$ is constant and the frequency tracking is disabled. From {eq}`linearized_model`, the closed-loop transfer function from the PCC voltage to its estimate becomes

```{math}
---
label: closed_loop_pll
---
\frac{\Delta\hatug(s)}{\Delta\ug(s)} = \frac{\alphao}{s + \alphao}
```

It can be realized that both the angle and magnitude of the PCC voltage estimate converge with the bandwidth $\alphao$.

Next, the frequency-tracking dynamics are also considered. From {eq}`linearized_model`, the closed-loop transfer function from the grid angular frequency to its estimate becomes

```{math}
---
label: closed_loop_pll_frequency_tracking
---
\frac{\Delta\hatomegag(s)}{\Delta\omegag(s)} = \frac{k_\upomega}{s^2 + \alphao s + k_\omega}
```

Choosing $k_\upomega = \alphapll^2$ and $\alphao = 2\alphapll$ yields the double pole at $s = -\alphapll$, where $\alphapll$ is the frequency-tracking bandwidth.

This PLL in estimated PCC coordinates is implemented in the class {class}`motulator.grid.control.PLL`. The {doc}`/grid_examples/grid_following/index` examples use the PLL to synchronize with the grid.
