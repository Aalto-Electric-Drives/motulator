# Disturbance-Observer-Based Grid-Forming Control

In these notes, disturbance-observer-based grid-forming control is discussed {cite}`Nur2024`. This control method is available in the {class}`motulator.grid.control.ObserverBasedGridFormingController` class. As can be realized by comparing examples {doc}`/grid_examples/grid_forming/plot_13kva_rfpsc_gfm` and {doc}`/grid_examples/grid_forming/plot_13kva_do_gfm`, the observer-based grid-forming control can be configured to behave practically identically with reference-feedforward power-synchronization control (PSC) {cite}`Har2020`. Compared to reference-feedforward PSC, disturbance-observer-based grid-forming control is easier to analyze and extend with different operating modes (including grid-following and transparent current-control modes). The order of these two control methods is the same and their computational complexity is comparable.

## System Model

First, the system is modeled in general coordinates, whose angle with respect to the stationary coordinates is $\thetac$ and the angular speed is $\omegac = \D\thetac/\D t$. The dynamics of the inductor current $\ic$ and the grid voltage $\ug$ are modeled as

```{math}
:label: system_model1

L\frac{\D \ic}{\D t} &= \uc - \ug - \jj\omegac L \ic \\
\frac{\D\ug}{\D t} &= \jj(\omegag - \omegac)\ug
```

where $L$ is the inductance and $\omegag$ is the grid angular frequency. The grid voltage $\ug$ is modeled as a disturbance; this same disturbance model for the grid voltage is also used in the development of {doc}`/control/grid/pll`. The active power fed to the grid is nonlinear in the state variables

```{math}
:label: power1

\pg = \frac{3}{2}\RE\{\ug\ic^*\}
```

For the purpose of grid-forming control, we may define the quasi-static converter voltage as an output variable as a function of the state variables,

```{math}
:label: quasi_static_converter_voltage

\vc = \ug + \jj\omegag L \ic
```

This voltage is controllable (unlike the grid voltage which is the disturbance) and it equals the converter output voltage $\uc$ in the steady state. The disturbance-observer-based PSC regulates the voltage magnitude $|\vc|$ and the power $p_\mathrm{g}$ fed to grid.

## Control System

The control system consists of a disturbance observer and a control law. The disturbance observer estimates the quasi-static converter voltage and the power fed to the grid. The control law generates the converter voltage reference based on the estimated grid voltage and power. The disturbance observer also provides the integral action for the control law. This control structure allows seamless switching between grid-forming and grid-following modes {cite}`Nur2024`. For simplicity, only the grid-forming mode is considered in the following.

### Disturbance Observer

Based on {eq}`system_model1`--{eq}`quasi_static_converter_voltage`, a disturbance observer for the grid voltage can be formed {cite}`Nur2024,Kuk2021,Fra1997`

```{math}
:label: disturbance_observer_gfm

\frac{\D \hatug}{\D t} &= \jj (\hatomegag - \omegac)\hatug + \alphao\left(\uc - \hat L \frac{\D \ic}{\D t} - \jj \omegac \hat L \ic - \hatug \right) \\
\hatvc &= \hatug + \jj\hatomegag \hat L \ic \\
\hatpg &= \frac{3}{2}\RE\{\hatvc\ic^*\}
```

where $\hatomegag$ is the nominal grid angular frequency and estimates are marked with hat.

```{note}
A conventional phase-locked loop (PLL) can be expressed in the same disturbance observer framework, see the {doc}`/control/grid/pll` notes. It can be realized that the measured grid voltage $\ug$ used in the conventional PLL is replaced by its converter-voltage-based estimate $\uc - \hat L (\D \ic/\D t) - \jj \omegac \hat L \ic$ in the disturbance observer {eq}`disturbance_observer_gfm`.
```

### Control Law

A nonlinear state feedback law is used

```{math}
:label: control_law_gfm

\ucref = \hatvc + \kP (\pgref - \hatpg) + \kV (\vcref - \hatabsvc)
```

where $\pgref$ is the active power reference, $\vcref$ is the converter voltage magnitude reference, and $\hatabsvc = |\hatvc|$ is the magnitude. The complex gains for the active-power and converter-voltage-magnitude channels, respectively, are selected as

```{math}
:label: gain_selection_gfm

\kP = \frac{R_\mathrm{a}}{v_\mathrm{c,ref}} \frac{\hatvc}{\hatabsvc} \qquad
\kV = (1 - \jj k_\mathrm{v}) \frac{\hatvc}{\hatabsvc}
```

where the gains $R_\mathrm{a} = 0.2$ p.u. and $k_\mathrm{v} = \alphao/\omegag$ can be used.

### Implementation Aspects

To avoid the derivate on the right-hand side of {eq}`disturbance_observer_gfm`, a new state variable $\hatug' = \hatug + \alphao \hat L \ic$ can be introduced {cite}`Fra1997`. Furthermore, the coordinate system for the implementation can be chosen freely. The simplest choice is to use the nominal grid frequency as the coordinate system frequency, $\omegac = \hatomegag$. Using these design choices, the whole control system consisting of the disturbance observer {eq}`disturbance_observer_gfm` and the control law {eq}`control_law_gfm` in the state-space form reduces to {cite}`Nur2024`

```{math}
:label: control_system_gfm

\frac{\D \hatug'}{\D t} &= \alphao (\ucref - \hatvc) \\
\hatvc &= \hatug' - (\alphao - \jj\hatomegag) \hat L \ic \\
\hatpg &= \frac{3}{2}\RE\{\hatvc\ic^*\} \\
\ucref &= \hatvc + \kP (\pgref - \hatpg) + \kV (\vcref - \hatabsvc)
```

where the gains can be selected according to {eq}`gain_selection_gfm` and the converter voltage appearing in the observer has been replaced with its reference. Various control modes could be easily incorporated into the control system {eq}`control_system_gfm`, simply by changing the feedback correction terms of the control law {cite}`Nur2024`. The switching between the modes is seamless since the control law does not have memory, but the integral action is provided by the disturbance observer (in addition to synchronization).

The control system implemented in the {class}`motulator.grid.control.ObserverBasedGridFormingController` class corresponds to {eq}`control_system_gfm`. In the example implementation, a transparent current-control mode is implemented. In the grid-forming mode, the observer bandwidth $\alphao = 1$ p.u. can be used. Furthermore, the inductance estimate can be set close to the lowest expected inductance value, e.g., $\hat L = 0.15$ p.u. Using this configuration, the robust performance from strong grids to very weak grids can be achieved. This grid-forming control method can also be used with LCL filters, similarly to reference-feedforward PSC.
