# AC Filter and Grid Impedance

This document describes continuous-time models of an AC filter and grid impedance.

## L Filter

[Figure 1](fig:l_filter) shows a space-vector equivalent circuit of an L filter and inductive-resistive grid impedance in stationary coordinates. This model is implemented in the {class}`motulator.grid.model.LFilter` class.

In general coordinates rotating at $\omegac$, the model is

```{math}
---
label: L_filter_model
---
    \Lt\frac{\D\ic}{\D t} = \uc - \eg - \Rt\ic - \jj\omegac \Lt\ic
```

where $\ic$ is the converter current, $\uc$ is the converter voltage, $\eg$ is the grid voltage, $\Rt = \Rfc + \Rg$ is the total resistance comprising the filter series resistance $\Rfc$ and the grid resistance $\Rg$, and $\Lt = \Lfc + \Lg$ is the total inductance comprising the filter inductance $\Lfc$ and the grid inductance $\Lg$. The point of common coupling (PCC) is modeled to be between the L filter and the grid impedance. The voltage at the PCC is

```{math}
---
label: L_filter_PCC_voltage
---
    \ug = \frac{\Lg(\uc - \Rfc\ic) + \Lfc(\eg + \Rg\ic)}{\Lt}
```

```{figure} ../figs/l_filter.svg
---
name: fig:l_filter
class: only-light
width: 100%
align: center
alt: Diagram of L filter and grid impedance
---
*Figure 1:* L filter and inductive-resistive grid impedance in stationary coordinates.
```

```{figure} ../figs/l_filter.svg
---
class: invert-colors-dark only-dark
width: 100%
align: center
alt: Diagram of L filter and grid impedance
---
*Figure 1:* L filter and inductive-resistive grid impedance in stationary coordinates.
```

## LCL Filter

[Figure 2](fig:lcl_filter) shows a space-vector equivalent circuit of an LCL filter and inductive-resistive grid impedance in stationary coordinates. This model is implemented in the {class}`motulator.grid.model.LCLFilter` class.

In general coordinates rotating at $\omegac$, the model is

```{math}
---
label: LCL_filter_model
---
    \Lfc\frac{\D\ic}{\D t} &= \uc - \uf - \Rfc\ic - \jj\omegac\Lfc\ic \\
    \Cf\frac{\D\uf}{\D t} &= \ic - \ig - \jj\omegac\Cf\uf \\
    \Lt\frac{\D\ig}{\D t} &= \uf - \eg - \Rt\ig - \jj\omegac\Lt\ig
```

where $\ic$ is the converter current, $\ig$ is the grid current, and $\uf$ is the capacitor voltage. The converter-side and grid-side inductances of the LCL filter are $\Lfc$ and $\Lfg$, respectively, and their series resistances are $\Rfc$ and $\Rfg$, respectively. The filter capacitance is $\Cf$. The total grid-side inductance and resistance are $\Lt = \Lfg + \Lg$ and $\Rt = \Rfg + \Rg$, respectively. The PCC is modeled to be between the LCL filter and the inductive-resistive grid impedance. The voltage at the PCC is

```{math}
---
label: LCL_filter_PCC_voltage
---
    \ug = \frac{\Lg(\uf - \Rfg\ig) + \Lfg(\eg + \Rg\ig)}{\Lt}
```

```{figure} ../figs/lcl_filter.svg
---
name: fig:lcl_filter
class: only-light
width: 100%
align: center
alt: Diagram of LCL filter and grid impedance
---
*Figure 2:* LCL filter and inductive-resistive grid impedance in stationary coordinates.
```

```{figure} ../figs/lcl_filter.svg
---
class: invert-colors-dark only-dark
width: 100%
align: center
alt: Diagram of LCL filter and grid impedance
---
*Figure 2:* LCL filter and inductive-resistive grid impedance in stationary coordinates.
```
