# Output LC Filter

An LC filter may be used between the voltage-source converter and the electric machine in some applications {cite}`Sal2006`. [Figure 1](fig:lc_filter) shows its space-vector model in stator coordinates. The subscripts c and s refer to the converter-side and stator-side quantities, respectively. The LC filter model is implemented in the {class}`motulator.drive.model.LCFilter` class.  

```{figure} ../figs/lc_filter.svg
    :name: fig:lc_filter
    :class: only-light
    :width: 100%
    :align: center
    :alt: LC filter

*Figure 1:* LC filter in stator coordinates. 
```

```{figure} ../figs/lc_filter.svg
    :class: invert-colors-dark only-dark
    :width: 100%
    :align: center
    :alt: LC filter

*Figure 1:* LC filter in stator coordinates. 
```

In general coordinates rotating at the angular speed $\omegac$, the dynamic model of the filter is

```{math}
    :label: LC_filter_model

    \Lf \frac{\D\ic}{\D t} &= \uc - \us - \Rf \ic - \jj\omegac \Lf\ic \\
    \Cf \frac{\D\us}{\D t} &= \ic - \is - \jj\omegac \Cf\us
```

where $\Lf$ is the filter inductance, $\Rf$ is the series resistance of the filter inductor, and $\Cf$ the filter capacitance. Furthermore, $\ic$ is the converter current, $\is$ is the stator current, $\uc$ is the converter voltage, and $\us$ is the capacitor voltage (corresponding to the stator voltage). 

See also the {doc}`/drive_examples/vhz/plot_2kw_im_lc_vhz` example.