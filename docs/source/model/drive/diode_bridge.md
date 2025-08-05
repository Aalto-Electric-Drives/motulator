# Six-Pulse Diode Bridge

[Figure 1](fig:diode_bridge) shows a six-pulse diode bridge rectifier, where the inductor $L_\mathrm{dc}$ is placed in the DC link. For simplicity, a three-phase supply voltage is assumed to be stiff. The DC current dynamics are

```{math}
    :label: diode_bridge

    \Ldc \frac{\D \iL}{\D t} = \udi - \udc
```

where $\iL$ is the DC-bus current, $\udi$ is the voltage over the diode bridge, $\udc$ is the DC-bus voltage, and $\Ldc$ is the DC-bus inductance.

```{figure} ../figs/diode_bridge.svg
    :name: fig:diode_bridge
    :class: only-light
    :width: 100%
    :align: center
    :alt: Six-pulse diode bridge rectifier an three-phase supply voltage

*Figure 1:* Six-pulse diode bridge rectifier.
```

```{figure} ../figs/diode_bridge.svg
    :class: invert-colors-dark only-dark
    :width: 100%
    :align: center
    :alt: Six-pulse diode bridge rectifier an three-phase supply voltage

*Figure 1:* Six-pulse diode bridge rectifier.
```

The voltage-source converter described in the {doc}`/model/common/converters` document is extended with this diode bridge model in the {class}`motulator.drive.model.FrequencyConverter` class. Examples using the six-pulse diode bridge can be found in {doc}`/drive_examples/vhz/plot_2kw_im_diode_vhz` and {doc}`/drive_examples/current_vector/plot_2kw_ipmsm_diode_cvc`.
