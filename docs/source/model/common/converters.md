# Voltage-Source Converter

## Model

[Figure 1](fig:inverter) shows a three-phase two-level voltage-source converter, where $\udc$ is the DC-bus voltage, $\idc$ is the external DC current, and $\Cdc$ is the DC-bus capacitance. The converter can operate both as an inverter and a rectifier, depending on the direction of the power flow. This model is provided in the {class}`motulator.drive.model.VoltageSourceConverter` class. It can be extended with a diode bridge model, see {doc}`/model/drive/diode_bridge`.

```{figure} ../figs/inverter.svg
    :name: fig:inverter
    :class: only-light
    :width: 100%
    :align: center
    :alt: Three-phase two-level voltage-source converter

*Figure 1:* Three-phase two-level voltage-source converter. The negative potential of the DC bus is marked with N and the output terminals with a, b, and c.
```

```{figure} ../figs/inverter.svg
    :class: invert-colors-dark only-dark
    :width: 100%
    :align: center
    :alt: Three-phase two-level voltage-source converter

*Figure 1:* Three-phase two-level voltage-source converter. The negative potential of the DC bus is marked with N and the output terminals with a, b, and c.
```

Assuming ideal transistors and diodes, the converter can be modeled with the equivalent circuit shown in [Figure 2](fig:pwm_inverter), in which the legs are modeled as bi-positional switches. Each changeover switch is connected to either negative or positive potential of the DC bus, and the switching phenomena are assumed to be infinitely fast. The state of each switch is defined using the switching state, which, using phase $a$ as an example, is $\qA = 0$ when the switch is connected to the negative potential and $\qA = 1$ when the switch is connected to the positive potential.

```{figure} ../figs/pwm_inverter.svg
    :name: fig:pwm_inverter
    :class: only-light
    :width: 100%
    :align: center
    :alt: Voltage-source converter and carrier comparison

*Figure 2:* Equivalent circuit of a three-phase voltage-source converter, connected to a generic three-phase load. The neutral point of the load is marked with n. In this example, the positions of the bi-positional switches correspond to the instantaneous switching states $\qA = 1$, $\qB = 0$, and $\qC = 0$.
```

```{figure} ../figs/pwm_inverter.svg
    :class: invert-colors-dark only-dark
    :width: 100%
    :align: center
    :alt: Voltage-source converter and carrier comparison

*Figure 2:* Equivalent circuit of a three-phase voltage-source converter, connected to a generic three-phase load. The neutral point of the load is marked with n. In this example, the positions of the bi-positional switches correspond to the instantaneous switching states $\qA = 1$, $\qB = 0$, and $\qC = 0$.
```

By default, the DC-bus voltage is constant, i.e., the DC-bus capacitor is replaced with a constant DC voltage source. Alternatively, if the DC bus is fed from an external current source $\idc$, the DC-bus dynamics are modeled as

```{math}
    :label: DC_bus_model

    \Cdc \frac{\D \udc}{\D t} = \idc - \idc'
```

where the converter-side DC current depends on the phase currents $\iA$, $\iB$, and $\iC$ as

```{math}
    :label: DC_current

    \idc' = \qA \iA + \qB \iB + \qC \iC
```

## Carrier Comparison

In pulse-width modulation (PWM), carrier comparison is commonly used to generate instantaneous switching state signals $\qA$, $\qB$, and $\qC$ from duty ratios $\dA$, $\dB$, and $\dC$. The duty ratios are continuous signals in the range [0, 1] while the switching states are either 0 or 1.

[Figure 3](fig:carrier_comparison) shows the principle of carrier comparison. The logic shown in the figure is implemented in the {class}`motulator.common.model.CarrierComparison` class, where the switching instants are explicitly computed in the beginning of each sampling period (instead of searching for zero crossings), allowing faster simulations. The sampling period $\Ts$ is returned by the control method, and it does not need to be constant.

```{figure} ../figs/carrier_comparison.svg
    :name: fig:carrier_comparison
    :class: only-light
    :width: 100%
    :align: center
    :alt: Carrier comparison

*Figure 3:* Carrier comparison. The duty ratios $\dA$, $\dB$, and $\dC$ are constant over the sampling period $\Ts$ (or, optionally, over the switching period $\Tsw = 2\Ts$). The carrier signal is the same for all three phases and varies between 0 and 1.
```

```{figure} ../figs/carrier_comparison.svg
    :class: invert-colors-dark only-dark
    :width: 100%
    :align: center
    :alt: Carrier comparison

*Figure 3:* Carrier comparison. The duty ratios $\dA$, $\dB$, and $\dC$ are constant over the sampling period $\Ts$ (or, optionally, over the switching period $\Tsw = 2\Ts$). The carrier signal is the same for all three phases and varies between 0 and 1.
```

The zero-sequence voltage does not affect the phase currents if the neutral of the load is not connected. Therefore, the reference potential of the phase voltages can be freely chosen when computing the space vector of the converter output voltage. The converter voltage vector in stationary coordinates is

```{math}
:label: carrier_comparison

\ucs &= \frac{2}{3}\left(\uAn + \uBn\e^{\jj 2\pi/3} + \uCn\e^{\jj 4\pi/3}\right) \\
&= \frac{2}{3}\left(\uAN + \uBN\e^{\jj 2\pi/3} + \uCN\e^{\jj 4\pi/3}\right) \\
&= \underbrace{\frac{2}{3}\left(\qA + \qB\e^{\jj 2\pi/3} + \qC\e^{\jj 4\pi/3}\right)}_{\qcs}\udc
```

where $\qcs$ is the switching-state space vector.

```{note}
The carrier comparison is compatible with all standard pulse-width modulation (PWM) methods, such as space-vector PWM (see {class}`motulator.common.control.PWM`) and discontinuous PWM {cite}`Hol1994,Hav1999`.
```

## Switching-Cycle Averaging

If the switching ripple is not of interest in simulations, the carrier comparison can be replaced with zero-order hold (ZOH) of the duty ratios. In this case, the output voltage vector is

```{math}
    :label: switching_cycle_averaging

    \ucs = \underbrace{\frac{2}{3}\left(\dA + \dB\e^{\jj 2\pi/3} + \dC\e^{\jj 4\pi/3}\right)}_{\dcs}\udc
```

where $\dcs$ is the duty ratio space vector. This ZOH is the default option in most {doc}`/drive_examples/index` and {doc}`/grid_examples/index` examples.