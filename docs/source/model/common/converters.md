# Voltage-Source Converter

## Model

[Figure 1](fig:inverter) shows a three-phase two-level voltage-source converter, where $\udc$ is the DC-bus voltage, $\idc$ is the external DC current, and $\Cdc$ is the DC-bus capacitance. The converter can operate both as an inverter and a rectifier, depending on the direction of the power flow. The model is provided in the {class}`motulator.common.model.VoltageSourceConverter` class. Without the transistors, the converter is a six-pulse diode bridge, see [Diode Bridge](sec:diode_bridge).

```{figure} ../figs/inverter.svg
---
name: fig:inverter
class: only-light
width: 100%
align: center
alt: Three-phase two-level voltage-source converter
---
*Figure 1:* Three-phase two-level voltage-source converter. The negative potential of the DC bus is marked with N and the output terminals with a, b, and c.
```

```{figure} ../figs/inverter.svg
---
class: invert-colors-dark only-dark
width: 100%
align: center
alt: Three-phase two-level voltage-source converter
---
*Figure 1:* Three-phase two-level voltage-source converter. The negative potential of the DC bus is marked with N and the output terminals with a, b, and c.
```

Assuming ideal transistors and diodes, the converter can be modeled with the equivalent circuit shown in [Figure 2](fig:pwm_inverter), in which the legs are modeled as bi-positional switches. Each changeover switch is connected to either negative or positive potential of the DC bus, and the switching phenomena are assumed to be infinitely fast. The state of each switch is defined using the switching state, which, using phase $a$ as an example, is $\qA = 0$ when the switch is connected to the negative potential and $\qA = 1$ when the switch is connected to the positive potential.

```{figure} ../figs/pwm_inverter.svg
---
name: fig:pwm_inverter
class: only-light
width: 100%
align: center
alt: Voltage-source converter and carrier comparison
---
*Figure 2:* Equivalent circuit of a three-phase voltage-source converter, connected to a generic three-phase load. The neutral point of the load is marked with n. In this example, the positions of the bi-positional switches correspond to the instantaneous switching states $\qA = 1$, $\qB = 0$, and $\qC = 0$.
```

```{figure} ../figs/pwm_inverter.svg
---
class: invert-colors-dark only-dark
width: 100%
align: center
alt: Voltage-source converter and carrier comparison
---
*Figure 2:* Equivalent circuit of a three-phase voltage-source converter, connected to a generic three-phase load. The neutral point of the load is marked with n. In this example, the positions of the bi-positional switches correspond to the instantaneous switching states $\qA = 1$, $\qB = 0$, and $\qC = 0$.
```

By default, the DC-bus voltage is constant, i.e., the DC-bus capacitor is replaced with a constant DC voltage source. Alternatively, the converter can be fed from the DC-bus model in the {class}`motulator.common.model.DCBus` class, which is given to the model of the converter. If the DC bus is fed from an external current source $\idc$, the DC-bus dynamics are

```{math}
---
label: DC_bus_model
---
    \Cdc \frac{\D \udc}{\D t} = \idc - \idc'
```

where the converter-side DC current depends on the phase currents $\iA$, $\iB$, and $\iC$ as

```{math}
---
label: DC_current
---
    \idc' = \qA \iA + \qB \iB + \qC \iC
```

The DC bus can also be shared by several converters, e.g., by the grid-side and machine-side converters of a drive with an active front end. In this case, the DC bus is given to the models of all the converters, and the DC currents $\idc'$ of all the converters are subtracted from the external current $\idc$. The models sharing the DC bus are combined using the {class}`motulator.common.model.CompositeModel` class, see {doc}`/drive_examples/grid_connection/plot_2kw_ipmsm_afe_cvc`. The DC bus can also be fed from a diode bridge, see [Diode Bridge](sec:diode_bridge).

## Gate Blocking

The control system can block the gates of the converter, see `set_enable` of {class}`motulator.common.control.ControlSystem`:

```python
ctrl.set_enable(lambda t: t >= 0.05)  # Gates blocked before 0.05 s
```

While the gates are blocked, all the switches are open, and the antiparallel diodes conduct according to the instantaneous phase currents. A leg carrying current is clamped to the negative rail if the current flows out of the converter and to the positive rail otherwise. A leg without current floats at the voltage that keeps its current at zero, as long as this voltage is between the rails; otherwise, its diode starts to conduct. Hence, a spinning machine coasts at open circuit if its voltage is below the DC-bus voltage, and it feeds the DC bus through the diodes otherwise, as does the grid when the DC bus is charged at the start-up. The threshold voltage $u_\mathrm{th}$ is included in the diode drops. The floating leg voltages are computed from the AC port of the load, i.e., the relation $\mathrm{d}\boldsymbol{i}/\mathrm{d}t = \mathbf{Y}(\boldsymbol{u} - \boldsymbol{e})$ between the terminal current and voltage, which the machine and filter models provide, see {class}`motulator.common.model.ACPort`. The machine models with core losses do not provide the port.

The switching state `q_c_ab` is zero while the gates are blocked, and the converter voltage `u_c_ab` is saved at the solution points of the solver. The gate enable is available as `enabled` in the converter time series and in the reference signals of the control system. See {doc}`/drive_examples/current_vector/plot_2kw_ipmsm_blocked_cvc` and {doc}`/grid_examples/grid_following/plot_10kva_startup_gfl` for examples.

(sec:diode_bridge)=

## Diode Bridge

A six-pulse diode bridge is a converter without the transistors, i.e., with permanently blocked gates. It is modeled in the {class}`motulator.common.model.DiodeBridge` class. The diodes conduct as described above, so the commutation of the current between the legs, including the overlap of the conduction intervals, follows from the inductance of the load, e.g., of an AC filter and the grid. A diode bridge connected to the grid is modeled with the {class}`motulator.grid.model.GridConverterSystem` class, see [Figure 3](fig:diode_bridge). The model can share its DC bus with other models, e.g., with a drive, see {doc}`/drive_examples/grid_connection/plot_2kw_im_diode_vhz`.

The bridge can also feed a DC load through a DC choke instead of a DC bus. The load then provides its DC port, i.e., the relation $u = e + Ri + L\,\mathrm{d}i/\mathrm{d}t$ between the DC current and the voltage, see {class}`motulator.common.model.DCPort`, and the bridge solves its DC voltage together with the leg voltages. Furthermore, the devices of the legs can be gated, which models thyristors: a floating leg is clamped to a rail only by a gated device, and a conducting device turns off when its current reverses. These capabilities are used by packages extending *motulator*, see {doc}`/related_packages`.

```{figure} ../figs/diode_bridge.svg
---
name: fig:diode_bridge
class: only-light
width: 100%
align: center
alt: Six-pulse diode bridge supplied from the grid through AC inductors
---
*Figure 3:* Six-pulse diode bridge supplied from the grid through the inductors of an L filter. The DC-side terminals correspond to those of the converter in [Figure 1](fig:inverter).
```

```{figure} ../figs/diode_bridge.svg
---
class: invert-colors-dark only-dark
width: 100%
align: center
alt: Six-pulse diode bridge supplied from the grid through AC inductors
---
*Figure 3:* Six-pulse diode bridge supplied from the grid through the inductors of an L filter. The DC-side terminals correspond to those of the converter in [Figure 1](fig:inverter).
```

## Carrier Comparison

In pulse-width modulation (PWM), carrier comparison is commonly used to generate instantaneous switching state signals $\qA$, $\qB$, and $\qC$ from duty ratios $\dA$, $\dB$, and $\dC$. The duty ratios are continuous signals in the range [0, 1] while the switching states are either 0 or 1.

[Figure 4](fig:carrier_comparison) shows the principle of carrier comparison. The logic shown in the figure is implemented in the {class}`motulator.common.model.CarrierComparison` class, where the switching instants are explicitly computed in the beginning of each half carrier period (instead of searching for zero crossings), allowing faster simulations. The sampling period $\Ts$ is returned by the control method, and it does not need to be constant. The carrier comparison is given to the converter model as its modulator, e.g., `VoltageSourceConverter(u_dc, CarrierComparison())`. The modulator also includes the computational delay of the control system, one sampling period by default.

```{figure} ../figs/carrier_comparison.svg
---
name: fig:carrier_comparison
class: only-light
width: 100%
align: center
alt: Carrier comparison
---
*Figure 4:* Carrier comparison. The duty ratios $\dA$, $\dB$, and $\dC$ are constant over the sampling period $\Ts$. The carrier signal is the same for all three phases and varies between 0 and 1.
```

```{figure} ../figs/carrier_comparison.svg
---
class: invert-colors-dark only-dark
width: 100%
align: center
alt: Carrier comparison
---
*Figure 4:* Carrier comparison. The duty ratios $\dA$, $\dB$, and $\dC$ are constant over the sampling period $\Ts$. The carrier signal is the same for all three phases and varies between 0 and 1.
```

The modulator parameter `t_d` delays the turn-on of each switch. During this dead time, both switches of the leg are off, and the converter resolves the leg state from the phase-current direction. The blanking interval may continue to the next sampling period. If the switching command reverses before a delayed turn-on, the turn-on is canceled. Transitions between the clamped duty ratios 0 and 1 also include the dead time. The converter parameter `u_th` models the threshold voltage of the power devices, assumed equal for the switches and the diodes, which reduces each phase voltage by `u_th*sign(i)`, where the current direction is determined at the switching instants, as during the blanking.

The zero-sequence voltage does not affect the phase currents if the neutral of the load is not connected. Therefore, the reference potential of the phase voltages can be freely chosen when computing the space vector of the converter output voltage. The converter voltage vector in stationary coordinates is

```{math}
---
label: carrier_comparison
---
\ucs &= \frac{2}{3}\left(\uAn + \uBn\e^{\jj 2\pi/3} + \uCn\e^{\jj 4\pi/3}\right) \\
&= \frac{2}{3}\left(\uAN + \uBN\e^{\jj 2\pi/3} + \uCN\e^{\jj 4\pi/3}\right) \\
&= \underbrace{\frac{2}{3}\left(\qA + \qB\e^{\jj 2\pi/3} + \qC\e^{\jj 4\pi/3}\right)}_{\qcs}\udc
```

where $\qcs$ is the switching-state space vector.

```{note}
The carrier comparison is compatible with all standard pulse-width modulation (PWM) methods, such as space-vector PWM and discontinuous PWM {cite}`Hol1994,Hav1999` (see {class}`motulator.common.control.PWM`).
```

## Switching-Cycle Averaging

If the switching ripple is not of interest in simulations, the carrier comparison can be replaced with zero-order hold (ZOH) of the duty ratios. In this case, the output voltage vector is

```{math}
---
label: switching_cycle_averaging
---
    \ucs = \underbrace{\frac{2}{3}\left(\dA + \dB\e^{\jj 2\pi/3} + \dC\e^{\jj 4\pi/3}\right)}_{\dcs}\udc
```

where $\dcs$ is the duty ratio space vector. The {class}`motulator.common.model.ZOH` modulator is the default of the converter models, used in most {doc}`/drive_examples/index` and {doc}`/grid_examples/index` examples.

With the dead time, the ZOH uses the switching-cycle averaged on-times of the upper and lower switches, where pulses shorter than `t_d` are suppressed. The converter resolves the remaining blanked fraction using the current-direction function `sign`. The {class}`motulator.common.control.InverterNonlinearity` class gives the corresponding duty-ratio error for the control system, including the threshold voltage. The transients due to changes in the duty ratios are only modeled by carrier comparison.
