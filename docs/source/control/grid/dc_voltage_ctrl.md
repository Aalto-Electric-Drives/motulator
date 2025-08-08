# DC-Bus Voltage Control

A PI controller for the DC-bus voltage is implemented in the {class}`motulator.grid.control.DCBusVoltageController` class, whose base class is {class}`motulator.common.control.PIController`. In the following, the tuning of the DC-bus voltage controller is considered in the continuous-time domain.

## Controller Structure

In order to have a linear closed-loop system, it is convenient to use the DC-bus energy as the controlled variable, {cite}`Hur2001`

```{math}
:label: dc_cap_energy

\hatWdc = \frac{1}{2}\hatCdc \udc^2 \qquad
\Wdcref = \frac{1}{2}\hatCdc (\udcref)^2
```

where $\hatCdc$ is the DC-bus capacitance estimate, $\udc$ is the measured DC-bus voltage, and $\udcref$ is the DC-bus voltage reference. Using these variables, the output power reference $p_\mathrm{c}^\mathrm{ref}$ for the converter is obtained from a PI controller,

```{math}
:label: dc_voltage_ctrl

p_\mathrm{c}^\mathrm{ref} = -\kp (\Wdcref - \hatWdc) - \int \ki (\Wdcref - \hatWdc) \D t
```

where $\kp$ is the proportional gain and $\ki$ is the integral gain. The negative signs appear since the positive converter output power decreases the DC-bus capacitor energy, see {eq}`capacitor`.

## Closed-Loop System

For simplicity, the capacitance estimate being accurate, $\hatCdc = \Cdc$. Furthermore, the converter is assumed to be lossless and its output power control ideal. The DC-bus energy balance is

```{math}
:label: capacitor

\frac{\D \Wdc}{\D t} = p_\mathrm{dc} - p_\mathrm{c}
```

where $p_\mathrm{dc}$ is the external power fed to the DC bus, $p_\mathrm{c}$ is the converter output power, and $\Wdc = (\Cdc/2) \udc^2$ is the energy stored in the DC-bus capacitor. The power $p_\mathrm{dc}$ is considered as an unknown disturbance.

In the Laplace domain, the closed-loop system resulting from {eq}`dc_voltage_ctrl` and {eq}`capacitor` is given by

```{math}
:label: dc_voltage_ctrl_closed_loop

\Wdc(s) = \frac{\kp s + \ki}{s^2 + \kp s + \ki} \Wdcref(s) + \frac{s}{s^2 + \kp s + \ki} p_\mathrm{dc}(s)
```

where it can be seen that $\Wdc = \Wdcref$ holds in the steady state. Furthermore, it can be shown that also $\udc = \udcref$ holds in the steady state (independently of the errors in the capacitance estimate $\hat{C}$, since both reference and measured values in {eq}`dc_cap_energy` are scaled by the same estimate).

## Gain Selection

Based on {eq}`dc_voltage_ctrl_closed_loop`, the gain selection

```{math}
:label: dc_voltage_ctrl_gain_selection

\kp = 2\alphadc \qquad
\ki = \alphadc^2
```

results in the double real pole at $s = -\alphadc$. The closed-loop bandwidth is approximately $\alphadc$.
