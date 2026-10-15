# Sampled-Data Systems

Machine drives and grid converter systems are sampled-data systems, consisting of a continuous-time system and a discrete-time control system as well as the interfaces between them {cite}`Fra1997,Bus2015`. [Figure 1](fig:system) shows a generic example system. The same architecture is used in *motulator*: the continuous-time system model is simulated in the continuous-time domain while the discrete-time control system runs in the discrete-time domain. The default solver is the explicit Runge-Kutta method of order 5(4) from [scipy.integrate.solve_ivp](https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_ivp.html).

```{figure} ../figs/system.svg
---
name: fig:system
class: only-light
width: 100%
align: center
alt: Block diagram of a sampled-data system
---
*Figure 1:* Block diagram of a sampled-data system. Discrete signals and systems are shown in blue. Continuous signals and systems are shown in red.
```

```{figure} ../figs/system.svg
---
class: invert-colors-dark only-dark
width: 100%
align: center
alt: Block diagram of a sampled-data system
---
*Figure 1:* Block diagram of a sampled-data system. Discrete signals and systems are shown in blue. Continuous signals and systems are shown in red.
```

As mentioned, the physical components of a machine drive or a grid converter system are modeled as continuous-time systems. Such a system model comprises a power converter model along with other subsystem models, such as an electric machine model or a grid model. In addition to the inputs $\mathbf{q}(t)$ from the control system, the continuous-time system may have external continuous-time inputs $\mathbf{e}(t)$, such as a load torque or power fed to the DC bus. After the simulation, all continuous-time states $\mathbf{x}(t)$ are available for post-processing and plotting. In the {doc}`/drive_examples/index` and {doc}`/grid_examples/index` examples, the instances of continuous-time system model classes are named `mdl`.

A discrete-time control system (named `ctrl` in the examples) contains control algorithms, such as a speed controller and a current controller. The reference signals $\mathbf{r}(k)$ could contain, e.g., a speed reference of an electric machine or a power reference of a grid converter. The feedback signals $\mathbf{y}(k)$ typically contain at least the measured DC-bus voltage and converter phase currents.

By default, the sensors are ideal. Nonideal sensors can be modeled with the {class}`motulator.common.model.Sensor` class, which adds a gain error, an offset, and white Gaussian noise to the sampled signals. A sensor is a parameter of the subsystem whose signals it measures, e.g., `i_s_sensor` of a machine model or `u_dc_sensor` of a converter model.

Digital control systems typically have a computational delay of one sampling period, $N=1$. If desired, the value of $N$ can be changed in simulations. By default, the duty ratios are sampled and updated twice per carrier period (double update), so the sampling period is half the carrier period. Alternatively, single update, where the sampling period equals the carrier period, can be selected in the system model. The delay $N$ is then counted in half carrier periods: $N=2$ corresponds to the computational delay of one sampling period, while $N=1$ loads the duty ratios at the carrier extremum between the sampling instants, halving the delay. The control systems compensate for the delay of one sampling period. [Figure 1](fig:system) includes carrier comparison, which realizes pulse-width modulation (PWM). If the switching ripple is not of interest in simulations, the carrier comparison can be replaced with a zero-order hold (ZOH).

A system may also contain several converters, each driven by its own control system, e.g., a drive with an active front end. Such a system is modeled as a {class}`motulator.common.model.CompositeModel`, which combines the models of the converter systems sharing a subsystem, such as a DC bus. Each model keeps its own carrier comparison (or ZOH) and computational delay. The control systems are independent: they may have different sampling periods, and their sampling instants and carriers need not be synchronized (the first sampling instants are given to the {class}`motulator.common.model.Simulation` class). The simulation calls each control system at its own sampling instants and integrates the continuous-time system between the sampling and switching instants of all the converters.
