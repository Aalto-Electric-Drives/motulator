# Sampled-Data Systems

Machine drives and grid converter systems are sampled-data systems, consisting of a continuous-time system and a discrete-time control system as well as the interfaces between them {cite}`Fra1997,Bus2015`. [Figure 1](fig:system) shows a generic example system. The same architecture is used in *motulator*: the continuous-time system model is simulated in the continuous-time domain while the discrete-time control system runs in the discrete-time domain. The default solver is the explicit Runge-Kutta method of order 5(4) from [scipy.integrate.solve_ivp](https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_ivp.html).

```{figure} ../figs/system.svg
    :name: fig:system
    :class: only-light
    :width: 100%
    :align: center
    :alt: Block diagram of a sampled-data system

*Figure 1:* Block diagram of a sampled-data system. Discrete signals and systems are shown in blue. Continuous signals and systems are shown in red.
```

```{figure} ../figs/system.svg
    :class: invert-colors-dark only-dark
    :width: 100%
    :align: center
    :alt: Block diagram of a sampled-data system

*Figure 1:* Block diagram of a sampled-data system. Discrete signals and systems are shown in blue. Continuous signals and systems are shown in red.
```

As mentioned, the physical components of a machine drive or a grid converter system are modeled as continuous-time systems. Such a system model comprises a power converter model along with other subsystem models, such as an electric machine model or a grid model. In addition to the inputs $\mathbf{q}(t)$ from the control system, the continuous-time system may have external continuous-time inputs $\mathbf{e}(t)$, such as a load torque or power fed to the DC bus. After the simulation, all continuous-time states $\mathbf{x}(t)$ are available for post-processing and plotting. In the {doc}`/drive_examples/index` and {doc}`/grid_examples/index` examples, the instances of continuous-time system model classes are named `mdl`.

A discrete-time control system (named `ctrl` in the examples) contains control algorithms, such as a speed controller and a current controller. The reference signals $\mathbf{r}(k)$ could contain, e.g., a speed reference of an electric machine or a power reference of a grid converter. The feedback signals $\mathbf{y}(k)$ typically contain at least the measured DC-bus voltage and converter phase currents.

Digital control systems typically have a computational delay of one sampling period, $N=1$. If desired, the value of $N$ can be changed in simulations. [Figure 1](fig:system) includes carrier comparison, which realizes pulse-width modulation (PWM). If the switching ripple is not of interest in simulations, the carrier comparison can be replaced with a zero-order hold (ZOH).
