# *motulator:* Motor Drive and Grid Converter Simulator in Python

This open-source software includes simulation models and control methods for electric machine drives and grid converter systems. The machine models include an induction machine, a synchronous reluctance machine, and a permanent-magnet synchronous machine. For modeling grid converter systems, various subsystem models are provided, such as an LCL filter connected to an inductive-resistive grid.

The system models are simulated in the continuous-time domain while the control methods run in discrete time. The default solver is the explicit Runge-Kutta method of order 5(4) from [scipy.integrate.solve_ivp](https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_ivp.html). A number of control methods are provided as examples. The example methods aim to be simple yet feasible.

```{toctree}
:titlesonly:
:caption: Getting Started
:name: getting_started
:maxdepth: 1

installation
usage
contributing
```

```{toctree}
:titlesonly:
:caption: Examples
:name: examples
:maxdepth: 1

drive_examples/index
grid_examples/index
```

```{toctree}
:titlesonly:
:caption: System Models
:name: models
:maxdepth: 1

model/common/index
model/drive/index
model/grid/index
```

```{toctree}
:titlesonly:
:caption: Control Methods
:name: controllers
:maxdepth: 1

control/common/index
control/drive/index
control/grid/index
```

```{toctree}
:titlesonly:
:caption: Reference
:name: reference
:maxdepth: 1

API <autoapi/motulator/index>
Bibliography <references>
```

```{rubric} Acknowledgement
```

This project has been sponsored by ABB Oy and by the Research Council of Finland *Centre of Excellence in High-Speed Electromechanical Energy Conversion Systems*. The example control methods included in this repository are based on published algorithms (available in textbooks and scientific articles). They do not present any proprietary control software.
