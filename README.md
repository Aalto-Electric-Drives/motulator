# *motulator:* Motor Drive Simulator in Python
<!-- ALL-CONTRIBUTORS-BADGE:START - Do not remove or modify this section -->
[![All Contributors](https://img.shields.io/badge/all_contributors-1-orange.svg?style=flat-square)](#contributors-)
<!-- ALL-CONTRIBUTORS-BADGE:END -->

[![Build Status](https://github.com/Aalto-Electric-Drives/motulator/actions/workflows/update_gh-pages.yml/badge.svg)](https://github.com/Aalto-Electric-Drives/motulator/actions/workflows/update_gh-pages.yml)
[![License](https://img.shields.io/github/license/mashape/apistatus)](https://github.com/Aalto-Electric-Drives/motulator/blob/main/LICENSE)
[![PyPI version shields.io](https://img.shields.io/pypi/v/motulator.svg)](https://pypi.org/project/motulator/)
[![All Contributors](https://img.shields.io/badge/all_contributors-13-orange.svg?style=flat-square)](#contributors)

Introduction
------------
This open-source software includes simulation models for an induction motor, a synchronous reluctance motor, and a permanent-magnet synchronous motor. The motor models are simulated in the continuous-time domain while the control algorithms run in discrete time. The default solver is the explicit Runge-Kutta method of order 5(4) from scipy.integrate.solve_ivp. Simple control algorithms are provided as examples. The documentation is available here:

https://aalto-electric-drives.github.io/motulator/

Installation
------------
This software can be installed using pip: 

```bash
pip install motulator
```
Alternatively, the repository can be cloned.

Usage
-----
The following example shows how to create a continuous-time system model, a discrete-time controller, and a simulation object:
```python3
import motulator as mt

# Continuous-time model for the drive system
motor = mt.InductionMotor()     # Motor model
mech = mt.Mechanics()           # Mechanics model
conv = mt.Inverter() 	        # Converter model
mdl = mt.InductionMotorDrive(motor, mech, conv)

# Discrete-time controller 
pars = mt.InductionMotorVectorCtrlPars() 	# Dataclass of control parameters
ctrl = mt.InductionMotorVectorCtrl(pars) 	# Sensorless controller

# Create a simulation object, simulate, and plot example figures
sim = mt.Simulation(mdl, ctrl)
sim.simulate()
mt.plot(sim)
```
This example applies the default settings. The drive system, controller, reference sequences etc. are easy to configure. The example scripts and Jupyter notebooks can be downloaded here:

https://aalto-electric-drives.github.io/motulator/auto_examples/index.html

New system models and controllers can be developed using the existing ones as templates.

Contributing
------------
If you'd like to help us develop motulator, please have a look at these [guidelines](https://github.com/Aalto-Electric-Drives/motulator/blob/main/CONTRIBUTING.md) first.

Contributors
------------

<!-- ALL-CONTRIBUTORS-LIST:START - Do not remove or modify this section -->
<!-- prettier-ignore-start -->
<!-- markdownlint-disable -->
<table>
  <tbody>
    <tr>
      <td align="center"><a href="https://github.com/lauritapio"><img src="https://avatars.githubusercontent.com/u/85596019?v=4?s=100" width="100px;" alt="Lauri Tiitinen"/><br /><sub><b>Lauri Tiitinen</b></sub></a><br /><a href="https://github.com/Aalto-Electric-Drives/motulator/commits?author=lauritapio" title="Code">💻</a></td>
    </tr>
  </tbody>
</table>

<!-- markdownlint-restore -->
<!-- prettier-ignore-end -->

<!-- ALL-CONTRIBUTORS-LIST:END -->

Acknowledgement
---------------
This project has been sponsored by ABB Oy and by the Academy of Finland *Centre of Excellence in High-Speed Electromechanical Energy Conversion Systems*. The example control methods included in this repository are based on published algorithms (available in textbooks and scientific articles). They do not present any proprietary control software.

## Contributors ✨

Thanks goes to these wonderful people ([emoji key](https://allcontributors.org/docs/en/emoji-key)):

<!-- ALL-CONTRIBUTORS-LIST:START - Do not remove or modify this section -->
<!-- prettier-ignore-start -->
<!-- markdownlint-disable -->
<!-- markdownlint-restore -->
<!-- prettier-ignore-end -->
<!-- ALL-CONTRIBUTORS-LIST:END -->

This project follows the [all-contributors](https://github.com/all-contributors/all-contributors) specification. Contributions of any kind welcome!