# *motulator*
Open-Source Simulator for Motor Drives and Power Converters

Introduction
------------
This software includes simulation models for an induction motor, a synchronous reluctance motor, and a permanent-magnet synchronous motor. Furthermore, some simple control algorithms are included as examples. The motor models are simulated in the continuous-time domain while the control algorithms run in discrete time. The default solver is the explicit Runge-Kutta method of order 5(4) from scipy.integrate.solve_ivp.

Some interfaces may change in the later versions. The example control algorithms aim to be simple yet feasible, and they have not been optimized.

<img src="pwm.png" alt="PWM waveforms" width="320"/><img src="pmsm.png" alt="Speed and torque waveforms" width="320"/>

Usage
-----
The main folder includes the example script `sim.py` to run the simulation. The folder `config` contains example configuration files that can be modified or used as templates for new ones. There are separate config files for the drive system and for its controller. For example, pulse-width modulation (PWM) can be enabled in the drive system config file. The control system can be configured to operate in a sensorless or sensored mode. More detailed documentation is available here:

https://aalto-electric-drives.github.io/motulator/

The main folder includes the example script `sim.py` to run the simulation. The folder `config` contains example configuration files that can be modified or used as templates for new ones. There are separate config files for the drive system and for its controller. For example, pulse-width modulation (PWM) can be enabled in the drive system config file. The control system can be configured to operate in a sensorless or sensored mode. 

Acknowledgement
---------------
This open-source project has been sponsored by ABB Oy. The example control methods included in this repository are based on published algorithms (available in textbooks and scientific articles). They do not present any proprietary control software.
