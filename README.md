# *motulator*
*motulator*: Motor Drive Simulator

Introduction
------------
This open-source software includes simulation models for an induction motor, a synchronous reluctance motor, and a permanent-magnet synchronous motor. The motor models are simulated in the continuous-time domain while the control algorithms run in discrete time. The default solver is the explicit Runge-Kutta method of order 5(4) from scipy.integrate.solve_ivp. Simple control algorithms are provided as examples. 

<img src="pwm.png" alt="PWM waveforms" width="320"/><img src="pmsm.png" alt="Speed and torque waveforms" width="320"/>

Usage
-----
The main folder includes the example script `sim.py` to run the simulation. The folder `config` contains example configuration files that can be modified or used as templates for new ones. There are separate config files for the drive system and for its controller. For example, pulse-width modulation (PWM) can be enabled in the drive system config file. The control system can be configured to operate in a sensorless or sensored mode. More detailed documentation is available here:

https://aalto-electric-drives.github.io/motulator/

Acknowledgement
---------------
This project has been sponsored by ABB Oy. The example control methods included in this repository are based on published algorithms (available in textbooks and scientific articles). They do not present any proprietary control software.
