# *motulator*
*motulator*: Motor Drive Simulator in Python

Introduction
------------
This open-source software includes simulation models for an induction motor, a synchronous reluctance motor, and a permanent-magnet synchronous motor. The motor models are simulated in the continuous-time domain while the control algorithms run in discrete time. The default solver is the explicit Runge-Kutta method of order 5(4) from scipy.integrate.solve_ivp. Simple control algorithms are provided as examples. 

<img src="pwm.png" alt="PWM waveforms" width="320"/><img src="pmsm.png" alt="Speed and torque waveforms" width="320"/>

Usage
-----
An example script `main.py` exemplifies how to configure and simulate different models. More detailed documentation will be provided quite soon...

Acknowledgement
---------------
This project has been sponsored by ABB Oy. The example control methods included in this repository are based on published algorithms (available in textbooks and scientific articles). They do not present any proprietary control software.
