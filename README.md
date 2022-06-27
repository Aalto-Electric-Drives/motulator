# *motulator*
*motulator:* Motor Drive Simulator in Python

Introduction
------------
This open-source software includes simulation models for an induction motor, a synchronous reluctance motor, and a permanent-magnet synchronous motor. The motor models are simulated in the continuous-time domain while the control algorithms run in discrete time. The default solver is the explicit Runge-Kutta method of order 5(4) from scipy.integrate.solve_ivp. Simple control algorithms are provided as examples. 

<img src="pwm.png" alt="PWM waveforms" width="320"/><img src="pmsm.png" alt="Speed and torque waveforms" width="320"/>

Installation
------------
This software can be installed using pip: 

```bash
pip install motulator
```
Alternatively, the repository can be cloned. Running `__init__.py` in the folder `motulator` allows importing the package.

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
This example applies the default settings. However, the drive system, controller, reference sequences etc. are easy to configure, see the folder `examples` for example scripts. New system models and controllers can be developed using the existing ones as templates. More features will be added later.

Acknowledgement
---------------
This project has been sponsored by ABB Oy. The example control methods included in this repository are based on published algorithms (available in textbooks and scientific articles). They do not present any proprietary control software.

