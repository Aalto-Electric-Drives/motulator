*motulator*: Motor Drive Simulator
==================================

.. toctree::
   :maxdepth: 1

   usage
   tutorial

This open-source software includes simulation models for an induction motor, a synchronous reluctance motor, and a permanent-magnet synchronous motor. The motor models are simulated in the continuous-time domain while the control algorithms run in discrete time. The default solver is the explicit Runge-Kutta method of order 5(4) from `scipy.integrate.solve_ivp`_. Simple control algorithms are provided as examples. The example algorithms aim to be simple yet feasible, and they have not been optimized. 

.. _scipy.integrate.solve_ivp: https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_ivp.html

.. image:: pwm.png
   :width: 320 px
   :alt: PWM waveforms

.. image:: pmsm.png
   :width: 320 px
   :alt: Speed and torque waveforms

Acknowledgement
---------------

This open-source project has been sponsored by ABB Oy. The example control methods included in this repository are based on published algorithms (available in textbooks and scientific articles). They do not present any proprietary control software.
