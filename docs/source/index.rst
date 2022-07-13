*motulator:* Motor Drive Simulator in Python
============================================

This open-source software includes simulation models for an induction motor, a synchronous reluctance motor, and a permanent-magnet synchronous motor. The motor models are simulated in the continuous-time domain while the control algorithms run in discrete time. The default solver is the explicit Runge-Kutta method of order 5(4) from `scipy.integrate.solve_ivp`_. Simple control algorithms are provided as examples. The example algorithms aim to be simple yet feasible. 

.. _scipy.integrate.solve_ivp: https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_ivp.html

.. toctree::
   :caption: Getting Started
   :maxdepth: 1

   installation
   usage
   auto_examples/index
   autoapi/index

.. toctree::
   :titlesonly:
   :caption: System Models
   :name: models
   :maxdepth: 2

   system
   induction_motor
   synchronous_motor
   mechanics
   converters

.. toctree::
   :caption: Contributing
   :maxdepth: 1

   contributing

Acknowledgement
---------------

This open-source project has been sponsored by ABB Oy. The example control methods included in this repository are based on published algorithms (available in textbooks and scientific articles). They do not present any proprietary control software.
