*motulator:* Motor Drive Simulator in Python
============================================

This open-source software includes simulation models for an induction machine, a synchronous reluctance machine, and a permanent-magnet synchronous machine. The machine models are simulated in the continuous-time domain while the control algorithms run in discrete time. The default solver is the explicit Runge-Kutta method of order 5(4) from `scipy.integrate.solve_ivp`_. Simple control algorithms are provided as examples. The example algorithms aim to be simple yet feasible. 

.. _scipy.integrate.solve_ivp: https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_ivp.html

.. toctree::
   :titlesonly:
   :caption: Getting Started
   :name: getting_started
   :maxdepth: 1

   installation
   usage
   auto_examples/index
   API Reference <autoapi/motulator/index>

.. toctree::
   :titlesonly:
   :caption: System Models
   :name: models
   :maxdepth: 2

   model/index

.. toctree::
   :titlesonly:
   :caption: Control Algorithms
   :name: controllers
   :maxdepth: 1

   control/common
   control/im
   control/sm

.. rubric::
   Acknowledgement

This project has been sponsored by ABB Oy and by the Academy of Finland *Centre of Excellence in High-Speed Electromechanical Energy Conversion Systems*. The example control methods included in this repository are based on published algorithms (available in textbooks and scientific articles). They do not present any proprietary control software.
