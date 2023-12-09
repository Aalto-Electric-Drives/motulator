*motulator:* Motor Drive Simulator in Python
============================================

This open-source software includes simulation models for an induction machine, a synchronous reluctance machine, and a permanent-magnet synchronous machine. The machine models are simulated in the continuous-time domain while the control methods run in discrete time. The default solver is the explicit Runge-Kutta method of order 5(4) from `scipy.integrate.solve_ivp`_. A number of control methods are provided as examples. The example methods aim to be simple yet feasible. 

.. _scipy.integrate.solve_ivp: https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_ivp.html

.. toctree::
   :titlesonly:
   :caption: Getting Started
   :name: getting_started
   :maxdepth: 1

   installation
   usage
   API Reference <autoapi/motulator/index>

.. toctree::
   :titlesonly:
   :caption: System Models
   :name: models
   :maxdepth: 2

   model/index

.. toctree::
   :titlesonly:
   :caption: Control Methods
   :name: controllers
   :maxdepth: 1

   control/design_notes
   auto_examples/index

.. rubric::
   Acknowledgement

This project has been sponsored by ABB Oy and by the Research Council of Finland *Centre of Excellence in High-Speed Electromechanical Energy Conversion Systems*. The example control methods included in this repository are based on published algorithms (available in textbooks and scientific articles). They do not present any proprietary control software.
