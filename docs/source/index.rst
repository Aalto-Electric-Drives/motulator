*motulator:* Motor Drive and Grid Converter Simulator in Python
===============================================================

This open-source software includes simulation models and controllers for electric machine drives and grid-connected converters.
The machine models include an induction machine, a synchronous reluctance machine, and a permanent-magnet synchronous machine.
Various models are provided for grid converter-related electrical subsystems such as an inductive-capacitive-inductive (LCL) filter connected to an inductive-resistive grid.

The system models are simulated in the continuous-time domain while the control methods run in discrete time.
The default solver is the explicit Runge-Kutta method of order 5(4) from `scipy.integrate.solve_ivp`_.
A number of control methods are provided as examples. The example methods aim to be simple yet feasible. 

.. _scipy.integrate.solve_ivp: https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_ivp.html

.. toctree::
   :titlesonly:
   :caption: Getting Started
   :name: getting_started
   :maxdepth: 1

   installation
   usage

.. toctree:: 
   :titlesonly:
   :caption: Examples
   :name: examples
   :maxdepth: 1

   drive_examples/index
   grid_examples/index

.. toctree::
   :titlesonly:
   :caption: System Models
   :name: models
   :maxdepth: 1

   model/system
   model/converters
   model/drive/index
   model/grid/index

.. toctree::
   :titlesonly:
   :caption: Design Notes on Control Methods
   :name: controllers
   :maxdepth: 1

   control/drive/index
   control/grid/index

.. toctree::
   :titlesonly:
   :caption: Class references
   :name: api
   :maxdepth: 1

   API Reference <autoapi/motulator/index>

.. rubric::
   Acknowledgement

This project has been sponsored by ABB Oy and by the Research Council of Finland *Centre of Excellence in High-Speed Electromechanical Energy Conversion Systems*. The example control methods included in this repository are based on published algorithms (available in textbooks and scientific articles). They do not present any proprietary control software.
