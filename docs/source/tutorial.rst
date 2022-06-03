Tutorial
--------
Motulator simulation tool is based on the control of an induction- or synchronous motor drive. Motulator includes scalar control (or Volts-per-Hertz-control) and rotor-flux-oriented vector control. In order to build the link between the continuous-time design and discrete-time implementation, all the control algorithms are implemented in the discrete-time domain based on forward Euler approximation.

The config files starting with mdl describe the motor data according to the inverse-gamma model of the induction motor...

.. image:: Inverse-gamma_model.png
   :scale: 100%
   :alt: alternate text
   :target: .