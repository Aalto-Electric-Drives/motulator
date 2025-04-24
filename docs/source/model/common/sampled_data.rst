Sampled-Data Systems
====================

Machine drives and grid converter systems are sampled-data systems, consisting of a continuous-time system and a discrete-time control system as well as the interfaces between them [#Fra1997]_, [#Bus2015]_. The figure below shows a generic example system. The same architecture is used in *motulator*: the continuous-time system model is simulated in the continuous-time domain while the discrete-time control system runs in the discrete-time domain. The default solver is the explicit Runge-Kutta method of order 5(4) from `scipy.integrate.solve_ivp`_.

.. _scipy.integrate.solve_ivp: https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_ivp.html

.. figure:: ../figs/system.svg
   :figclass: only-light
   :width: 100%
   :align: center
   :alt: Block diagram of a sampled-data system
   :target: .

   Block diagram of a sampled-data system. Discrete signals and systems are shown in blue, and continuous signals and systems are shown in red.

.. figure:: ../figs/system.svg
   :figclass: invert-colors-dark only-dark
   :width: 100%
   :align: center
   :alt: Block diagram of a sampled-data system
   :target: .

   Block diagram of a sampled-data system. Discrete signals and systems are shown in blue, and continuous signals and systems are shown in red.

As mentioned, the physical components of a machine drive or a grid converter system are modeled as continuous-time systems. Such a system model comprises a power converter model along with other subsystem models, such as an electric machine model or grid model. In addition to the inputs :math:`\boldsymbol{q}(t)` from the control system, the continuous-time system may have external continuous-time inputs :math:`\boldsymbol{e}(t)`, such as a load torque or power fed to the DC bus. After the simulation, all continuous-time states :math:`\boldsymbol{x}(t)` are available for post-processing and plotting. In :doc:`Drive Examples </drive_examples/index>` and :doc:`Grid Examples </grid_examples/index>`, the instances of continuous-time system model classes are named `mdl`.

A discrete-time control system (named `ctrl` in the examples) contains control algorithms, such as a speed controller and current controller. The reference signals :math:`\boldsymbol{r}(k)` could contain, e.g., a speed reference of an electric machine or a power reference of a grid converter. The feedback signals :math:`\boldsymbol{y}(k)` typically contain at least the measured DC-bus voltage and converter phase currents.

Digital control systems typically have a computational delay of one sampling period, :math:`N=1`. The PWM block shown in the figure models the carrier comparison. If the switching ripple is not of interest in simulations, the carrier comparison can be replaced with a zero-order hold (ZOH).


.. rubric:: References

.. [#Fra1997] Franklin, Powell, Workman, "Digital Control of Dynamic Systems," Menlo Park, CA, USA: Addison-Wesley, 1997

.. [#Bus2015] Buso, Mattavelli, "Digital Control in Power Electronics," 2nd ed., Morgan & Claypool, 2015, https://doi.org/10.2200/S00637ED1V01Y201503PEL007
