Common
======

Concepts and models common to machine drives and grid converter systems are described in this section.

Sampled-Data Systems
--------------------

Machine drives and grid converter systems are sampled-data systems, consisting of a continuous-time system and a discrete-time control system as well as the interfaces between them [#Fra1997]_, [#Bus2015]_. The figure below shows a generic example system. The same architecture is used in *motulator*: the continuous-time system model is simulated in the continuous-time domain while the discrete-time control system runs in the discrete-time domain. The default solver is the explicit Runge-Kutta method of order 5(4) from `scipy.integrate.solve_ivp`_.

.. _scipy.integrate.solve_ivp: https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_ivp.html

.. figure:: figs/system.svg
   :width: 100%
   :align: center
   :alt: Block diagram of a sampled-data system
   :target: .

   Block diagram of a sampled-data system. Discrete signals and systems are shown in blue, and continuous signals and systems are shown in red.

As mentioned, the physical components of a machine drive or a grid converter system are modeled as continuous-time systems. Such a system model comprises a power converter model along with other subsystem models, such as an electric machine model or grid model. In addition to the inputs :math:`\boldsymbol{q}(t)` from the control system, the continuous-time system may have external continuous-time inputs :math:`\boldsymbol{e}(t)`, such as a load torque or power fed to the DC bus. After the simulation, all continuous-time states :math:`\boldsymbol{x}(t)` are available for post-processing and plotting. In the :doc:`/drive_examples/index` and :doc:`/grid_examples/index` examples, the instances of continuous-time system model classes are named `mdl`.

A discrete-time control system (named `ctrl` in the examples) contains control algorithms, such as a speed controller and current controller. The reference signals :math:`\boldsymbol{r}(k)` could contain, e.g., a speed reference of an electric machine or a power reference of a grid converter. The feedback signals :math:`\boldsymbol{y}(k)` typically contain at least the measured DC-bus voltage and converter phase currents.

Digital control systems typically have a computational delay of one sampling period, :math:`N=1`. The PWM block shown in the figure models the carrier comparison. If the switching ripple is not of interest in simulations, the carrier comparison can be replaced with a zero-order hold (ZOH).

Space Vectors
-------------

The system models in *motulator* apply peak-valued complex space vectors, marked with boldface in the following equations [#Hin2024]_. As an example, the space vector of the converter current is

.. math::
	\boldsymbol{i}^\mathrm{s}_\mathrm{c} = \frac{2}{3}\left(i_\mathrm{a} + i_\mathrm{b}\mathrm{e}^{\mathrm{j}2\pi/3} + i_\mathrm{c}\mathrm{e}^{\mathrm{j} 4\pi/3}\right)
   :label: space_vector

where :math:`i_\mathrm{a}`, :math:`i_\mathrm{b}`, and :math:`i_\mathrm{c}` are the phase currents, which may vary freely in time. In our notation, the subscript c refers to the converter-side AC quantities and the superscript s refers to the stationary coordinates. In addition, the subscript c is also used to denote the phase :math:`c` quantities, but the meaning should be clear from the context.

The space vector does not include the zero-sequence component, which is defined as

.. math::
	i_0 = \frac{1}{3}\left(i_\mathrm{a} + i_\mathrm{b} + i_\mathrm{c}\right)
   :label: zero_sequence

Even though the zero-sequence voltage exists at the output of typical converters, there is no path for the zero-sequence current to flow (i.e., :math:`i_0 = 0`), if the three-phase system is delta-connected or its star point is not connected. Consequently, the zero-sequence voltage cannot produce power or torque.

The space vector transformation in :eq:`space_vector` is implemented in the function :func:`motulator.common.utils.abc2complex` and its inverse transformation in the function :func:`motulator.common.utils.complex2abc`.

Voltage-Source Converter
------------------------

The figure below shows a three-phase two-level voltage-source converter, where :math:`u_\mathrm{dc}` is the DC-bus voltage, :math:`i_\mathrm{dc}` is the external DC current, and :math:`C_\mathrm{dc}` is the DC-bus capacitance. This converter can operate both as an inverter and a rectifier, depending on the direction of the power flow.

.. figure:: figs/inverter.svg
   :width: 100%
   :align: center
   :alt: Three-phase two-level voltage-source converter
   :target: .

   Three-phase two-level voltage-source converter. The negative potential of the DC bus is marked with N and the output terminals with a, b, and c.

Assuming ideal transistors and diodes, the converter can be modeled with the equivalent circuit shown in the figure below, in which the legs are modeled as bi-positional switches. Each changeover switch is connected to either negative or positive potential of the DC bus, and the switching phenomena are assumed to be infinitely fast. The state of each switch is defined using the switching state, which, using phase :math:`a` as an example, is :math:`q_\mathrm{a} = 0` when the switch is connected to the negative potential and
:math:`q_\mathrm{a} = 1` when the switch is connected to the positive potential.

.. figure:: figs/pwm_inverter.svg
   :width: 100%
   :align: center
   :alt: Voltage-source converter and carrier comparison
   :target: .

   Equivalent circuit of a three-phase voltage-source converter, connected to a generic three-phase load. The neutral point of the load is marked with n. In this example, the positions of the bi-positional switches correspond to the instantaneous switching states :math:`q_\mathrm{a} = 1`, :math:`q_\mathrm{b} = 0`, and :math:`q_\mathrm{c}=0`.

By default, the DC-bus voltage is constant, i.e., the DC-bus capacitor is replaced with a constant DC voltage source. Alternatively, if the DC bus is fed from an external current source :math:`i_\mathrm{dc}`, the DC-bus dynamics are modeled as

.. math::
   C_\mathrm{dc}\frac{\mathrm{d}u_\mathrm{dc}}{\mathrm{d} t} = i_\mathrm{dc} - i'_\mathrm{dc}
   :label: DC_bus_model

where the converter-side DC current depends on the phase currents :math:`i_\mathrm{a}`, :math:`i_\mathrm{b}`, and :math:`i_\mathrm{c}` as

.. math::
   i'_\mathrm{dc} = q_\mathrm{a} i_\mathrm{a} + q_\mathrm{b} i_\mathrm{b} + q_\mathrm{c} i_\mathrm{c}
   :label: DC_current

The voltage-source converter model is provided in the class :class:`motulator.drive.model.VoltageSourceConverter`. This model can be extended with a diode bridge model, see :doc:`/model/drive/diode_bridge`.

Carrier Comparison
^^^^^^^^^^^^^^^^^^

In pulse-width modulation (PWM), carrier comparison is commonly used to generate instantaneous switching state signals :math:`q_\mathrm{a}`, :math:`q_\mathrm{b}`, and :math:`q_\mathrm{c}` from duty ratios :math:`d_\mathrm{a}`, :math:`d_\mathrm{b}`, and :math:`d_\mathrm{c}`. The duty ratios are continuous signals in the range [0, 1] while the switching states are either 0 or 1.

The figure below shows the principle of carrier comparison. The logic shown in the figure is implemented in the class :class:`motulator.common.model.CarrierComparison`, where the switching instants are explicitly computed in the beginning of each sampling period (instead of searching for zero crossings), allowing faster simulations.

.. figure:: figs/carrier_comparison.svg
   :width: 100%
   :align: center
   :alt: Carrier comparison
   :target: .

   Carrier comparison. The duty ratios are :math:`d_\mathrm{a}`, :math:`d_\mathrm{b}`, and :math:`d_\mathrm{c}` are constant over the sampling period :math:`T_\mathrm{s}` (or, optionally, over the the switching period :math:`T_\mathrm{sw} = 2T_\mathrm{s}`). The carrier signal is the same for all three phases and varies between 0 and 1.

The zero-sequence voltage does not affect the phase currents if the neutral of the load is not connected. Therefore, the reference potential of the phase voltages can be freely chosen when computing the space vector of the converter output voltage. The converter voltage vector in stationary coordinates is

.. math::
	\boldsymbol{u}_\mathrm{c}^\mathrm{s} &= \frac{2}{3}\left(u_\mathrm{an} + u_\mathrm{bn}\mathrm{e}^{\mathrm{j}2\pi/3} + u_\mathrm{cn}\mathrm{e}^{\mathrm{j} 4\pi/3}\right) \\
	&= \frac{2}{3}\left(u_\mathrm{aN} + u_\mathrm{bN}\mathrm{e}^{\mathrm{j} 2\pi/3} + u_\mathrm{cN}\mathrm{e}^{\mathrm{j} 4\pi/3}\right) \\
   &= \underbrace{\frac{2}{3}\left(q_\mathrm{a} + q_\mathrm{b}\mathrm{e}^{\mathrm{j} 2\pi/3} + q_\mathrm{c}\mathrm{e}^{\mathrm{j} 4\pi/3}\right)}_{\boldsymbol{q}_\mathrm{c}^\mathrm{s}}u_\mathrm{dc}
   :label: carrier_comparison

where :math:`\boldsymbol{q}_\mathrm{c}^\mathrm{s}` is the switching-state space vector.

.. note::
   The carrier comparison is compatible with all standard pulse-width modulation (PWM) methods, such as space-vector PWM (see :class:`motulator.common.control.PWM`) and discontinuous PWM methods [#Hol1994]_, [#Hav1999]_.

   The sampling period :math:`T_\mathrm{s}` is returned by the control method, and it does not need to be constant.

   If the zero sequence is of interest, it could be easily added to the converter model.

Switching-Cycle Averaging
^^^^^^^^^^^^^^^^^^^^^^^^^

If the switching ripple is not of interest in simulations, the carrier comparison can be replaced with zero-order hold (ZOH) of the duty ratios. In this case, the output voltage vector is

.. math::
	\boldsymbol{u}_\mathrm{c}^\mathrm{s} = \underbrace{\frac{2}{3}\left(d_\mathrm{a} + d_\mathrm{b}\mathrm{e}^{\mathrm{j} 2\pi/3} + d_\mathrm{c}\mathrm{e}^{\mathrm{j} 4\pi/3}\right)}_{\boldsymbol{d}_\mathrm{c}^\mathrm{s}}u_\mathrm{dc}
   :label: switching_cycle_averaging

where :math:`\boldsymbol{d}_\mathrm{c}^\mathrm{s}` is the duty ratio space vector. This ZOH is the default option in most of :doc:`/drive_examples/index` and :doc:`/grid_examples/index` examples.

.. rubric:: References

.. [#Fra1997] Franklin, Powell, Workman, "Digital Control of Dynamic Systems," Menlo Park, CA, USA: Addison-Wesley, 1997

.. [#Bus2015] Buso, Mattavelli, "Digital Control in Power Electronics," 2nd ed., Morgan & Claypool, 2015, https://doi.org/10.2200/S00637ED1V01Y201503PEL007

.. [#Hin2024] Hinkkanen,  Harnefors, Kukkola, "Fundamentals of Electric Machine Drives," lecture notes, 2024, https://doi.org/10.5281/zenodo.10609166

.. [#Hol1994] Holtz, "Pulsewidth modulation for electronic power conversion," Proc. IEEE, 1994, https://doi.org/10.1109/5.301684

.. [#Hav1999] Hava, Kerkman, Lipo, "Simple analytical and graphical methods for carrier-based PWM-VSI drives," IEEE Trans. Ind. Appl., 1999, https://doi.org/10.1109/63.737592
