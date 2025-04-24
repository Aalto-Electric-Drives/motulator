Controllers
===========

2DOF PI Controller
------------------

Proportional-integral (PI) control is widely used in various applications. A standard one-degree-of-freedom (1DOF) PI controller manipulates only the control error, i.e., it has single input and single output. Its two-degrees-of-freedom (2DOF) variants have two inputs (reference signal and feedback signal), which allows to design disturbance rejection and reference tracking separately [#Sko1996]_. The 2DOF PI controller is available in the :class:`motulator.common.control.PIController` class, which is the base class for the :class:`motulator.drive.control.im.SpeedController` and :class:`motulator.grid.control.DCBusVoltageController` classes.

Typical Structure
^^^^^^^^^^^^^^^^^

The figure below shows a 2DOF PI controller with an optional feedforward term. Its equivalent state-space form is given by

.. math::
	\frac{\mathrm{d} u_\mathrm{i}}{\mathrm{d} t} &= k_\mathrm{i}\left(r - y\right) \\
   u &= k_\mathrm{t}r - k_\mathrm{p}y + u_\mathrm{i} + u_\mathrm{ff}
   :label: 2dof_pi

where :math:`r` is the reference signal, :math:`y` is the measured (or estimated) feedback signal, :math:`u_\mathrm{i}` is the the integral state, and :math:`u_\mathrm{ff}` is the optional feedforward signal. Furthermore, :math:`k_\mathrm{t}` is the reference-feedforward gain, :math:`k_\mathrm{p}` is the proportional gain, and :math:`k_\mathrm{i}` is the integral gain. Setting :math:`k_\mathrm{t} = k_\mathrm{p}` and :math:`u_\mathrm{ff} = 0` results in the standard PI controller. This 2DOF PI controller can also be understood as a state-feedback controller with integral action and reference feedforward [#Fra1997]_.

.. figure:: ../figs/2dof_pi.svg
   :figclass: only-light
   :width: 100%
   :align: center
   :alt: 2DOF PI controller.

   2DOF PI controller with an optional feedforward term. The operator :math:`1/s` refers to integration. A discrete-time variant of this controller with the integrator anti-windup is implemented in the :class:`motulator.common.control.PIController` class.

.. figure:: ../figs/2dof_pi.svg
   :figclass: invert-colors-dark only-dark
   :width: 100%
   :align: center
   :alt: 2DOF PI controller.

   2DOF PI controller with an optional feedforward term. The operator :math:`1/s` refers to integration. A discrete-time variant of this controller with the integrator anti-windup is implemented in the :class:`motulator.common.control.PIController` class.

.. _disturbance-observer-structure:

Disturbance-Observer Structure
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The controller :eq:`2dof_pi` can be equally represented using the disturbance-observer structure as

.. math::
	\frac{\mathrm{d} u_\mathrm{i}}{\mathrm{d} t} &= \alpha_\mathrm{i}\left(u - \hat v\right) \\
   \hat v &= u_\mathrm{i} - (k_\mathrm{p} - k_\mathrm{t})y + u_\mathrm{ff} \\
   u &= k_\mathrm{t}\left(r - y\right) + \hat v
   :label: 2dof_pi_disturbance_observer

where :math:`\alpha_\mathrm{i} = k_\mathrm{i}/k_\mathrm{t}` is the redefined integral gain and :math:`\hat v` is the input-equivalent disturbance estimate. This structure is convenient to prevent the integral windup that originates from the actuator saturation [#Fra1997]_. The actuator output is limited in practice due to physical constraints. Consequently, the realized actuator output is

.. math::
   \bar{u} = \mathrm{sat}(u)

where :math:`\mathrm{sat}(\cdot)` is the saturation function. If this saturation function is known, the anti-windup of the integrator can be implemented simply as

.. math::
	\frac{\mathrm{d} u_\mathrm{i}}{\mathrm{d} t} = \alpha_\mathrm{i}\left(\bar{u} - \hat v \right)
   :label: anti_windup

The other parts of the above controller are not affected by the saturation.

Discrete-Time Algorithm
^^^^^^^^^^^^^^^^^^^^^^^

The discrete-time variant of the controller :eq:`2dof_pi_disturbance_observer` with the anti-windup in :eq:`anti_windup` is given by

.. math::
	u_\mathrm{i}(k+1) &= u_\mathrm{i}(k) + T_\mathrm{s} \alpha_\mathrm{i} \left[\bar{u}(k) - \hat v(k) \right] \\
   \hat v(k) &= u_\mathrm{i}(k) - (k_\mathrm{p} - k_\mathrm{t})y(k) + u_\mathrm{ff}(k) \\
   u(k) &= k_\mathrm{t}\left[r(k) - y(k)\right] + \hat v(k) \\
   \bar{u}(k) &= \mathrm{sat}[u(k)]
   :label: discrete_2dof_pi

where :math:`T_\mathrm{s}` is the sampling period and :math:`k` is the discrete-time index. This algorithm corresponds to the actual implementation in the :class:`motulator.common.control.PIController` class.

.. _complex-vector-2dof-pi-controller:

Complex-Vector 2DOF PI Controller
---------------------------------

As shown in the figure below, the 2DOF PI controller presented above can be extended for the control of complex-valued space vectors in a coordinate system rotating at the angular speed :math:`\omega` [#Bri1999]_. Depending on the control task, the controlled quantity is typically either a current vector or a flux linkage vector. In the continuous-time domain, the controller in the state-space form is given by

.. math::
	\frac{\mathrm{d} \boldsymbol{u}_\mathrm{i}}{\mathrm{d} t} &= (\boldsymbol{k}_\mathrm{i} + \mathrm{j}\omega \boldsymbol{k}_\mathrm{t})\left(\boldsymbol{r} - \boldsymbol{y}\right) \\
    \boldsymbol{u} &= \boldsymbol{k}_\mathrm{t}\boldsymbol{r} - \boldsymbol{k}_\mathrm{p}\boldsymbol{y} + \boldsymbol{u}_\mathrm{i} + \boldsymbol{u}_\mathrm{ff}
    :label: complex_vector_2dof_pi

where :math:`\boldsymbol{u}` is the output of the controller, :math:`\boldsymbol{r}` is the reference signal, :math:`\boldsymbol{u}_\mathrm{i}` is the the integral state, and :math:`\boldsymbol{u}_\mathrm{ff}` is the optional feedforward signal. Furthermore, :math:`\boldsymbol{k}_\mathrm{t}` is the reference-feedforward gain, :math:`\boldsymbol{k}_\mathrm{p}` is the proportional gain, and :math:`\boldsymbol{k}_\mathrm{i}` is the integral gain.

.. figure:: ../figs/complex_vector_2dof_pi.svg
   :figclass: only-light
   :width: 100%
   :align: center
   :alt: 2DOF complex-vector PI controller, with feedforward.

   2DOF complex-vector PI controller with an optional feedforward term.

.. figure:: ../figs/complex_vector_2dof_pi.svg
   :figclass: invert-colors-dark only-dark
   :width: 100%
   :align: center
   :alt: 2DOF complex-vector PI controller, with feedforward.

   2DOF complex-vector PI controller with an optional feedforward term.

The discrete-time implementation of :eq:`complex_vector_2dof_pi` with the anti-windup is given in the :class:`motulator.common.control.ComplexPIController` class, which is the base class for :class:`motulator.drive.control.sm.CurrentController`, :class:`motulator.drive.control.im.CurrentController`, and :class:`motulator.grid.control.CurrentController` classes. The algorithm is similar to the real-valued case given in :eq:`discrete_2dof_pi`.

.. rubric:: References

.. [#Sko1996] Skogestad, Postlethwaite, "Multivariable Feedback Control: Analysis and Design," West Sussex, England: John Wiley and Sons, 1996

.. [#Fra1997] Franklin, Powell, Workman, "Digital Control of Dynamic Systems," 3rd ed., Menlo Park, CA: Addison-Wesley, 1997

.. [#Bri1999] Briz del Blanco, Degner, Lorenz, “Dynamic analysis of current regulators for AC motors using complex vectors,” IEEE Trans. Ind. Appl., 1999, https://doi.org/10.1109/28.806058
