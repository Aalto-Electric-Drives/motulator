Speed Control
=============

Proportional-integral (PI) control is widely used in machine drives. A standard one-degree-of-freedom (1DOF) PI controller manipulates only the control error, i.e., it has single input and single output. Its two-degrees-of-freedom (2DOF) variants have two inputs (reference signal and feedback signal), which allows to design disturbance rejection and reference tracking separately [#Sko1996]_. In the following, we will use a speed controller as an example, cf. the :class:`motulator.control.SpeedCtrl` class. The presented control design can be extended to many other control tasks as well. 

Continuous-Time Design
----------------------

Even if controllers operate in the discrete-time domain, they are often designed and analyzed in the continuous-time domain.  

Typical Structure
^^^^^^^^^^^^^^^^^

The state-space form of a simple 2DOF PI speed controller is given by

.. math::
	\frac{\mathrm{d} \tau_\mathrm{i}}{\mathrm{d} t} &= k_\mathrm{i}\left(\omega_\mathrm{M,ref} - \omega_\mathrm{M}\right) \\
    	\tau_\mathrm{M,ref} &= k_\mathrm{t}\omega_\mathrm{M,ref} - k_\mathrm{p}\omega_\mathrm{M} + \tau_\mathrm{i} 

where :math:`\omega_\mathrm{M}` is the measured (or estimated) mechanical angular speed of the rotor, :math:`\omega_\mathrm{M,ref}` is the reference angular speed, and :math:`\tau_\mathrm{i}` is the the integral state. Furthermore, :math:`k_\mathrm{t}` is the reference feedforward gain, :math:`k_\mathrm{p}` is the proportional gain, and :math:`k_\mathrm{i}` is the integral gain. Setting :math:`k_\mathrm{t} = k_\mathrm{p}` results in the standard PI controller. This 2DOF PI controller can also be understood as a state feedback controller with intergral action and reference feedforward [#Fra1997]_. 

..
    For analysis purposes, the above controller can be presented in the Laplace domain as
..
    .. math::
	\tau_\mathrm{M,ref}(s) = K(s) \left[\omega_\mathrm{M,ref}(s) - \omega_\mathrm{M}(s)\right] + (k_\mathrm{t} - k_\mathrm{p})\omega_\mathrm{M,ref}(s) 
..
    where
..
    .. math::
	K(s) = k_\mathrm{p} + \frac{k_\mathrm{i}}{s}
..
    is the standard PI controller.

Disturbance-Observer Structure
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The above 2DOF PI controller can be equally represented using the disturbance-observer structure,

.. math::
	\frac{\mathrm{d} \tau_\mathrm{i}}{\mathrm{d} t} &= \alpha_\mathrm{i}\left(\tau_\mathrm{M,ref} - \hat \tau_\mathrm{L}\right) \\
    \hat \tau_\mathrm{L} &= \tau_\mathrm{i} - (k_\mathrm{p} - k_\mathrm{t})\omega_\mathrm{M} \\
    \tau_\mathrm{M,ref} &= k_\mathrm{t}\left(\omega_\mathrm{M,ref} - \omega_\mathrm{M}\right) + \hat \tau_\mathrm{L} 

where :math:`\alpha_\mathrm{i} = k_\mathrm{i}/k_\mathrm{t}` is the redefined integral gain and :math:`\hat \tau_\mathrm{L}` is the input-equivalent disturbance estimate (i.e., the load torque estimate). This structure is convenient to prevent the integral windup that originates from the actuator saturation [#Fra1997]_. The electromagnetic torque is limited in practice due to the maximum current of the inverter (and possibly due to other constraints as well). Consequently, the realized (limited) torque reference is

.. math::
    \bar{\tau}_\mathrm{M,ref} = \mathrm{sat}(\tau_\mathrm{M,ref})

where :math:`\mathrm{sat}(\cdot)` is the saturation function. If this saturation function is known, the anti-windup of the integrator can be implemented simply as

.. math::
	\frac{\mathrm{d} \tau_\mathrm{i}}{\mathrm{d} t} = \alpha_\mathrm{i}\left(\bar{\tau}_\mathrm{M,ref} - \hat \tau_\mathrm{L}\right) 

The other parts of the above controller are not affected by the saturation. The implementation in the :class:`motulator.control.SpeedCtrl` class is based on this disturbance-observer form.

Gain Selection Example
^^^^^^^^^^^^^^^^^^^^^^

For simplicity, let us assume ideal torque control (:math:`\tau_\mathrm{M} = \tau_\mathrm{M,ref}`) and a stiff mechanical system [#Har2013]_

.. math::
    J\frac{\mathrm{d}\omega_\mathrm{M}}{\mathrm{d} t} = \tau_\mathrm{M} - \tau_\mathrm{L}

where :math:`\tau_\mathrm{M}` is the electromagnetic torque, :math:`\tau_\mathrm{L}` is the load torque, and :math:`J` is the total moment of inertia. In the Laplace domain, the resulting closed-loop system is given by

.. math::
    \omega_\mathrm{M}(s) = \frac{k_\mathrm{t} s + k_\mathrm{i}}{J s^2 + k_\mathrm{p} s + k_\mathrm{i}} \omega_\mathrm{M,ref}(s) - \frac{s}{J s^2 + k_\mathrm{p} s + k_\mathrm{i}} \tau_\mathrm{L}(s)

where it can be seen that the gain :math:`k_\mathrm{t}` allows to place the reference-tracking zero. 

The gain selection 

.. math::
    k_\mathrm{t} = \alpha_\mathrm{s} \hat{J} \qquad
    k_\mathrm{p} = 2\alpha_\mathrm{s} \hat{J} \qquad
    k_\mathrm{i} = \alpha_\mathrm{s}^2 \hat{J} 

results in 

.. math::
    \omega_\mathrm{M}(s) = \frac{\alpha_\mathrm{s}}{s + \alpha_\mathrm{s}} \omega_\mathrm{M,ref}(s) - \frac{s}{J (s + \alpha_\mathrm{s})^2} \tau_\mathrm{L}(s)

where :math:`\alpha_\mathrm{s}` is the closed-loop bandwidth of reference tracking and :math:`\hat{J} = J` is assumed.

Discrete-Time Algorithm
-----------------------

The discrete-time variant of the controller is given by

.. math::
	\tau_\mathrm{i}(k+1) &= \tau_\mathrm{i}(k) + T_\mathrm{s} \alpha_\mathrm{i} \left[\bar{\tau}_\mathrm{M,ref}(k) - \hat \tau_\mathrm{L}(k) \right] \\
    \hat \tau_\mathrm{L}(k) &= \tau_\mathrm{i}(k) - (k_\mathrm{p} - k_\mathrm{t})\omega_\mathrm{M}(k) \\
    \tau_\mathrm{M,ref}(k) &= k_\mathrm{t}\left[\omega_\mathrm{M,ref}(k) - \omega_\mathrm{M}(k)\right] + \hat \tau_\mathrm{L}(k) \\
    \bar{\tau}_\mathrm{M,ref}(k) &= \mathrm{sat}[\tau_\mathrm{M,ref}(k)]

where :math:`T_\mathrm{s}` is the sampling period and :math:`k` is the discrete-time index. This corresponds to the implementation in the :class:`motulator.control.SpeedCtrl` class. 

.. rubric:: References

.. [#Sko1996] Skogestad, Postlethwaite, "Multivariable Feedback Control: Analysis and Design," West Sussex, England: John Wiley and Sons, 1996

.. [#Fra1997] Franklin, Powell, Workman, "Digital Control of Dynamic Systems," 3rd ed., Menlo Park, CA: Addison-Wesley, 1997

.. [#Har2013] Harnefors, Saarakkala, Hinkkanen, "Speed control of electrical drives using classical control methods," IEEE Trans. Ind. Appl., 2013, https://doi.org/10.1109/TIA.2013.2244194
