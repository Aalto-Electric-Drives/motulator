Speed Control
=============

A speed controller is implemented in the :class:`motulator.drive.control.SpeedController` class, whose base class is :class:`motulator.common.control.PIController`. In the following, the tuning of the speed controller is discussed. The presented approach can be extended to many other control tasks as well.

.. _2dof-pi-controller:

2DOF PI Controller
------------------

Even if controllers operate in the discrete-time domain, they are often designed and analyzed in the continuous-time domain. The state-space form of a simple 2DOF PI speed controller is given by [#Hin2024]_

.. math::
	\frac{\mathrm{d} \tau_\mathrm{i}}{\mathrm{d} t} &= k_\mathrm{i}\left(\omega_\mathrm{M,ref} - \omega_\mathrm{M}\right) \\
    	\tau_\mathrm{M,ref} &= k_\mathrm{t}\omega_\mathrm{M,ref} - k_\mathrm{p}\omega_\mathrm{M} + \tau_\mathrm{i}
    :label: speed_ctrl

where :math:`\omega_\mathrm{M}` is the measured (or estimated) mechanical angular speed of the rotor, :math:`\omega_\mathrm{M,ref}` is the reference angular speed, and :math:`\tau_\mathrm{i}` is the the integral state. Furthermore, :math:`k_\mathrm{t}` is the reference feedforward gain, :math:`k_\mathrm{p}` is the proportional gain, and :math:`k_\mathrm{i}` is the integral gain. Setting :math:`k_\mathrm{t} = k_\mathrm{p}` results in the standard PI controller. This 2DOF PI controller can also be understood as a state feedback controller with integral action and reference feedforward [#Fra1997]_.

Closed-Loop System
------------------

For simplicity, let us assume ideal torque control (:math:`\tau_\mathrm{M} = \tau_\mathrm{M,ref}`) and a stiff mechanical system

.. math::
    J\frac{\mathrm{d}\omega_\mathrm{M}}{\mathrm{d} t} = \tau_\mathrm{M} - \tau_\mathrm{L}
    :label: stiff_mech

where :math:`\tau_\mathrm{M}` is the electromagnetic torque, :math:`\tau_\mathrm{L}` is the load torque, and :math:`J` is the total moment of inertia. In the Laplace domain, the closed-loop system resulting from :eq:`speed_ctrl` and :eq:`stiff_mech` is given by

.. math::
    \omega_\mathrm{M}(s) = \frac{k_\mathrm{t} s + k_\mathrm{i}}{J s^2 + k_\mathrm{p} s + k_\mathrm{i}} \omega_\mathrm{M,ref}(s) - \frac{s}{J s^2 + k_\mathrm{p} s + k_\mathrm{i}} \tau_\mathrm{L}(s)
    :label: speed_ctrl_closed_loop_system

where it can be seen that the gain :math:`k_\mathrm{t}` allows to place the reference-tracking zero.

Gain Selection
--------------

The gain selection [#Har2013]_

.. math::
    k_\mathrm{t} = \alpha_\mathrm{s} \hat{J} \qquad
    k_\mathrm{p} = (\alpha_\mathrm{s} + \alpha_\mathrm{i}) \hat{J} \qquad
    k_\mathrm{i} = \alpha_\mathrm{s}\alpha_\mathrm{i} \hat{J}
    :label: speed_ctrl_gain_selection

results in

.. math::
    \omega_\mathrm{M}(s) = \frac{\alpha_\mathrm{s}}{s + \alpha_\mathrm{s}} \omega_\mathrm{M,ref}(s) - \frac{s}{J (s + \alpha_\mathrm{s})(s + \alpha_\mathrm{i})} \tau_\mathrm{L}(s)
    :label: speed_ctrl_closed_loop_system2

where :math:`\alpha_\mathrm{s}` is the closed-loop reference-tracking bandwidth and :math:`\alpha_\mathrm{i}` is the integral action bandwidth. An accurate inertia estimate :math:`\hat{J} = J` is assumed in the above closed-loop system.

.. rubric:: References

.. [#Hin2024] Hinkkanen,  Harnefors, Kukkola, "Fundamentals of Electric Machine Drives," lecture notes, 2024, https://doi.org/10.5281/zenodo.10609166

.. [#Fra1997] Franklin, Powell, Workman, "Digital Control of Dynamic Systems," 3rd ed., Menlo Park, CA: Addison-Wesley, 1997

.. [#Har2013] Harnefors, Saarakkala, Hinkkanen, "Speed control of electrical drives using classical control methods," IEEE Trans. Ind. Appl., 2013, https://doi.org/10.1109/TIA.2013.2244194
