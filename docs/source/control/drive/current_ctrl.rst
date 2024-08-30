Current Control
===============

Synchronous-frame two-degrees-of-freedom (2DOF) proportional-integral (PI) current control is commonly used in three-phase AC machine drives [#Har1998]_, [#Bri1999]_, [#Awa2019]_, [#Hin2024]_. This control structure allows to compensate for the cross-coupling originating from rotating coordinates as well as to improve disturbance rejection. 

The current controller for an induction machine is available in the :class:`motulator.drive.control.im.CurrentController` class and for a synchronous machine in the :class:`motulator.drive.control.sm.CurrentController` class, both of which inherit from the :class:`motulator.common.control.ComplexPIController` class. In the following, current control of an induction machine is first considered in detail. Then, the same principles are applied to synchronous machines. Complex space vectors are used to represent three-phase quantities.

Induction Machines
------------------

System Model
^^^^^^^^^^^^

The inverse-Γ model of an induction machine is considered (see :doc:`/model/drive/machines`). Using the stator current :math:`\boldsymbol{i}_\mathrm{s}` and the rotor flux linkage :math:`\boldsymbol{\psi}_\mathrm{R}` as state variables, the model in synchronous coordinates rotating at :math:`\omega_\mathrm{s}` can be written as

.. math::
    L_\sigma \frac{\mathrm{d} \boldsymbol{i}_\mathrm{s}}{\mathrm{d} t} &= \boldsymbol{u}_\mathrm{s} - (R_\sigma + \mathrm{j} \omega_\mathrm{s}L_\sigma)\boldsymbol{i}_\mathrm{s} - \underbrace{\left(\mathrm{j}\omega_\mathrm{m} - \frac{R_\mathrm{R}}{L_\mathrm{M}}\right)\boldsymbol{\psi}_\mathrm{R}}_{\text{back-emf } \boldsymbol{e}_\mathrm{s}} \\
	\frac{\mathrm{d} \boldsymbol{\psi}_\mathrm{R}}{\mathrm{d} t} &= R_\mathrm{R}\boldsymbol{i}_\mathrm{s} - \left(\frac{R_\mathrm{R}}{L_\mathrm{M}} + \mathrm{j}\omega_\mathrm{r} \right)\boldsymbol{\psi}_\mathrm{R} 

where :math:`R_\sigma = R_\mathrm{s} + R_\mathrm{R}` is the total resistance and :math:`\omega_\mathrm{r} = \omega_\mathrm{s} - \omega_\mathrm{m}` is the slip angular frequency. The rotor flux linkage :math:`\boldsymbol{\psi}_\mathrm{R}` and the rotor speed :math:`\omega_\mathrm{m}` change slowly as compared to the stator current. 

Consequently, the back-emf :math:`\boldsymbol{e}_\mathrm{s}` can be considered as a quasi-constant input (load) disturbance for the current controller, and it suffices to consider the stator current dynamics

.. math::
    L_\sigma \frac{\mathrm{d} \boldsymbol{i}_\mathrm{s}}{\mathrm{d} t} = \boldsymbol{u}_\mathrm{s} - (R_\sigma + \mathrm{j} \omega_\mathrm{s}L_\sigma)\boldsymbol{i}_\mathrm{s} - \boldsymbol{e}_\mathrm{s}
    :label: im_current

2DOF PI Controller
^^^^^^^^^^^^^^^^^^

The design of synchronous-frame 2DOF PI current control is considered in the continuous-time domain, even though the actual implementation is discrete. Two typical gain selections for this control type are known as the internal-model-control (IMC) design [#Har1998]_ and the complex-vector design [#Bri1999]_. Here, only the complex-vector design is considered, see :ref:`complex-vector-2dof-pi-controller`, which is compatible with the :class:`motulator.common.control.ComplexPIController` base class. The controller can be expressed in a state-space form as

.. math::
	\frac{\mathrm{d} \boldsymbol{u}_\mathrm{i}}{\mathrm{d} t} &= (\boldsymbol{k}_\mathrm{i} + \mathrm{j}\omega_\mathrm{s}\boldsymbol{k}_\mathrm{t} )\left(\boldsymbol{i}_\mathrm{s,ref} - \boldsymbol{i}_\mathrm{s}\right) \\
    \boldsymbol{u}_\mathrm{s,ref} &= \boldsymbol{k}_\mathrm{t}\boldsymbol{i}_\mathrm{s,ref} - \boldsymbol{k}_\mathrm{p}\boldsymbol{i}_\mathrm{s} + \boldsymbol{u}_\mathrm{i} 
    :label: cc

where :math:`\boldsymbol{u}_\mathrm{s,ref}` is the output of the controller, i.e., the stator voltage reference, :math:`\boldsymbol{i}_\mathrm{s,ref}` is the stator current reference, :math:`\boldsymbol{u}_\mathrm{i}` is the the integral state, and :math:`\omega_\mathrm{s}` is the angular speed of the coordinate system. Furthermore, :math:`\boldsymbol{k}_\mathrm{t}` is the reference-feedforward gain, :math:`\boldsymbol{k}_\mathrm{p}` is the proportional gain, and :math:`\boldsymbol{k}_\mathrm{i}` is the integral gain. 

.. note::
   The gain definitions used in :eq:`cc` differ from [#Hin2024]_, where a more general controller structure is considered. 

Closed-Loop System 
^^^^^^^^^^^^^^^^^^

Here, ideal voltage production is assumed, :math:`\boldsymbol{u}_\mathrm{s} = \boldsymbol{u}_\mathrm{s,ref}`. Using :eq:`im_current` and :eq:`cc`, the closed-loop system in the Laplace domain becomes

.. math::
	\boldsymbol{i}_\mathrm{s} = \boldsymbol{G}_\mathrm{c}(s)\boldsymbol{i}_\mathrm{s,ref} - \boldsymbol{Y}_\mathrm{c}(s)\boldsymbol{e}_\mathrm{s}

The disturbance rejection depends on the closed-loop admittance

.. math::
    \boldsymbol{Y}_\mathrm{c}(s) = \frac{s}{L_\sigma s^2 + (R_\sigma + \mathrm{j}\omega_\mathrm{s} L_\sigma + \boldsymbol{k}_\mathrm{p}) s + \boldsymbol{k}_\mathrm{i} + \mathrm{j}\omega_\mathrm{s} \boldsymbol{k}_\mathrm{t}} 
    :label: Yc

The closed-loop poles can be arbitrarily placed by means of the gains. The reference-tracking transfer function is

.. math::
	\boldsymbol{G}_\mathrm{c}(s) = \frac{(s + \mathrm{j}\omega_\mathrm{s}) \boldsymbol{k}_\mathrm{t} + \boldsymbol{k}_\mathrm{i} }{L_\sigma s^2 + (R_\sigma + \mathrm{j}\omega_\mathrm{s} L_\sigma + \boldsymbol{k}_\mathrm{p}) s + \boldsymbol{k}_\mathrm{i} + \mathrm{j}\omega_\mathrm{s} \boldsymbol{k}_\mathrm{t}}     
    :label: Gc

whose zero can be placed by means of the reference-feedforward gain :math:`\boldsymbol{k}_\mathrm{t}`.

Gain Selection
^^^^^^^^^^^^^^

Consider the gains

.. math::                
    \boldsymbol{k}_\mathrm{p} = 2\alpha_\mathrm{c} \hat L_\sigma - \hat R_\sigma \qquad\qquad
    \boldsymbol{k}_\mathrm{i} = \alpha_\mathrm{c}^2\hat L_\sigma  \qquad \qquad
    \boldsymbol{k}_\mathrm{t} = \alpha_\mathrm{c} \hat L_\sigma
    :label: complex_vector_gains

where :math:`\hat R_\sigma = 0` can be used in practice due to its minor effect and integral action. Assuming accurate parameter estimates, the closed-loop transfer functions :eq:`Yc` and :eq:`Gc` reduce to

.. math::
    \boldsymbol{Y}_\mathrm{c}(s) = \frac{s}{L_\sigma (s + \alpha_\mathrm{c})(s + \alpha_\mathrm{c} + \mathrm{j}\omega_\mathrm{s})}
    \qquad\qquad
    \boldsymbol{G}_\mathrm{c}(s) = \frac{\alpha_\mathrm{c}}{s + \alpha_\mathrm{c}} 

It can be seen that this design results in the first-order reference-tracking dynamics. Furthermore, one pole is placed at the real axis at :math:`s=-\alpha_\mathrm{c}`, while another pole moves with the angular frequency of the coordinate system, :math:`s= -\alpha_\mathrm{c} - \mathrm{j}\omega_\mathrm{s}`. The complex-vector design tends to be slightly more robust to parameter errors than the IMC design since the other closed-loop pole approximately corresponds to the open-loop pole.  

This gain selection is used in the :class:`motulator.drive.control.im.CurrentController` class. The stator voltage is limited in practice due to the limited DC-bus voltage of the converter. Consequently, the realized (limited) voltage reference is

.. math::
    \bar{\boldsymbol{u}}_\mathrm{s,ref} = \mathrm{sat}(\boldsymbol{u}_\mathrm{s,ref})

where :math:`\mathrm{sat}(\cdot)` is the saturation function. The limited voltage can be obtained from a pulse-width modulation (PWM) algorithm. The anti-windup of the integrator is included in the implementation of the :class:`motulator.common.control.ComplexPIController` base class.

Synchronous Machines
--------------------

System Model
^^^^^^^^^^^^

Consider the synchronous machine model in rotor coordinates, rotating at :math:`\omega_\mathrm{m}` and aligned along the d-axis of the rotor, 

.. math::
    \frac{\mathrm{d}\boldsymbol{\psi}_\mathrm{s}}{\mathrm{d} t} &= \boldsymbol{u}_\mathrm{s} - R_\mathrm{s}\boldsymbol{i}_\mathrm{s} - \mathrm{j}\omega_\mathrm{m}\boldsymbol{\psi}_\mathrm{s} \\
 	\boldsymbol{\psi}_\mathrm{s} &= L_\mathrm{d}\mathrm{Re}\{\boldsymbol{i}_\mathrm{s}\} + \mathrm{j}L_\mathrm{q}\mathrm{Im}\{\boldsymbol{i}_\mathrm{s}\} + \psi_\mathrm{f} 
    :label: sm_model
 
where linear magnetics are assumed for simplicity. For further details, see :doc:`/model/drive/machines`.

2DOF PI Controller
^^^^^^^^^^^^^^^^^^

An internal change of the state variable from the stator current to the stator flux linkage simplifies the control design for synchronous machines, allowing the same algorithm to be used for both non-salient and salient synchronous machines [#Awa2019]_. Both the reference current and the actual current are scaled by the inductance estimates,

.. math::
    \boldsymbol{\psi}_\mathrm{s,ref} &= \hat{L}_\mathrm{d}\mathrm{Re}\{\boldsymbol{i}_\mathrm{s,ref}\} + \mathrm{j} \hat{L}_\mathrm{q}\mathrm{Im}\{\boldsymbol{i}_\mathrm{s,ref}\} \\
    \hat{\boldsymbol{\psi}_\mathrm{s}} &= \hat{L}_\mathrm{d}\mathrm{Re}\{\boldsymbol{i}_\mathrm{s}\} + \mathrm{j} \hat{L}_\mathrm{q}\mathrm{Im}\{\boldsymbol{i}_\mathrm{s}\} 
    :label: flux_mapping_sm

This choice of using the flux linkage as the internal state has some advantages: the gain expressions become simpler; the magnetic saturation would be more convenient to take into account; and the same control structure can be used for salient and nonsalient machines. 

Here, the complex vector design is considered. Hence, the controller :eq:`cc` can be rewritten as 

.. math::
	\frac{\mathrm{d} \boldsymbol{u}_\mathrm{i}}{\mathrm{d} t} &= (\boldsymbol{k}_\mathrm{i} + \mathrm{j}\omega_\mathrm{s}\boldsymbol{k}_\mathrm{t} )\left(\boldsymbol{\psi}_\mathrm{s,ref} - \hat{\boldsymbol{\psi}}_\mathrm{s}\right) \\
    \boldsymbol{u}_\mathrm{s,ref} &= \boldsymbol{k}_\mathrm{t}\boldsymbol{\psi}_\mathrm{s,ref} - \boldsymbol{k}_\mathrm{p}\hat{\boldsymbol{\psi}}_\mathrm{s} + \boldsymbol{u}_\mathrm{i} 
    :label: cc_flux

where the angular speed of the coordinate system equals typically the measured rotor speed, :math:`\omega_\mathrm{s} = \omega_\mathrm{m}`, or the estimated rotor speed :math:`\omega_\mathrm{s} = \hat{\omega}_\mathrm{m}`. If the magnetic saturation is not considered, this flux-linkage-based current controller is equivalent to a regular 2DOF PI current controller (even if inductance estimates are inaccurate). Notice that :math:`\boldsymbol{i}_\mathrm{s,ref} = \boldsymbol{i}_\mathrm{s}` holds in the steady state even with inductance estimate inaccuracies, since the same inductances are used to map both the reference current and the actual current to the corresponding flux linkages. 

The gain design analogous to :eq:`complex_vector_gains` becomes

.. math::                
    \boldsymbol{k}_\mathrm{p} = 2\alpha_\mathrm{c} - \hat R_\mathrm{s} \qquad\qquad
    \boldsymbol{k}_\mathrm{i} = \alpha_\mathrm{c}^2 \qquad \qquad
    \boldsymbol{k}_\mathrm{t} = \alpha_\mathrm{c} 

where :math:`\hat R_\mathrm{s} = 0` can be used in practice. Assume accurate parameter estimates and perfect alignment of the controller coordinate system with the rotor coordinate system. Then, using :eq:`sm_model`, :eq:`flux_mapping_sm`, and :eq:`cc_flux`, the closed-loop system can be shown to be analogous to the induction machine case. This control design corresponds to the implementation in the :class:`motulator.drive.control.sm.CurrentController` class. 

.. rubric:: References

.. [#Har1998] Harnefors, Nee, "Model-based current control of AC machines using the internal model control method," IEEE Trans. Ind. Appl., 1998, https://doi.org/10.1109/28.658735

.. [#Bri1999] Briz del Blanco, Degner, Lorenz, “Dynamic analysis of current regulators for AC motors using complex vectors,” IEEE Trans.Ind. Appl., 1999, https://doi.org/10.1109/28.806058

.. [#Awa2019] Awan, Saarakkala, Hinkkanen, "Flux-linkage-based current control of saturated synchronous motors," IEEE Trans. Ind. Appl. 2019, https://doi.org/10.1109/TIA.2019.2919258

.. [#Hin2024] Hinkkanen,  Harnefors, Kukkola, "Fundamentals of Electric Machine Drives," lecture notes, 2024, https://doi.org/10.5281/zenodo.10609166


