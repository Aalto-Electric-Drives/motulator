Current Control
===============

Synchronous-frame two-degrees-of-freedom (2DOF) proportional-integral (PI) current control is commonly used in three-phase AC machine drives [#Har1998]_, [#Bri1999]_, [#Awa2019]_. This control structure allows to compensate for the cross-coupling originating from rotating coordinates as well as to improve disturbance rejection. The structure and design principles are essentially the same as those of 2DOF PI speed control (see :doc:`speed_ctrl`).

In the following, current control of an induction machine is first considered in detail. Then, the same control structure is applied to synchronous machines. Finally, the discrete-time implementation is described. Complex space vectors are used to represent three-phase quantities.

Induction Machines
------------------

System Model
^^^^^^^^^^^^

The inverse-Γ model of an induction machine is considered (see :doc:`/model/machines`). Using the stator current :math:`\boldsymbol{i}_\mathrm{s}` and the rotor flux linkage :math:`\boldsymbol{\psi}_\mathrm{R}` as state variables, the model in synchronous coordinates rotating at :math:`\omega_\mathrm{s}` can be written as

.. math::
    L_\sigma \frac{\mathrm{d} \boldsymbol{i}_\mathrm{s}}{\mathrm{d} t} &= \boldsymbol{u}_\mathrm{s} - (R_\sigma + \mathrm{j} \omega_\mathrm{s}L_\sigma)\boldsymbol{i}_\mathrm{s} - \underbrace{\left(\mathrm{j}\omega_\mathrm{m} - \frac{R_\mathrm{R}}{L_\mathrm{M}}\right)\boldsymbol{\psi}_\mathrm{R}}_{\text{back-emf } \boldsymbol{e}_\mathrm{s}} \\
	\frac{\mathrm{d} \boldsymbol{\psi}_\mathrm{R}}{\mathrm{d} t} &= R_\mathrm{R}\boldsymbol{i}_\mathrm{s} - \left(\frac{R_\mathrm{R}}{L_\mathrm{M}} + \mathrm{j}\omega_\mathrm{r} \right)\boldsymbol{\psi}_\mathrm{R}

where :math:`R_\sigma = R_\mathrm{s} + R_\mathrm{R}` is the total resistance and :math:`\omega_\mathrm{r} = \omega_\mathrm{s} - \omega_\mathrm{m}` is the slip angular frequency. The rotor flux linkage :math:`\boldsymbol{\psi}_\mathrm{R}` and the rotor speed :math:`\omega_\mathrm{m}` change slowly as compared to the stator current.

Consequently, the back-emf :math:`\boldsymbol{e}_\mathrm{s}` can be considered as a quasi-constant input (load) disturbance for the current controller, and it suffices to consider the stator current dynamics

.. math::
    L_\sigma \frac{\mathrm{d} \boldsymbol{i}_\mathrm{s}}{\mathrm{d} t} = \boldsymbol{u}_\mathrm{s} - (R_\sigma + \mathrm{j} \omega_\mathrm{s}L_\sigma)\boldsymbol{i}_\mathrm{s} - \boldsymbol{e}_\mathrm{s}
    :label: im_current

Equivalently, the stator current dynamics in :eq:`im_current` can be expressed as

.. math::
    \frac{\mathrm{d} \boldsymbol{\psi}_\sigma}{\mathrm{d} t} &= \boldsymbol{u}_\mathrm{s} - \left(\frac{R_\sigma}{L_\sigma} + \mathrm{j} \omega_\mathrm{s}\right)\boldsymbol{\psi}_\sigma - \boldsymbol{e}_\mathrm{s} \\
    &= \boldsymbol{u}_\mathrm{s} - \mathrm{j} \omega_\mathrm{s}\boldsymbol{\psi}_\sigma - \boldsymbol{v}_\mathrm{s}
   :label: im_leakage_flux

where :math:`\boldsymbol{\psi}_\sigma = L_\sigma \boldsymbol{i}_\mathrm{s}` is the leakage flux linkage and :math:`\boldsymbol{v} = \boldsymbol{e}_\mathrm{s} + R_\sigma \boldsymbol{i}_\mathrm{s}` is the input disturbance.

2DOF PI Control Structure
^^^^^^^^^^^^^^^^^^^^^^^^^

First, synchronous-frame 2DOF PI current control is designed and analyzed in the continuous-time domain. The controller can be expressed in a state-space form as [#Awa2019]_

.. math::
	\frac{\mathrm{d} \boldsymbol{u}_\mathrm{i}}{\mathrm{d} t} &= \boldsymbol{k}'_\mathrm{i}\left(\boldsymbol{i}_\mathrm{s,ref} - \boldsymbol{i}_\mathrm{s}\right) \\
    \boldsymbol{u}_\mathrm{s,ref} &= \boldsymbol{k}'_\mathrm{t}\boldsymbol{i}_\mathrm{s,ref} - \boldsymbol{k}'_\mathrm{p}\boldsymbol{i}_\mathrm{s} + \boldsymbol{u}_\mathrm{i}
    :label: cc

where :math:`\boldsymbol{u}_\mathrm{s,ref}` is the output of the controller, i.e., the stator voltage reference, :math:`\boldsymbol{i}_\mathrm{s,ref}` is the stator current reference, and :math:`\boldsymbol{u}_\mathrm{i}` is the the integral state. Furthermore, :math:`\boldsymbol{k}'_\mathrm{t}` is the reference-feedforward gain, :math:`\boldsymbol{k}'_\mathrm{p}` is the proportional gain, and :math:`\boldsymbol{k}'_\mathrm{i}` is the integral gain. Setting :math:`\boldsymbol{k}'_\mathrm{t} = \boldsymbol{k}'_\mathrm{p}` results in the standard PI controller.

Closed-Loop System
^^^^^^^^^^^^^^^^^^

Here, ideal voltage production is assumed, :math:`\boldsymbol{u}_\mathrm{s} = \boldsymbol{u}_\mathrm{s,ref}`. Using :eq:`im_current` and :eq:`cc`, the closed-loop system in the Laplace domain becomes

.. math::
	\boldsymbol{i}_\mathrm{s} = \boldsymbol{G}_\mathrm{c}(s)\boldsymbol{i}_\mathrm{s,ref} - \boldsymbol{Y}_\mathrm{c}(s)\boldsymbol{e}_\mathrm{s}

The disturbance rejection depends on the closed-loop admittance

.. math::
    \boldsymbol{Y}_\mathrm{c}(s) = \frac{s}{L_\sigma s^2 + (R_\sigma + \mathrm{j}\omega_\mathrm{s} L_\sigma + \boldsymbol{k}'_\mathrm{p}) s + \boldsymbol{k}'_\mathrm{i}}
    :label: Yc

The closed-loop poles can be arbitrarily placed by means of :math:`\boldsymbol{k}'_\mathrm{p}` and :math:`\boldsymbol{k}'_\mathrm{i}`. The reference-tracking transfer function is

.. math::
	\boldsymbol{G}_\mathrm{c}(s) = \frac{s \boldsymbol{k}'_\mathrm{t} + \boldsymbol{k}'_\mathrm{i}}{L_\sigma s^2 + (R_\sigma + \mathrm{j}\omega_\mathrm{s} L_\sigma + \boldsymbol{k}'_\mathrm{p}) s + \boldsymbol{k}'_\mathrm{i}}
    :label: Gc

whose zero can be placed by means of the reference-feedforward gain :math:`\boldsymbol{k}'_\mathrm{t}`.

Gain Selection
^^^^^^^^^^^^^^

Two typical gain selections, known as the internal-model-control (IMC) design [#Har1998]_ and the complex-vector design [#Bri1999]_, are described in the following.

IMC Design
""""""""""

Consider the gains

.. math::
    \boldsymbol{k}'_\mathrm{p} = (2\alpha_\mathrm{c} - \mathrm{j}\omega_\mathrm{s}) \hat L_\sigma - \hat R_\sigma \qquad\qquad
    \boldsymbol{k}'_\mathrm{i} = \alpha_\mathrm{c}^2 \hat L_\sigma \qquad \qquad
    \boldsymbol{k}'_\mathrm{t} = \alpha_\mathrm{c} \hat L_\sigma

where the hat indicates the parameter estimates. Assuming accurate parameter estimates, the closed-loop transfer functions :eq:`Yc` and :eq:`Gc` reduce to

.. math::
    \boldsymbol{Y}_\mathrm{c}(s) = \frac{s}{L_\sigma(s + \alpha_\mathrm{c})^2}
    \qquad\qquad
    \boldsymbol{G}_\mathrm{c}(s) = \frac{\alpha_\mathrm{c}}{s + \alpha_\mathrm{c}}

where :math:`\alpha_\mathrm{c}` is the closed-loop bandwidth for reference tracking. The effect of the resistance is negligible, i.e., :math:`\hat R_\sigma = 0` can be chosen.

Complex-Vector Design
"""""""""""""""""""""

Consider the gains

.. math::
    \boldsymbol{k}'_\mathrm{p} = 2\alpha_\mathrm{c} \hat L_\sigma - \hat R_\sigma \qquad\qquad
    \boldsymbol{k}'_\mathrm{i} = \alpha_\mathrm{c}(\alpha_\mathrm{c} + \mathrm{j}\omega_\mathrm{s}) \hat L_\sigma \qquad \qquad
    \boldsymbol{k}'_\mathrm{t} = \alpha_\mathrm{c} \hat L_\sigma

Assuming accurate parameter estimates, the closed-loop transfer functions :eq:`Yc` and :eq:`Gc` reduce to

.. math::
    \boldsymbol{Y}_\mathrm{c}(s) = \frac{s}{L_\sigma (s + \alpha_\mathrm{c})(s + \alpha_\mathrm{c} + \mathrm{j}\omega_\mathrm{s})}
    \qquad\qquad
    \boldsymbol{G}_\mathrm{c}(s) = \frac{\alpha_\mathrm{c}}{s + \alpha_\mathrm{c}}

It can be seen that both gain designs result in the first-order reference-tracking dynamics. The complex-vector design tends to be slightly more robust to parameter errors than the IMC design since the other closed-loop pole approximately corresponds to the open-loop pole.

Flux Linkage as an Internal State
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Based on :eq:`im_leakage_flux`, both the reference current and the measured current can be scaled by the leakage inductance estimate,

.. math::
    \boldsymbol{\psi}_{\mathrm{ref}} &= \hat L_\sigma \boldsymbol{i}_\mathrm{s,ref} \\
    \hat{\boldsymbol{\psi}} &= \hat L_\sigma \boldsymbol{i}_\mathrm{s}
    :label: flux_mapping_im

where the notation of the leakage flux has been simplified by dropping the subscript :math:`\sigma` (in order to be able to reuse some of the following equations for synchronous machines). Now the 2DOF PI controller :eq:`cc` can be rewritten as

.. math::
	\frac{\mathrm{d} \boldsymbol{u}_\mathrm{i}}{\mathrm{d} t} &= \boldsymbol{k}_\mathrm{i}\left(\boldsymbol{\psi}_{\mathrm{ref}} - \hat{\boldsymbol{\psi}}\right) \\
    \boldsymbol{u}_\mathrm{s,ref} &= \boldsymbol{k}_\mathrm{t}\boldsymbol{\psi}_{\mathrm{ref}} - \boldsymbol{k}_\mathrm{p}\hat{\boldsymbol{\psi}} + \boldsymbol{u}_\mathrm{i}
    :label: cc_flux

It can be easily seen that the controllers :eq:`cc` and :eq:`cc_flux` are equivalent if :math:`\boldsymbol{k}_\mathrm{p} = \boldsymbol{k}'_\mathrm{p}/\hat L_\sigma`, :math:`\boldsymbol{k}_\mathrm{i} = \boldsymbol{k}'_\mathrm{i}/\hat L_\sigma`, and :math:`\boldsymbol{k}_\mathrm{t} = \boldsymbol{k}'_\mathrm{t}/\hat L_\sigma`. As an example, gains for the complex-vector design reduce to

.. math::
    \boldsymbol{k}_\mathrm{p} = 2\alpha_\mathrm{c} \qquad\qquad
    \boldsymbol{k}_\mathrm{i} = \alpha_\mathrm{c}(\alpha_\mathrm{c} + \mathrm{j}\omega_\mathrm{s})  \qquad \qquad
    \boldsymbol{k}_\mathrm{t} = \alpha_\mathrm{c}
    :label: complex_vector_gains_flux

where :math:`\hat R_\sigma = 0` is assumed. This choice of using the leakage flux linkage as the internal state has some advantages: the gain expressions become simpler; the magnetic saturation would be more convenient to take into account; and the same control structure can be extended to synchronous machines [#Awa2019]_.

Disturbance-Observer Structure
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The controller :eq:`cc_flux` can be equally represented using the disturbance-observer structure,

.. math::
	\frac{\mathrm{d} \boldsymbol{u}_\mathrm{i}}{\mathrm{d} t} &= \boldsymbol{\alpha}_\mathrm{i}\left(\boldsymbol{u}_{\mathrm{s,ref}} - \hat{\boldsymbol{v}}_\mathrm{s}\right) \\
    \hat{\boldsymbol{v}}_\mathrm{s} &= \boldsymbol{u}_\mathrm{i} - (\boldsymbol{k}_\mathrm{p} - \boldsymbol{k}_\mathrm{t})\hat{\boldsymbol{\psi}} \\
    \boldsymbol{u}_\mathrm{s,ref} &= \boldsymbol{k}_\mathrm{t}\left(\boldsymbol{\psi}_{\mathrm{ref}} - \hat{\boldsymbol{\psi}}\right) + \hat{\boldsymbol{v}}_\mathrm{s}
    :label: cc_disturbance

where :math:`\boldsymbol{\alpha}_\mathrm{i} = \boldsymbol{k}_\mathrm{i}/\boldsymbol{k}_\mathrm{t}` is the redefined integral gain and :math:`\hat{\boldsymbol{v}}_\mathrm{s}` is the estimated input disturbance. This structure is convenient to prevent the integral windup that originates from the actuator saturation [#Fra1997]_. The stator voltage is limited in practice due to the limited DC-bus voltage of the converter. Consequently, the realized (limited) voltage reference is

.. math::
    \bar{\boldsymbol{u}}_\mathrm{s,ref} = \mathrm{sat}(\boldsymbol{u}_\mathrm{s,ref})

where :math:`\mathrm{sat}(\cdot)` is the saturation function. The limited voltage can be obtained from a pulse-width modulation (PWM) algorithm. The anti-windup of the integrator can be implemented simply as

.. math::
	\frac{\mathrm{d} \boldsymbol{u}_\mathrm{i}}{\mathrm{d} t} = \boldsymbol{\alpha}_\mathrm{i}\left(\bar{\boldsymbol{u}}_\mathrm{s,ref} - \hat{\boldsymbol{v}}_\mathrm{s}\right)

The other parts of the above controller are not affected by the saturation. The implementation in the :class:`motulator.common.control.ComplexPIController` class is based on this disturbance-observer form.

Synchronous Machines
--------------------

The flux-based control algorithms :eq:`cc_flux` and :eq:`cc_disturbance` can be directly used for both non-salient and salient synchronous machines by mapping the stator current to the flux linkage, [#Awa2019]_

.. math::
    \boldsymbol{\psi}_\mathrm{ref} &= \hat{L}_\mathrm{d}\mathrm{Re}\{\boldsymbol{i}_\mathrm{s,ref}\} + \mathrm{j} \hat{L}_\mathrm{q}\mathrm{Im}\{\boldsymbol{i}_\mathrm{s,ref}\} \\
    \hat{\boldsymbol{\psi}} &= \hat{L}_\mathrm{d}\mathrm{Re}\{\boldsymbol{i}_\mathrm{s}\} + \mathrm{j} \hat{L}_\mathrm{q}\mathrm{Im}\{\boldsymbol{i}_\mathrm{s}\}
    :label: flux_mapping_sm

It is important to notice that :math:`\boldsymbol{i}_\mathrm{s,ref} = \boldsymbol{i}_\mathrm{s}` holds in the steady state even with inductance estimate inaccuracies, since the same inductances are used to map both the reference current and the actual current to the corresponding flux linkages.

Discrete-Time Algorithm
-----------------------

The discrete-time variant of the disturbance-observer form :eq:`cc_disturbance` is given by

.. math::
	\boldsymbol{u}_\mathrm{i}(k+1) &= \boldsymbol{u}_\mathrm{i}(k) + T_\mathrm{s} \boldsymbol{\alpha}_\mathrm{i} \left[\bar{\boldsymbol{u}}_\mathrm{s,ref}(k) - \hat{\boldsymbol{v}}_\mathrm{s}(k) \right] \\
    \hat{\boldsymbol{v}}_\mathrm{s}(k) &= \boldsymbol{u}_\mathrm{i}(k) - (\boldsymbol{k}_\mathrm{p} - \boldsymbol{k}_\mathrm{t})\hat{\boldsymbol{\psi}}(k) \\
    \boldsymbol{u}_\mathrm{s,ref}(k) &= \boldsymbol{k}_\mathrm{t}\left[\boldsymbol{\psi}_{\mathrm{ref}}(k) - \hat{\boldsymbol{\psi}}(k)\right] + \hat{\boldsymbol{v}}_\mathrm{s} \\
     \bar{\boldsymbol{u}}_\mathrm{s,ref}(k) &= \mathrm{sat}\left[\boldsymbol{u}_\mathrm{s,ref}(k)\right]

where :math:`T_\mathrm{s}` is the sampling period and :math:`k` is the discrete-time index. Depending on the machine type, either :eq:`flux_mapping_im` or :eq:`flux_mapping_sm` is used to map the stator current to the flux linkage. This discrete-time algorithm corresponds to the implementation in the :class:`motulator.drive.control.sm.CurrentController` class. The default gain selection corresponds to the complex-vector gains in :eq:`complex_vector_gains_flux`.

.. rubric:: References

.. [#Har1998] Harnefors, Nee, "Model-based current control of AC machines using the internal model control method," IEEE Trans. Ind. Appl., 1998, https://doi.org/10.1109/28.658735

.. [#Bri1999] Briz del Blanco, Degner, Lorenz, “Dynamic analysis of current regulators for AC motors using complex vectors,” IEEE Trans.Ind. Appl., 1999, https://doi.org/10.1109/28.806058

.. [#Awa2019] Awan, Saarakkala, Hinkkanen, "Flux-linkage-based current control of saturated synchronous motors," IEEE Trans. Ind. Appl. 2019, https://doi.org/10.1109/TIA.2019.2919258

.. [#Fra1997] Franklin, Powell, Workman, "Digital Control of Dynamic Systems," 3rd ed., Menlo Park, CA: Addison-Wesley, 1997
