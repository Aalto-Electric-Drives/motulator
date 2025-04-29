Flux-Vector Control
===================

Flux vector control directly controls the stator flux magnitude and the electromagnetic torque, which simplifies reference generation especially in the field-weakening region [#Pel2009]_. We use the control law that results from feedback linearization of the flux and torque dynamics [#Awa2019]_. Here, the inclusion of integral action is briefly discussed, as an extension to [#Tii2025a]_, [#Tii2025b]_. Furthermore, these notes aim to provide a link between the column-vector and complex-vector formulations of the control law. Here, a synchronous machine is considered as an example (see the :ref:`synchronous-machine` document for the model and notation), but the approach is analogous for induction machines.

Flux and Torque Dynamics
------------------------

From the machine model, the flux and torque dynamics can be written as

.. math::
    \frac{\mathrm{d} \psi_\mathrm{s}}{\mathrm{d} t} &= \frac{1}{|\boldsymbol{\psi}_\mathrm{s}|} \mathrm{Re} \{\left(\boldsymbol{u}_\mathrm{s} - R_\mathrm{s} \boldsymbol{i}_\mathrm{s} - \mathrm{j}\omega_\mathrm{m}\boldsymbol{\psi}_\mathrm{s}\right) \boldsymbol{\psi}_\mathrm{s}^* \} \\
    \frac{\mathrm{d}\tau_\mathrm{M}}{\mathrm{d} t} &= \frac{3 n_\mathrm{p}}{2} \mathrm{Im} \{ \left(\boldsymbol{u}_\mathrm{s} - R_\mathrm{s} \boldsymbol{i}_\mathrm{s} - \mathrm{j}\omega_\mathrm{m}\boldsymbol{\psi}_\mathrm{s} \right) \boldsymbol{i}_\mathrm{a}^*\}
    :label: flux_torque

These expressions are valid also for saturated machines, when the auxiliary current :math:`\boldsymbol{i}_\mathrm{a}` is defined according to :eq:`sm_mtpv_aux` in the :ref:`synchronous-machine` document.

Proportional Control
--------------------

Consider proportional control of the form, resulting from the feedback linearization, [#Tii2025a]_

.. math::
	\boldsymbol{u}_\mathrm{s,ref} = \hat R_\mathrm{s} \boldsymbol{i}_\mathrm{s} + \mathrm{j}\omega_\mathrm{m} \hat{\boldsymbol{\psi}}_\mathrm{s} + e_\psi \boldsymbol{t}_{\psi} + e_\tau\boldsymbol{t}_{\tau}
    :label: flux_vector_control_usref

where :math:`\hat R_\mathrm{s}` is the stator resistance estimate, :math:`\hat{\boldsymbol{\psi}}_\mathrm{s}` is the estimated flux vector, and :math:`e_\psi = \alpha_\psi(\psi_\mathrm{s,ref} - |\hat{\boldsymbol{\psi}}_\mathrm{s}|)` and :math:`e_\tau = \alpha_\tau(\tau_\mathrm{M,ref} - \hat{\tau}_\mathrm{M})` are the control errors in the flux and torque, respectively, scaled by the respective bandwidths. The complex quantities :math:`\boldsymbol{t}_{\psi}` and :math:`\boldsymbol{t}_{\tau}` are the direction vectors.

.. math::
	\boldsymbol{t}_{\psi} = \frac{ |\hat{\boldsymbol{\psi}}_\mathrm{s}|}{ \mathrm{Re}\{\hat{\boldsymbol{\psi}}_\mathrm{s}\hat{i}_\mathrm{a}^*\}}\hat{i}_\mathrm{a} \qquad
	\boldsymbol{t}_{\tau} = \frac{2}{3 n_\mathrm{p} \mathrm{Re}\{\hat{\boldsymbol{\psi}}_\mathrm{s}\hat{i}_\mathrm{a}^*\}}\mathrm{j}\hat{\boldsymbol{\psi}}_\mathrm{s}
    :label: flux_vector_control_dir

Notice that :math:`\mathrm{Re}\{\hat{\boldsymbol{\psi}}_\mathrm{s}\hat{i}_\mathrm{a}^*\} > 0` in the whole feasible operating region.

Assuming accurate estimates, linearization of the closed-loop system consisting of :eq:`flux_torque`--:eq:`flux_vector_control_dir` results in

.. math::
	\frac{\mathrm{d}\Delta \psi_\mathrm{s}}{\mathrm{d} t} = \alpha_\psi (\Delta \psi_\mathrm{s,ref} - \Delta \psi_\mathrm{s} ) \qquad
	\frac{\mathrm{d}\Delta\tau_\mathrm{M}}{\mathrm{d} t} = \alpha_\tau(\Delta\tau_\mathrm{M,ref} - \Delta \tau_\mathrm{M})
    :label: flux_vector_control_closed_lin

These closed-loop small-signal dynamics are valid also for saturated machines, if the auxiliary current is estimated based on :eq:`sm_mtpv_aux`.

Inclusion of Integral Action
----------------------------

Consider a 2DOF PI control law consisting of :eq:`flux_vector_control_usref` and :eq:`flux_vector_control_dir` and the following integral states and control error terms,

.. math::
	\frac{\mathrm{d} x_\psi}{\mathrm{d} t} &= \alpha_\psi\alpha_\mathrm{i}(\psi_\mathrm{s,ref} - |\hat{\boldsymbol{\psi}}_\mathrm{s}|) \\
	\frac{\mathrm{d} x_\tau}{\mathrm{d} t} &= \alpha_\tau\alpha_\mathrm{i}(\tau_\mathrm{M,ref} - \hat{\tau}_\mathrm{M}) \\
	e_\psi &= \alpha_\psi(\psi_\mathrm{s,ref} - |\hat{\boldsymbol{\psi}}_\mathrm{s}|) + x_\psi - \alpha_\mathrm{i} |\hat{\boldsymbol{\psi}}_\mathrm{s}| \\
    e_\tau &= \alpha_\tau(\tau_\mathrm{M,ref} - \hat{\tau}_\mathrm{M}) + x_\tau - \alpha_\mathrm{i} \hat{\tau}_\mathrm{M}
    :label: flux_vector_control_2dof

where :math:`\alpha_\psi` and :math:`\alpha_\tau` are the reference-tracking bandwidths, :math:`\alpha_\psi` and :math:`\alpha_\tau` are the integral states, and :math:`\alpha_\mathrm{i}` is the integral action bandwidth. It can be shown that the resulting linearized closed-loop reference-following dynamics remain the same as  :eq:`flux_vector_control_closed_lin`. However, in this case, both the flux and torque dynamics have additional pole at :math:`s = - \alpha_\mathrm{i}` resulting from the integral action. The integral action pole is canceled from reference-tracking dynamics (see the :ref:`2dof-pi-controller` document).

This control law is a variant of [#Awa2019]_, having channel-specific reference-tracking bandwidths. It has been implemented in the :class:`motulator.drive.control.sm.FluxVectorController` and :class:`motulator.drive.control.im.FluxVectorController` classes for synchronous and induction machines, respectively. The disturbance observer structure is used in the implementation to avoid the need for anti-windup mechanism (see the :ref:`disturbance-observer-structure` document).

.. rubric:: References

.. [#Pel2009] Pellegrino, Armando, Guglielmi, “Direct flux field-oriented control of IPM drives with variable DC link in the field-weakening region,” IEEE Trans. Ind. Appl., 2009, https://doi.org/10.1109/TIA.2009.2027167

.. [#Awa2019] Awan, Hinkkanen, Bojoi, Pellegrino, "Stator-flux-oriented control of synchronous motors: A systematic design procedure," IEEE Trans. Ind. Appl., 2019, https://doi.org/10.1109/TIA.2019.2927316

.. [#Tii2025a] Tiitinen, Hinkkanen, Harnefors, "Design framework for sensorless control of synchronous machine drives," IEEE Trans. Ind. Electron., 2025, https://doi.org/10.1109/TIE.2024.3429650

.. [#Tii2025b] Tiitinen, Hinkkanen, Harnefors, "Sensorless flux-vector control framework: An extension for induction machines," IEEE Trans. Ind. Electron., 2025, https://doi.org/10.1109/TIE.2025.3559958
