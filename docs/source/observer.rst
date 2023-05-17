Reduced-Order Observer
======================

Under construction...

In electric machine drives, an observer is commonly used to estimate the state of an electrical machine. In motion-sensored drives, the state of interest is typically the flux linkage. In sensorless induction machine drives, the rotor speed estimate (and the rotor position estimate if a synchronous machine is considered) is needed as well. 

In the following, reduced-order observer designs for the induction machine are considered [Ver1988]_, [Har2001]_, [Hin2010]_. Similar design principles are applied to synchronous machines as well.

System Model
------------

The inverse-Γ model of an induction machine is considered (see :doc:`induction_machine`). In a coordinate system rotating at the angular speed :math:`\omega_\mathrm{s}`, the rotor flux dynamics can be expressed using the stator and rotor quantities, respectively, as

.. math::
	\frac{\mathrm{d} \boldsymbol{\psi}_\mathrm{R}}{\mathrm{d} t} + \mathrm{j}\omega_\mathrm{s}\boldsymbol{\psi}_\mathrm{R} &= \boldsymbol{u}_\mathrm{s} - R_\mathrm{s}\boldsymbol{i}_\mathrm{s} - L_\sigma \frac{\mathrm{d} \boldsymbol{i}_\mathrm{s}}{\mathrm{d} t} - \mathrm{j} \omega_\mathrm{s}L_\sigma\boldsymbol{i}_\mathrm{s} \\
    &= R_\mathrm{R}\boldsymbol{i}_\mathrm{s} - \left(\alpha - \mathrm{j}\omega_\mathrm{m} \right)\boldsymbol{\psi}_\mathrm{R}
    :label: dpsiR

where :math:`\omega_\mathrm{m}` is the electrical rotor angular speed. 

General Coordinates
-------------------

Based on :eq:`dpsiR`, a reduced-order observer is formulated,

.. math::
    \frac{\mathrm{d} \hat{\boldsymbol{\psi}}_\mathrm{R}}{\mathrm{d} t} + \mathrm{j}\omega_\mathrm{s}\hat{\boldsymbol{\psi}}_\mathrm{R} = \boldsymbol{v} + \boldsymbol{k}_1(\hat{\boldsymbol{v}} - \boldsymbol{v}) + \boldsymbol{k}_2(\hat{\boldsymbol{v}} - \boldsymbol{v})^* 
    :label: dhatpsiR
    
where :math:`^*` marks the complex conjugate and the back-emf estimates are 

.. math::
    \boldsymbol{v} &= \boldsymbol{u}_\mathrm{s} - R_\mathrm{s}\boldsymbol{i}_\mathrm{s} - L_\sigma \frac{\mathrm{d} \boldsymbol{i}_\mathrm{s}}{\mathrm{d} t} - \mathrm{j} \omega_\mathrm{s}L_\sigma\boldsymbol{i}_\mathrm{s} \\
	\hat{\boldsymbol{v}} &= R_\mathrm{R}\boldsymbol{i}_\mathrm{s} - \left(\alpha - \mathrm{j}\hat{\omega}_\mathrm{m} \right)\hat{\boldsymbol{\psi}}_\mathrm{R}
    :label: hatv

In sensored drives, the rotor speed estimate is replaced with the measured speed,  :math:`\hat{\omega}_\mathrm{m} = \omega_\mathrm{m}`. If needed, the estimates for the stator flux linkage and the electromagnetic torque, respectively, are

.. math::
    \hat{\boldsymbol{\psi}}_\mathrm{s} &= \hat{\boldsymbol{\psi}}_\mathrm{R} + L_\sigma \boldsymbol{i}_\mathrm{s} \\
    \hat{\tau}_\mathrm{M} &= \frac{3n_\mathrm{p}}{2}\mathrm{Im}\left\{\boldsymbol{i}_\mathrm{s} \hat{\boldsymbol{\psi}}_\mathrm{R}^* \right\}
    :label: tauM

Choosing :math:`\boldsymbol{k}_2 = 0` results in the reduced-order observer proposed in [Ver1988]_, [Har2001]_, suitable for sensored drives. As a special case, :math:`\boldsymbol{k}_1 = \boldsymbol{k}_2 = 0` yields the voltage model. Another special case is the current model, which is obtained choosing :math:`\boldsymbol{k}_1 = 1` and :math:`\boldsymbol{k}_2 = 0`. For sensorless drives, choosing :math:`\boldsymbol{k}_1 = \boldsymbol{k}_2` leads to an inherently sensorless observer, i.e., the rotor speed :math:`\omega_\mathrm{m}` cancels out from the observer equations [Hin2010]_.

.. note::
    Real-valued column vectors and the corresponding :math:`2\times 2` gain matrix were used in [Hin2010]_. The complex form in :eq:`dhatpsiR` has the same degrees of freedom.

Estimated Flux Coordinates
--------------------------

The flux estimate in stator coordinates can be expressed using the polar form, :math:`\hat{\boldsymbol{\psi}}_\mathrm{R}^\mathrm{s} = \hat{\psi}_\mathrm{R}\mathrm{e}^{\mathrm{j}\hat{\vartheta}_\mathrm{s}}`, where :math:`\hat{\psi}_\mathrm{R}` is the magnitude and :math:`\hat{\vartheta}_\mathrm{s}` is the angle of the estimate. The observer is convenient to implement in estimated rotor flux coordinates, in which the rotor flux estimate is real, :math:`\hat{\boldsymbol{\psi}}_\mathrm{R} = \hat{\psi}_\mathrm{R} + \mathrm{j}0`. Under these conditions, the angular speed of the coordinates system can be solved from :eq:`dhatpsiR` as

.. math::
    \frac{\mathrm{d}\hat{\vartheta}_\mathrm{s}}{\mathrm{d} t} = \omega_\mathrm{s}
    = \frac{\mathrm{Im} \{ \boldsymbol{v}' + \boldsymbol{k}_1(\hat{\boldsymbol{v}} - \boldsymbol{v}') + \boldsymbol{k}_2(\hat{\boldsymbol{v}} - \boldsymbol{v}')^* \} }{\hat{\psi}_\mathrm{R} + L_\sigma \mathrm{Re}\{(1 - \boldsymbol{k}_1)\boldsymbol{i}_\mathrm{s} + \boldsymbol{k}_2 \boldsymbol{i}_\mathrm{s}^* \}} 
    :label: hatws

where 

.. math::
    \boldsymbol{v}' = \boldsymbol{u}_\mathrm{s} - R_\mathrm{s}\boldsymbol{i}_\mathrm{s} - L_\sigma \frac{\mathrm{d} \boldsymbol{i}_\mathrm{s}}{\mathrm{d} t} 
    :label: vp

The flux magnitude dynamics are 

.. math::
    \frac{\mathrm{d} \hat{\psi}_\mathrm{R}}{\mathrm{d} t} 
    = \mathrm{Re}\{ \boldsymbol{v} + \boldsymbol{k}_1(\hat{\boldsymbol{v}} - \boldsymbol{v}) + \boldsymbol{k}_2(\hat{\boldsymbol{v}} - \boldsymbol{v})^* \}
    :label: dhatpsiR_abs

Notice that the right-hand side of :eq:`hatws` is independent of :math:`\omega_\mathrm{s}`. Furthermore, the derivative of the stator current is integrated, i.e., the noise is not amplified. This observer structure is implemented in the :class:`motulator.control.im.Observer`, where simple forward-Euler discretization is used. 

Estimation-Error Dynamics   
-------------------------

The estimation-error dynamics are obtained by subtracting :eq:`dhatpsiR` from :eq:`dpsiR`. Furthermore, this system can be linearized for analysis purposes. Using the rotor speed as an exmaple, the small-signal deviation about the operating point is :math:`\Delta \omega_\mathrm{m} = \omega_\mathrm{m} - \omega_\mathrm{m0}`, where the subscript 0 refers to the operating point. Linearization of the estimation-error dynamics leads to [Hin2010]_

.. math::
	\frac{\mathrm{d} \Delta\tilde{\boldsymbol{\psi}}_\mathrm{R}}{\mathrm{d} t} =  \boldsymbol{k}_1\Delta \tilde{\boldsymbol{v}} + \boldsymbol{k}_2\Delta \tilde{\boldsymbol{v}}^* - \mathrm{j}\omega_\mathrm{s0}\Delta\tilde{\boldsymbol{\psi}}_\mathrm{R}
    :label: dtildepsiR

where the estimation error of the rotor flux is :math:`\Delta\tilde{\boldsymbol{\psi}}_\mathrm{R} = \Delta\boldsymbol{\psi}_\mathrm{R} - \Delta\hat{\boldsymbol{\psi}}_\mathrm{R}` and other estimation errors are marked similarly. Furthermore, the back-emf estimation error is

.. math::
    \Delta\tilde{\boldsymbol{v}} = -\left(\alpha - \mathrm{j}\omega_\mathrm{m0} \right)\Delta\tilde{\boldsymbol{\psi}}_\mathrm{R} + \mathrm{j}\boldsymbol{\psi}_\mathrm{R0}\Delta\tilde{\omega}_\mathrm{m}
    :label: dtildev

Sensored Case
^^^^^^^^^^^^^

In sensored drives, :math:`\Delta\tilde{\omega}_\mathrm{m} = 0` and :math:`\boldsymbol{k}_2 = 0` is a feasible choice, resulting in

.. math::
	\frac{\mathrm{d} \Delta\tilde{\boldsymbol{\psi}}_\mathrm{R}}{\mathrm{d} t} =  -\left[\boldsymbol{k}_1\left(\alpha - \mathrm{j}\omega_\mathrm{m0} \right) + \mathrm{j}\omega_\mathrm{s0}\right]\Delta\tilde{\boldsymbol{\psi}}_\mathrm{R}
    :label: dtildepsiR_sensored

The complex closed-loop pole could be arbitrarily placed by means of the gain :math:`\boldsymbol{k}_1`. Well-damped estimation-error dynamics can be obtained, e.g., by choosing

.. math:: 
    \boldsymbol{k}_1 = 1 + \frac{g |\omega_\mathrm{m}|}{\alpha - \mathrm{j}\omega_\mathrm{m}} \qquad \boldsymbol{k}_2 = 0
    :label: k1k2_sensored

where :math:`g` is a unitless positive design parameter. The corresponding pole is located at :math:`s = - g |\omega_\mathrm{m0}| -\alpha - \mathrm{j}\omega_\mathrm{r0}`, where :math:`\omega_\mathrm{r0} = \omega_\mathrm{s0} - \omega_\mathrm{m0}` is the operating-point slip angular frequency.

Sensorless Case
^^^^^^^^^^^^^^^

In sensorless drives, choosing :math:`\boldsymbol{k}_1 = \boldsymbol{k}_2 = k_\mathrm{d} + \mathrm{j}k_\mathrm{q}` results in [Hin2010]_

.. math::
	\frac{\mathrm{d}}{\mathrm{d} t} \begin{bmatrix} \Delta\tilde{\psi}_\mathrm{Rd} \\ \Delta\tilde{\psi}_\mathrm{Rq} \end{bmatrix} = \begin{bmatrix} -2k_\mathrm{d}\alpha & -2k_\mathrm{d}\omega_\mathrm{m0} + \omega_\mathrm{s0} \\ -2k_\mathrm{q}\alpha - \omega_\mathrm{s0} & -2k_\mathrm{q}\omega_\mathrm{m0} 
     \end{bmatrix} \begin{bmatrix} \Delta\tilde{\psi}_\mathrm{Rd} \\ \Delta\tilde{\psi}_\mathrm{Rq} \end{bmatrix}
    :label: dtildepsiR_sensorless

It can be seen that the dynamics of the rotor speed are decoupled from the flux-estimation error dynamics. The decay rate :math:`\sigma` be freely assigned by choosing the gains

.. math::
    \boldsymbol{k}_1 = \boldsymbol{k}_2 = \frac{\sigma}{\alpha - \mathrm{j}\hat\omega_\mathrm{m}}
    :label: k1k2_sensorless

which result in the characteristic polynomial :math:`s^2 + 2\sigma s + \omega_\mathrm{s0}^2`.

References
----------

.. [Ver1988] Verghese, Sanders, “Observers for flux estimation in induction machines,” IEEE Trans. Ind. Electron., 1988, https://doi.org/10.1109/41.3067

.. [Har2001] Harnefors, “Design and analysis of general rotor-flux-oriented vector control systems,” IEEE Trans. Ind. Electron., 2001, https://doi.org/10.1109/41.915417

.. [Hin2010] Hinkkanen, Harnefors, Luomi, "Reduced-order flux observers with stator-resistance adaptation for speed-sensorless induction motor drives," IEEE Trans. Power Electron., 2010, https://doi.org/10.1109/TPEL.2009.2039650



