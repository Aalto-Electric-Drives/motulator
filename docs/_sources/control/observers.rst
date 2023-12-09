Observer Design
===============

An observer is commonly used to estimate the state of an electrical machine. In motion-sensored drives, the state of interest is typically the flux linkage. In sensorless drives, the rotor speed estimate is needed as well. Furthermore, sensorless synchronous machine drives require estimation of the rotor position.  

Induction Machines
------------------

In the following, reduced-order observer designs for the induction machine are considered [#Ver1988]_, [#Har2001]_, [#Hin2010]_. Both sensored and sensorless observer variants are available in the :class:`motulator.control.im.Observer` class. Similar design principles are applied to synchronous machines as well.

Machine Model
^^^^^^^^^^^^^

The inverse-Γ model of an induction machine is considered (see :doc:`/model/machines`). In a coordinate system rotating at the angular speed :math:`\omega_\mathrm{s}`, the rotor flux dynamics can be expressed using the stator and rotor quantities, respectively, as

.. math::
	\frac{\mathrm{d} \boldsymbol{\psi}_\mathrm{R}}{\mathrm{d} t} + \mathrm{j}\omega_\mathrm{s}\boldsymbol{\psi}_\mathrm{R} &= \boldsymbol{u}_\mathrm{s} - R_\mathrm{s}\boldsymbol{i}_\mathrm{s} - L_\sigma \frac{\mathrm{d} \boldsymbol{i}_\mathrm{s}}{\mathrm{d} t} - \mathrm{j} \omega_\mathrm{s}L_\sigma\boldsymbol{i}_\mathrm{s} \\
    &= R_\mathrm{R}\boldsymbol{i}_\mathrm{s} - \left(\alpha - \mathrm{j}\omega_\mathrm{m} \right)\boldsymbol{\psi}_\mathrm{R}
    :label: dpsiR

where :math:`\omega_\mathrm{m}` is the electrical rotor angular speed. 

Observer Structure
^^^^^^^^^^^^^^^^^^

General Coordinates
"""""""""""""""""""

Based on :eq:`dpsiR`, a reduced-order observer can be formulated in a coordinate system rotating at :math:`\omega_\mathrm{s}`,

.. math::
    \frac{\mathrm{d} \hat{\boldsymbol{\psi}}_\mathrm{R}}{\mathrm{d} t} + \mathrm{j}\omega_\mathrm{s}\hat{\boldsymbol{\psi}}_\mathrm{R} = \boldsymbol{v} + \boldsymbol{k}_1(\hat{\boldsymbol{v}} - \boldsymbol{v}) + \boldsymbol{k}_2(\hat{\boldsymbol{v}} - \boldsymbol{v})^* 
    :label: dhatpsiR
    
where :math:`\boldsymbol{k}_1` and :math:`\boldsymbol{k}_2` are complex gains, the estimates are marked with the hat, and :math:`^*` marks the complex conjugate. The back-emf estimates are 

.. math::
    \boldsymbol{v} &= \boldsymbol{u}_\mathrm{s} - R_\mathrm{s}\boldsymbol{i}_\mathrm{s} - L_\sigma \frac{\mathrm{d} \boldsymbol{i}_\mathrm{s}}{\mathrm{d} t} - \mathrm{j} \omega_\mathrm{s}L_\sigma\boldsymbol{i}_\mathrm{s} \\
	\hat{\boldsymbol{v}} &= R_\mathrm{R}\boldsymbol{i}_\mathrm{s} - \left(\alpha - \mathrm{j}\hat{\omega}_\mathrm{m} \right)\hat{\boldsymbol{\psi}}_\mathrm{R}
    :label: hatv

In sensored drives, the rotor speed estimate is replaced with the measured speed,  :math:`\hat{\omega}_\mathrm{m} = \omega_\mathrm{m}`. If needed, the estimates for the stator flux linkage and the electromagnetic torque, respectively, are

.. math::
    \hat{\boldsymbol{\psi}}_\mathrm{s} &= \hat{\boldsymbol{\psi}}_\mathrm{R} + L_\sigma \boldsymbol{i}_\mathrm{s} \\
    \hat{\tau}_\mathrm{M} &= \frac{3n_\mathrm{p}}{2}\mathrm{Im}\left\{\boldsymbol{i}_\mathrm{s} \hat{\boldsymbol{\psi}}_\mathrm{R}^* \right\}
    :label: tauM

It is also worth noticing that the derivative of the stator current in :eq:`hatv` is integrated, i.e., the noise is not amplified.

.. note::
    Real-valued column vectors and the corresponding :math:`2\times 2` gain matrix were used in [#Hin2010]_. The complex form in :eq:`dhatpsiR` has the same degrees of freedom.

Estimated Flux Coordinates
""""""""""""""""""""""""""

The flux estimate in stator coordinates can be expressed using the polar form, :math:`\hat{\boldsymbol{\psi}}_\mathrm{R}^\mathrm{s} = \hat{\psi}_\mathrm{R}\mathrm{e}^{\mathrm{j}\hat{\vartheta}_\mathrm{s}}`, where :math:`\hat{\psi}_\mathrm{R}` is the magnitude and :math:`\hat{\vartheta}_\mathrm{s}` is the angle of the estimate. The observer is convenient to implement in estimated rotor flux coordinates, in which the rotor flux estimate is real, :math:`\hat{\boldsymbol{\psi}}_\mathrm{R} = \hat{\psi}_\mathrm{R} + \mathrm{j}0` [#Har2001]_. Under these conditions, the angular speed of the coordinates system can be solved from :eq:`dhatpsiR` as

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

Notice that the right-hand side of :eq:`hatws` is independent of :math:`\omega_\mathrm{s}`. Futhermore, in these coordinates, the condition :eq:`inherently` for an inherently sensorless observer reduces to :math:`\boldsymbol{k}_2 = \boldsymbol{k}_1`. This observer structure is implemented in the :class:`motulator.control.im.Observer`, where simple forward-Euler discretization is used. 

Gain Selection
^^^^^^^^^^^^^^

The estimation-error dynamics are obtained by subtracting :eq:`dhatpsiR` from :eq:`dpsiR`. The resulting system can be linearized for analysis and gain selection purposes. Using the rotor speed as an exmaple, the small-signal deviation about the operating point is :math:`\Delta \omega_\mathrm{m} = \omega_\mathrm{m} - \omega_\mathrm{m0}`, where the subscript 0 refers to the operating point. Linearization of the estimation-error dynamics leads to [#Hin2010]_

.. math::
	\frac{\mathrm{d} \Delta\tilde{\boldsymbol{\psi}}_\mathrm{R}}{\mathrm{d} t} =  \boldsymbol{k}_1\Delta \tilde{\boldsymbol{v}} + \boldsymbol{k}_2\Delta \tilde{\boldsymbol{v}}^* - \mathrm{j}\omega_\mathrm{s0}\Delta\tilde{\boldsymbol{\psi}}_\mathrm{R}
    :label: dtildepsiR

where the estimation error of the rotor flux is :math:`\Delta\tilde{\boldsymbol{\psi}}_\mathrm{R} = \Delta\boldsymbol{\psi}_\mathrm{R} - \Delta\hat{\boldsymbol{\psi}}_\mathrm{R}` and other estimation errors are marked similarly. Furthermore, the back-emf estimation error is

.. math::
    \Delta\tilde{\boldsymbol{v}} = -\left(\alpha - \mathrm{j}\omega_\mathrm{m0} \right)\Delta\tilde{\boldsymbol{\psi}}_\mathrm{R} + \mathrm{j}\boldsymbol{\psi}_\mathrm{R0}\Delta\tilde{\omega}_\mathrm{m}
    :label: dtildev

Sensored Case
"""""""""""""

Here, :math:`\boldsymbol{k}_2 = 0` and :math:`\Delta\tilde{\omega}_\mathrm{m} = 0` are assumed, corresponding to the sensored reduced-order observer in [#Ver1988]_. Under these assumptions, the estimation-error dynamics in :eq:`dtildepsiR` reduce to 

.. math::
	\frac{\mathrm{d} \Delta\tilde{\boldsymbol{\psi}}_\mathrm{R}}{\mathrm{d} t} =  -\left[\boldsymbol{k}_1\left(\alpha - \mathrm{j}\omega_\mathrm{m0} \right) + \mathrm{j}\omega_\mathrm{s0}\right]\Delta\tilde{\boldsymbol{\psi}}_\mathrm{R}
    :label: dtildepsiR_sensored

It can be noticed that the closed-loop pole could be arbitrarily placed via the gain :math:`\boldsymbol{k}_1`. Well-damped estimation-error dynamics can be obtained, e.g., by choosing

.. math:: 
    \boldsymbol{k}_1 = 1 + \frac{g |\omega_\mathrm{m}|}{\alpha - \mathrm{j}\omega_\mathrm{m}} 
    :label: k1_sensored

where :math:`g` is a unitless positive design parameter. The corresponding pole is located at :math:`s = -\alpha - g |\omega_\mathrm{m0}| - \mathrm{j}\omega_\mathrm{r0}`, where :math:`\omega_\mathrm{r0} = \omega_\mathrm{s0} - \omega_\mathrm{m0}` is the operating-point slip angular frequency.

.. note::

    As a special case, :math:`\boldsymbol{k}_1  = 0` yields the voltage model. As another special case, the current model is obtained choosing :math:`\boldsymbol{k}_1 = 1`. 

Sensorless Case
"""""""""""""""

For sensorless drives, choosing 

.. math::
    \boldsymbol{k}_2 = (\hat{\boldsymbol{\psi}}_\mathrm{R}/\hat{\boldsymbol{\psi}}_\mathrm{R}^*) \boldsymbol{k}_1
    :label: inherently
    
yields an inherently sensorless observer, i.e., the rotor speed estimate :math:`\hat{\omega}_\mathrm{m}` cancels out from the observer equations [#Hin2010]_. Under this condition, the linearized estimation-error dynamics in :eq:`dtildepsiR` become

.. math::
	\frac{\mathrm{d}}{\mathrm{d} t} \begin{bmatrix} \Delta\tilde{\psi}_\mathrm{Rd} \\ \Delta\tilde{\psi}_\mathrm{Rq} \end{bmatrix} = \begin{bmatrix} -2k_\mathrm{d}\alpha & -2k_\mathrm{d}\omega_\mathrm{m0} + \omega_\mathrm{s0} \\ -2k_\mathrm{q}\alpha - \omega_\mathrm{s0} & -2k_\mathrm{q}\omega_\mathrm{m0} 
     \end{bmatrix} \begin{bmatrix} \Delta\tilde{\psi}_\mathrm{Rd} \\ \Delta\tilde{\psi}_\mathrm{Rq} \end{bmatrix}
    :label: dtildepsiR_sensorless

where the gain components correspond to :math:`\boldsymbol{k}_1 = k_\mathrm{d} + \mathrm{j}k_\mathrm{q}`. It can be seen that the dynamics of the rotor speed are decoupled from the flux-estimation error dynamics. The decay rate :math:`\sigma` be assigned by choosing

.. math::
    \boldsymbol{k}_1 = \frac{\sigma}{\alpha - \mathrm{j}\hat\omega_\mathrm{m}}
    :label: k1_sensorless

which results in the characteristic polynomial :math:`D(s)=s^2 + 2\sigma s + \omega_\mathrm{s0}^2`. The decay rate can be selected as :math:`\sigma = \alpha/2 + \zeta_\infty|\hat{\omega}_\mathrm{m}|`, where :math:`\zeta_\infty` is the desired damping ratio at high speed. At zero stator frequency :math:`\omega_\mathrm{s0} = 0`, the poles are located at :math:`s = 0` and :math:`s = -\alpha`, which allows stable magnetizing and starting the machine.

.. rubric:: References

.. [#Ver1988] Verghese, Sanders, “Observers for flux estimation in induction machines,” IEEE Trans. Ind. Electron., 1988, https://doi.org/10.1109/41.3067

.. [#Har2001] Harnefors, “Design and analysis of general rotor-flux-oriented vector control systems,” IEEE Trans. Ind. Electron., 2001, https://doi.org/10.1109/41.915417

.. [#Hin2010] Hinkkanen, Harnefors, Luomi, "Reduced-order flux observers with stator-resistance adaptation for speed-sensorless induction motor drives," IEEE Trans. Power Electron., 2010, https://doi.org/10.1109/TPEL.2009.2039650


Synchronous Machines
--------------------

In sensorless control of synchronous machine drives, the rotor position and speed estimates are needed [#Jon1989]_, [#Cap2001]_, [#Hin2018]_. As a side product, the stator flux linkage is also estimated. In the following, an observer design available in the :class:`motulator.control.sm.Observer` class is considered, which is based on [#Hin2018]_. This observer implementation also includes a sensored mode. 


Machine Model in General Coordinates
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In :doc:`/model/machines`, the synchronous machine model is given in rotor coordinates. For the observer design and analysis, it is convenient to express this model in general coordinates, aligned at :math:`\vartheta_\mathrm{s}` and rotating at :math:`\omega_\mathrm{s} = \mathrm{d} \vartheta_\mathrm{s}/\mathrm{d} t` with respect to stator coordinates. Furthermore, the rotor is aligned at :math:`\vartheta_\mathrm{m}` and rotates at :math:`\omega_\mathrm{m} = \mathrm{d} \vartheta_\mathrm{m}/\mathrm{d} t` with respect to stator coordinates. This coordinate transformation results in 

.. math::
    \frac{\mathrm{d}\boldsymbol{\psi}_\mathrm{s}}{\mathrm{d} t} &= \boldsymbol{u}_\mathrm{s} - R_\mathrm{s}\boldsymbol{i}_\mathrm{s} - \mathrm{j}\omega_\mathrm{s}\boldsymbol{\psi}_\mathrm{s} \\
    \frac{\mathrm{d}\delta}{\mathrm{d} t} &= \omega_\mathrm{m} - \omega_\mathrm{s}  
    :label: sm

where :math:`\boldsymbol{u}_\mathrm{s}` is the stator voltage, :math:`\boldsymbol{i}_\mathrm{s}` is the stator current, and :math:`\delta = \vartheta_\mathrm{m} - \vartheta_\mathrm{s}` is the electrical angle of the rotor as seen from the general coordinate system. Assuming linear magnetics, the relation between the stator flux linkage and the stator current is governed by 

.. math::
	\boldsymbol{\psi}_\mathrm{s} = \mathrm{e}^{\mathrm{j}\delta}\left(L_\mathrm{d}\mathrm{Re}\{\boldsymbol{i}_\mathrm{s} \mathrm{e}^{-\mathrm{j}\delta}\} + \mathrm{j}L_\mathrm{q}\mathrm{Im}\{\boldsymbol{i}_\mathrm{s}\mathrm{e}^{-\mathrm{j}\delta}\} + \psi_\mathrm{f}\right)
    :label: sm_flux_gen 

where :math:`L_\mathrm{d}` is the d-axis inductance, :math:`L_\mathrm{q}` is the q-axis inductance, and :math:`\psi_\mathrm{f}` is the PM flux linkage. 

Notice that setting :math:`\vartheta_\mathrm{s}=0` yields the machine model in stator coordinates. In the following, the coordinate system will be fixed to the estimated angle of the rotor, i.e., to the coordinate system used by the control system.  

Observer Structure
^^^^^^^^^^^^^^^^^^

The observer is assumed to operate in estimated rotor coordinates, whose d-axis is aligned with the rotor angle estimate :math:`\hat{\vartheta}_\mathrm{m}`. Now, the angle :math:`\delta = \vartheta_\mathrm{m} - \hat{\vartheta}_\mathrm{m}` in the machine model :eq:`sm` corresponds to the estimation error of the rotor angle, which naturally is unknown to the sensorless control system. 

Since the stator current is measured, the observer is fundamentally corrected by means of the current estimation error. However, due to the saliency, it is more convenient to scale the current estimation error by the stator inductance, resulting in the flux linkage error   

.. math::
	\boldsymbol{e} = \psi_\mathrm{f} + L_\mathrm{d} \mathrm{Re}\{ \boldsymbol{i}_\mathrm{s}\} + \mathrm{j} L_\mathrm{q} \mathrm{Im}\{\boldsymbol{i}_\mathrm{s} \} - \hat{\boldsymbol{\psi}}_\mathrm{s} 
    :label: e

where :math:`\hat{\boldsymbol{\psi}}_\mathrm{s}` is the stator flux estimate. The flux linkage is estimated by 

.. math::
    \frac{\mathrm{d} \hat{\boldsymbol{\psi}}_\mathrm{s}}{\mathrm{d} t} = \boldsymbol{u}_\mathrm{s} - R_\mathrm{s}\boldsymbol{i}_\mathrm{s} - \mathrm{j}\omega_\mathrm{s}\hat{\boldsymbol{\psi}}_\mathrm{s} + \boldsymbol{k}_1 \boldsymbol{e} + \boldsymbol{k}_2 \boldsymbol{e}^* 
    :label: sm_flux_observer

where :math:`\boldsymbol{k}_1` and :math:`\boldsymbol{k}_2` are gains (complex in a general case), the estimates are marked with the hat, and :math:`^*` marks the complex conjugate. 

In the sensored mode, :math:`\omega_\mathrm{s} = \omega_\mathrm{m}` is used. In the sensorless mode, the speed-adaptive structure (which would correspond to the phase-locked loop if the observer were implemented in stator coordinates) can be used to estimate the rotor angle and speed, respectively, as

.. math::
    \frac{\mathrm{d} \hat{\omega}_\mathrm{m}}{\mathrm{d} t} &= \mathrm{Im}\{\boldsymbol{k}_\mathrm{i} \boldsymbol{e}\} \\
    \frac{\mathrm{d}\hat{\vartheta}_\mathrm{m}}{\mathrm{d} t} &= \hat{\omega}_\mathrm{m} + \mathrm{Im}\{\boldsymbol{k}_\mathrm{p} \boldsymbol{e}\} = \omega_\mathrm{s}  
    :label: sm_speed_pos_observer

where :math:`\boldsymbol{k}_\mathrm{i}` and :math:`\boldsymbol{k}_\mathrm{p}` are complex gains. This observer structure is used in the :class:`motulator.control.sm.Observer` class. 

.. note::
    Real-valued column vectors and the corresponding :math:`2\times 2` gain matrix were used in [#Hin2018]_. The complex form in :eq:`sm_flux_observer` has the same degrees of freedom.

.. eps = -np.imag(e/psi_a) if np.abs(psi_a) > 0 else 0
.. w_s = self.k_p*eps + self.w_m
.. self.w_m += T_s*self.k_i*eps

Gain Selection
^^^^^^^^^^^^^^

Sensored Case
"""""""""""""

In the sensored case, the gain :math:`\boldsymbol{k}_2=0` can be set in :eq:`sm_flux_observer`. Furthermore, :math:`\delta=0` holds. Therefore, using :eq:`sm` and :eq:`sm_flux_observer`, the linearized estimation error dynamics become 

.. math::
    \frac{\mathrm{d} \Delta\tilde{\boldsymbol{\psi}}_\mathrm{s}}{\mathrm{d} t} = -(\boldsymbol{k}_1 + \mathrm{j}\omega_\mathrm{m0})\Delta\tilde{\boldsymbol{\psi}}_\mathrm{s}  
    :label: dtildepsis_sensored

where :math:`\tilde{\boldsymbol{\psi}}_\mathrm{s} = \boldsymbol{\psi}_\mathrm{s} - \hat{\boldsymbol{\psi}}_\mathrm{s}` is the estimation error, :math:`\Delta` marks the small-signal quantities, and the subscript 0 marks the operating-point quantities. It can be seen that the pole can be arbitrarily placed via the gain :math:`\boldsymbol{k}_1`. Well-damped estimation-error dynamics can be obtained simply by using a real gain, :math:`\boldsymbol{k}_1 = \sigma`, resulting in the pole at :math:`s = -\sigma - \mathrm{j}\omega_\mathrm{m0}`, where :math:`\sigma = 2\pi \cdot 15` rad/s is used as the default value in :class:`motulator.control.sm.Observer`. 

Sensorless Case
"""""""""""""""

The analysis of the sensorless case is more complicated. Here, the main results are summarized using the complex notation. The following results can be derived from the linearized form of :eq:`sm` -- :eq:`sm_speed_pos_observer`, see further details in [#Hin2018]_

To decouple the flux estimation from the rotor angle, the gains of :eq:`sm_flux_observer` have to be of the form

.. math::
	\boldsymbol{k}_1 = \sigma \qquad \boldsymbol{k}_2 = \frac{\sigma\hat{\boldsymbol{\psi}}_\mathrm{a}}{\hat{\boldsymbol{\psi}}_\mathrm{a}^*}
    :label: k1k2_sensorless

where :math:`\sigma` is the desired decay rate of the flux estimation error and

.. math::
    \hat{\boldsymbol{\psi}}_\mathrm{a} = \psi_\mathrm{f} + (L_\mathrm{d} - L_\mathrm{q}) \boldsymbol{i}_\mathrm{s}^*
    :label: sm_aux_flux

allows to decouple the flux-estimation error dynamics from the rotor-position dynamics. By default, the decay rate is scheduled as

.. math:: 
    \sigma = \frac{R_\mathrm{s}}{4}\left(\frac{1}{L_\mathrm{d}} + \frac{1}{L_\mathrm{q}}\right) + \zeta_\infty |\hat{\omega}_\mathrm{m} |
    :label: sigma_sensorless

where :math:`\zeta_\infty` is the desired damping ratio at high speed. At zero speed, :eq:`sigma_sensorless` places one pole at :math:`s = 0` and another at :math:`s = -(R_\mathrm{s}/2)(1/L_\mathrm{d} + 1/L_\mathrm{q})`.

The gains of the speed adaptation in :eq:`sm_speed_pos_observer` are selected as

.. math::
	\boldsymbol{k}_\mathrm{i} = -\frac{\alpha_\mathrm{o}^2}{\hat{\boldsymbol{\psi}}_\mathrm{a}} \qquad \boldsymbol{k}_\mathrm{p} = -\frac{2\alpha_\mathrm{o}}{\hat{\boldsymbol{\psi}}_\mathrm{a}}
    :label: ki_kp_sensorless

where :math:`\alpha_\mathrm{o}` is the desired speed-estimation bandwidth. The choices :eq:`k1k2_sensorless` and :eq:`ki_kp_sensorless` result in the observer characteristic polynomial :math:`D(s) = (s^2 + 2\sigma s + \omega_\mathrm{m0}^2)(s + \alpha_\mathrm{o})^2`. Furthermore, it can also be shown that the resulting speed-estimation error dynamics are

.. math:: 
    \frac{\Delta \hat{\omega}_\mathrm{m}(s)}{\Delta \omega_\mathrm{m}(s)} = \frac{\alpha_\mathrm{o}^2}{(s + \alpha_\mathrm{o})^2}
    :label: speed_est_dyn

.. note:: 
    The flux linkage :math:`\boldsymbol{\psi}_\mathrm{a}` is called the auxiliary flux linkage in [#Hin2018]_. It is also linked to the maximum-torque-per-ampere (MTPA) condition, which can be compactly expressed as :math:`\mathrm{Re}\{\boldsymbol{i}_\mathrm{s}\boldsymbol{\psi}_\mathrm{a}^*\}=0` [#Var2021]_. 

.. rubric:: References

.. [#Jon1989] Jones, Lang, “A state observer for the permanent-magnet synchronous motor,” IEEE Trans. Ind. Electron., 1989, https://doi.org/10.1109/41.31500

.. [#Cap2001] Capecchi, Guglielmo, Pastorelli, Vagati, “Position-sensorless control of the transverse-laminated synchronous reluctance motor,” IEEE Trans. Ind. Appl., 2001, https://doi.org/10.1109/28.968190

.. [#Hin2018] Hinkkanen, Saarakkala, Awan, Mölsä, Tuovinen, "Observers for sensorless synchronous motor drives: Framework for design and analysis," IEEE Trans. Ind. Appl., 2018, https://doi.org/10.1109/TIA.2018.2858753

.. [#Var2021] Varatharajan, Pellegrino, Armando, “Direct flux vector control of synchronous motor drives: A small-signal model for optimal reference generation,” IEEE Trans. Power Electron., 2021, https://doi.org/10.1109/TPEL.2021.3067694
