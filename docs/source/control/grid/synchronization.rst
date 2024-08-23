Synchronization Methods
=======================

Phase-Locked Loop
-----------------

The :doc:`/grid_examples/grid_following/index` examples use a Phase-Locked Loop (PLL) to synchronize with the grid. The block diagram of the PLL is shown in the figure below.

.. figure:: ../figs/pll.svg
   :width: 100%
   :align: center
   :alt: Block diagram of the phase-locked loop for grid synchronization
   :target: .

   Block diagram of the phase-locked loop.

The PLL drives the signal :math:`\hat{u}_\mathrm{gq}` to zero, leading to :math:`\hat{\vartheta}_\mathrm{g}=\vartheta_\mathrm{g}` and :math:`\hat{u}_\mathrm{gd}=u_\mathrm{gd}` in ideal conditions. The grid voltage-vector :math:`\boldsymbol{u}_\mathrm{g}^\mathrm{s}=u_\mathrm{g} \mathrm{e}^{\mathrm{j} \vartheta_\mathrm{g}}` is measured. The angle :math:`\vartheta_\mathrm{g}` can be noisy and it is not directly used in the control. Instead, the PLL tracks :math:`\vartheta_{g}` and filters its noise and harmonics above the PLL bandwidth.

The gain selection:

.. math:: 
    k_\mathrm{p} = \frac{2 \zeta \omega_\mathrm{0,PLL}}{u_\mathrm{gN}} \qquad
    k_\mathrm{i} = \frac{\omega_\mathrm{0,PLL}^2}{u_\mathrm{gN}}

where :math:`\zeta` is the damping factor, :math:`\omega_\mathrm{0,PLL}` is the natural frequency of the PLL, and :math:`u_\mathrm{gN}` is the nominal grid voltage amplitude.

More details on the control methods used can be found in [#Kau1997]_.

This controller is implemented in the class :class:`motulator.grid.control.PLL`.

Power Synchronization
---------------------

In :doc:`/grid_examples/grid_forming/plot_gfm_rfpsc_13kva` a power synchronization loop (PSL) is used as a means of synchronizing with the grid. The dynamics of a synchronous machine are emulated, as the converter output active power is tied to the angle of the converter output voltage. This allows for synchronization of a converter with the grid without the use of a PLL. More details on the control method used can be found in [#Har2020]_.

The PSL is implemented as

.. math::
    \frac{\mathrm{d}\vartheta_\mathrm{c}}{\mathrm{d}t} = \omega_\mathrm{g} + k_\mathrm{p} (p_\mathrm{g,ref} - p_\mathrm{g})
    :label: psl

where :math:`\vartheta_\mathrm{c}` is the converter output voltage angle, :math:`\omega_\mathrm{g}` the nominal grid angular frequency and :math:`k_\mathrm{p}` the active power control gain. Furthermore, :math:`p_\mathrm{g,ref}` and :math:`p_\mathrm{g}` are the reference and realized value for the converter active power output, respectively. The active power output is calculated from the measured converter current and the realized converter output voltage obtained from the PWM.

Disturbance Observer
--------------------

In :doc:`/grid_examples/grid_forming/plot_gfm_obs_13kva`, a disturbance observer-based control method is used, where the disturbance observer provides grid synchronization [#Nur2024]_. The observer is implemented in coordinates rotating at angular frequency :math:`\omega_\mathrm{c}` as

.. math::
    \frac{\mathrm{d}\hat{\boldsymbol{v}}_\mathrm{c}'}{\mathrm{d}t} &= (\boldsymbol{k}_\mathrm{o}
    + \mathrm{j}\hat{\omega}_\mathrm{g})\boldsymbol{e}_\mathrm{c} + \mathrm{j}(\hat{\omega}_\mathrm{g}
    - \omega_\mathrm{c})\hat{\boldsymbol{v}}_\mathrm{c}'\\
    \hat{\boldsymbol{v}}_\mathrm{c} &= \hat{\boldsymbol{v}}_\mathrm{c}'
    - \boldsymbol{k}_\mathrm{o}\hat{L}\boldsymbol{i}_\mathrm{c}
    :label: gfm_obs_eqs

where :math:`\hat{\boldsymbol{v}}_\mathrm{c}` is the quasi-static converter output voltage estimate, :math:`\boldsymbol{k}_\mathrm{o}` the observer gain, :math:`\hat{\omega}_\mathrm{g}` the grid angular frequency estimate, :math:`\boldsymbol{e}_\mathrm{c}` the feedback correction term, :math:`\hat{L}` the total inductance estimate, and :math:`\boldsymbol{i}_\mathrm{c}` the measured converter current.

.. rubric:: References

.. [#Kau1997] Kaura and Blasko, "Operation of a phase locked loop system under distorted utility conditions," in IEEE Trans. Ind. Appl., vol. 33, no. 1, pp. 58-63, Jan.-Feb. 1997, https://doi.org/10.1109/28.567077

.. [#Har2020] Harnefors, Rahman, Hinkkanen, Routimo, "Reference-Feedforward Power-Synchronization Control," IEEE Trans. Power Electron., Sep. 2020, https://doi.org/10.1109/TPEL.2020.2970991

.. [#Nur2024] Nurminen, Mourouvin, Hinkkanen, Kukkola, "Multifunctional grid-forming converter control based on a disturbance observer, "IEEE Trans. Power Electron., 2024, https://doi.org/10.1109/TPEL.2024.3433503