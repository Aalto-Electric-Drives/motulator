Phase-Locked Loop
=================

A phase-locked loop (PLL) is commonly used in grid-following converters to synchronize the converter output with the grid [#Kau1997]_. Here, we represent the PLL using the disturbance observer structure [#Fra1997]_, which may be simpler to extend than the classical PLL. Furthermore, this structure allows to see links to synchronization methods used in grid-forming converters, see [#Nur2024]_.

Disturbance Model
-----------------

Consider the positive-sequence voltage at the point of common coupling (PCC) in general coordinates, rotating at the angular speed :math:`\omega_\mathrm{c}`. The dynamics of the PCC voltage :math:`\boldsymbol{u}_\mathrm{g}` can be modeled using a disturbance model as

.. math::
    \frac{\mathrm{d}\boldsymbol{u}_\mathrm{g}}{\mathrm{d}t} = \mathrm{j}(\omega_\mathrm{g} - \omega_\mathrm{c})\boldsymbol{u}_\mathrm{g}
    :label: disturbance

where :math:`\omega_\mathrm{g}` is the grid angular frequency. If needed, this disturbance model could be extended, e.g., with a negative-sequence component, allowing to design the PLL for unbalanced grids.

.. note:: The disturbance model in :eq:`disturbance` can be equivalently expressed in polar form as

    .. math::
        \frac{\mathrm{d} u_\mathrm{g}}{\mathrm{d} t} &= 0 \\
        \frac{\mathrm{d} \delta_\mathrm{g}}{\mathrm{d} t} &= \omega_\mathrm{g} - \omega_\mathrm{c} \\
        \boldsymbol{u}_\mathrm{g} &= u_\mathrm{g} \mathrm{e}^{\mathrm{j}\delta_\mathrm{g}}

    where :math:`\delta_\mathrm{g} = \vartheta_\mathrm{g} - \vartheta_\mathrm{c}` is the angle of the grid voltage in general coordinates and the last equation is the output equation.

PLL in General Coordinates
--------------------------

Based on :eq:`disturbance`, the disturbance observer containing the regular PLL functionality can be formulated as

.. math::
    \frac{\mathrm{d}\hat{\boldsymbol{u}}_\mathrm{g}}{\mathrm{d}t} = \mathrm{j}(\hat{\omega}_\mathrm{g} - \omega_\mathrm{c})\hat{\boldsymbol{u}}_\mathrm{g} + \alpha_\mathrm{o} (\boldsymbol{u}_\mathrm{g} - \hat{\boldsymbol{u}}_\mathrm{g} )
    :label: pll

where :math:`\hat{\boldsymbol{u}}_\mathrm{g}` is the estimated PCC voltage, :math:`\hat{\omega}_\mathrm{g}` is the grid angular frequency estimate (either constant corresponding to the nominal value or dynamic from grid-frequency tracking), and :math:`\alpha_\mathrm{o}` is the bandwidth. If needed, the disturbance observer can be extended with the frequency tracking as

.. math::
    \frac{\mathrm{d}\hat{\omega}_\mathrm{g}}{\mathrm{d}t} = k_\omega\mathrm{Im}\left\{ \frac{\boldsymbol{u}_\mathrm{g} - \hat{\boldsymbol{u}}_\mathrm{g}}{\hat{\boldsymbol{u}}_\mathrm{g}} \right\}
    :label: frequency_tracking

where :math:`k_\omega` is the frequency-tracking gain. Notice that :eq:`pll` and :eq:`frequency_tracking` are both driven by the estimation error :math:`\boldsymbol{u}_\mathrm{g} - \hat{\boldsymbol{u}}_\mathrm{g}` of the PCC voltage.

PLL in Estimated PCC Voltage Coordinates
----------------------------------------

The disturbance observer :eq:`pll` can be equivalently expressed in estimated PCC voltage coordinates, where :math:`\hat{\boldsymbol{u}}_\mathrm{g} = \hat{u}_\mathrm{g}` and :math:`\omega_\mathrm{c} = \mathrm{d} \hat{\vartheta}_\mathrm{g}/ \mathrm{d} t`, resulting in

.. math::
    \frac{\mathrm{d}\hat{u}_\mathrm{g}}{\mathrm{d}t} &= \alpha_\mathrm{o} \left(\mathrm{Re}\{\boldsymbol{u}_\mathrm{g}\} - \hat{u}_\mathrm{g} \right) \\
    \frac{\mathrm{d} \hat{\vartheta}_\mathrm{g}}{\mathrm{d} t} &= \hat{\omega}_\mathrm{g} + \frac{\alpha_\mathrm{o}}{\hat{u}_\mathrm{g}}\mathrm{Im}\{ \boldsymbol{u}_\mathrm{g} \} = \omega_\mathrm{c}
    :label: pll_polar

It can be seen that the first equation in :eq:`pll_polar` is low-pass filtering of the measured PCC voltage magnitude and the second equation is the conventional angle-tracking PLL. In these coordinates, the frequency tracking :eq:`frequency_tracking` reduces to

.. math::
    \frac{\mathrm{d} \hat{\omega}_\mathrm{g}}{\mathrm{d} t} = \frac{k_\omega}{\hat{u}_\mathrm{g} } \mathrm{Im}\{ \boldsymbol{u}_\mathrm{g} \}
    :label: frequency_tracking_polar

It can be noticed that the disturbance observer with the frequency tracking equals the conventional frequency-adaptive PLL [#Kau1997]_, with the additional feature of low-pass filtering the PCC voltage magnitude. The low-pass filtered PCC voltage can be used as a feedforward term in current control [#Har2021]_.

Linearized Closed-Loop System
-----------------------------

The estimation-error dynamics are analyzed by means of linearization. Using the PCC voltage as an example, the small-signal deviation about the operating point is denoted by :math:`\Delta \boldsymbol{u}_\mathrm{g} = \boldsymbol{u}_\mathrm{g} - \boldsymbol{u}_\mathrm{g0}`, where :math:`\boldsymbol{u}_\mathrm{g0}` is the operating-point quantity. From :eq:`disturbance`--:eq:`frequency_tracking`, the linearized model for the estimation-error dynamics is obtained as

.. math::
    \frac{\mathrm{d}\Delta \tilde{\boldsymbol{u}}_\mathrm{g}}{\mathrm{d}t} &= -\alpha_\mathrm{o}\Delta \tilde{\boldsymbol{u}}_\mathrm{g} + \mathrm{j}\boldsymbol{u}_\mathrm{g0} (\Delta \omega_\mathrm{g} - \Delta \hat{\omega}_\mathrm{g}) \\
    \frac{\mathrm{d}\Delta \hat{\omega}_\mathrm{g}}{\mathrm{d}t} &= k_\omega\mathrm{Im}\left\{ \frac{\Delta \tilde{\boldsymbol{u}}_\mathrm{g}}{\boldsymbol{u}_\mathrm{g0}} \right\}
    :label: linearized_model

where :math:`\Delta \tilde{\boldsymbol{u}}_\mathrm{g} = \Delta\boldsymbol{u}_\mathrm{g} - \Delta \hat{\boldsymbol{u}}_\mathrm{g}` is the estimation error.

First, assume that the grid frequency :math:`\omega_\mathrm{g}` is constant and the frequency tracking is disabled. From :eq:`linearized_model`, the closed-loop transfer function from the PCC voltage to its estimate becomes

.. math::
    \frac{\Delta\hat{\boldsymbol{u}}_\mathrm{g}(s)}{\Delta\boldsymbol{u}_\mathrm{g}(s)} = \frac{\alpha_\mathrm{o}}{s + \alpha_\mathrm{o}}
    :label: closed_loop_pll

It can be realized that both the angle and magnitude of the PCC voltage estimate converge with the bandwidth :math:`\alpha_\mathrm{o}`.

Next, the frequency-tracking dynamics are also considered. From :eq:`linearized_model`, the closed-loop transfer function from the grid angular frequency to its estimate becomes

.. math::
    \frac{\Delta\hat{\omega}_\mathrm{g}(s)}{\Delta\omega_\mathrm{g}(s)}
    = \frac{k_\omega}{s^2 + \alpha_\mathrm{o}s + k_\omega}
    :label: closed_loop_pll_frequency_tracking

Choosing :math:`k_\omega = \alpha_\mathrm{pll}^2` and :math:`\alpha_\mathrm{o} = 2\alpha_\mathrm{pll}` yields the double pole at :math:`s = -\alpha_\mathrm{pll}`, where :math:`\alpha_\mathrm{pll}` is the frequency-tracking bandwidth.

This PLL in estimated PCC coordinates is implemented in the class :class:`motulator.grid.control.PLL`. The :doc:`/grid_examples/grid_following/index` examples use the PLL to synchronize with the grid.

.. rubric:: References

.. [#Kau1997] Kaura, Blasko, "Operation of a phase locked loop system under distorted utility conditions," IEEE Trans. Ind. Appl., 1997, https://doi.org/10.1109/28.567077

.. [#Fra1997] Franklin, Powell, Workman, "Digital Control of Dynamic Systems," 3rd ed., Menlo Park, CA: Addison-Wesley, 1997

.. [#Nur2024] Nurminen, Mourouvin, Hinkkanen, Kukkola, "Multifunctional grid-forming converter control based on a disturbance observer, "IEEE Trans. Power Electron., 2024, https://doi.org/10.1109/TPEL.2024.3433503

.. [#Har2021] Harnefors, Kukkola, Routimo, Hinkkanen, Wang, "A universal controller for grid-connected voltage-source converters," IEEE J. Emerg. Sel. Topics Power Electron., 2021, https://doi.org/10.1109/JESTPE.2020.3039407
