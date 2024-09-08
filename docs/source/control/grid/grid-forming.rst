Disturbance-Observer-Based Grid-Forming Control
===============================================

In these notes, disturbance-observer-based grid-forming control is discussed [#Nur2024]_. This control method is available in the :class:`motulator.grid.control.ObserverBasedGridFormingControl` class. As can be realized by comparing the examples :doc:`/grid_examples/grid_forming/plot_gfm_rfpsc_13kva` and :doc:`/grid_examples/grid_forming/plot_gfm_obs_13kva`, the observer-based grid-forming control can be configured to behave practically identically with reference-feedforward power-synchronization control (PSC) [#Har2020]_. Compared to reference-feedforward PSC, disturbance-observer-based grid-forming control is easier to analyze and extend with different operating modes (including grid-following and transparent current-control modes). The order of these two control methods is the same and their computational complexity is comparable.

System Model
------------

First, the system is modeled in general coordinates, whose angle with respect to the stationary coordinates is :math:`\vartheta_\mathrm{c}` and the angular speed is :math:`\omega_\mathrm{c} = \mathrm{d}\vartheta_\mathrm{c}/\mathrm{d} t`. The dynamics of the inductor current :math:`\boldsymbol{i}_\mathrm{c}` and the grid voltage :math:`\boldsymbol{u}_\mathrm{g}` are modeled as

.. math::
    L\frac{\mathrm{d} \boldsymbol{i}_\mathrm{c}}{\mathrm{d} t} &= \boldsymbol{u}_\mathrm{c} - \boldsymbol{u}_\mathrm{g} - \mathrm{j}\omega_\mathrm{c} L \boldsymbol{i}_\mathrm{c} \\
    \frac{\mathrm{d}\boldsymbol{u}_\mathrm{g}}{\mathrm{d} t} &= \mathrm{j}(\omega_\mathrm{g} - \omega_\mathrm{c})\boldsymbol{u}_\mathrm{g}
    :label: system_model1

where :math:`L` is the inductance and :math:`\omega_\mathrm{g}` is the grid angular frequency. The grid voltage :math:`\boldsymbol{u}_\mathrm{g}` is modeled as a disturbance; this same disturbance model for the grid voltage is also used in the development of :doc:`/control/grid/pll`. The active power fed to the grid is nonlinear in the state variables

.. math::
    p_\mathrm{g} = \frac{3}{2}\mathrm{Re}\{\boldsymbol{u}_\mathrm{g}\boldsymbol{i}_\mathrm{c}^*\}
    :label: power1

For the purpose of grid-forming control, we may define the quasi-static converter voltage as an output variable as a function of the state variables,

.. math::
    \boldsymbol{v}_\mathrm{c} = \boldsymbol{u}_\mathrm{g} + \mathrm{j}\omega_\mathrm{g} L \boldsymbol{i}_\mathrm{c}
    :label: quasi_static_converter_voltage

This voltage is controllable (unlike the grid voltage which is the disturbance) and it equals the converter output voltage :math:`\boldsymbol{u}_\mathrm{c}` in the steady state. The disturbance-observer-based PSC regulates the voltage magnitude :math:`|\boldsymbol{v}_\mathrm{c}|` and the power :math:`p_\mathrm{g}` fed to grid.

Control System
--------------

The control system consists of a disturbance observer and a control law. The disturbance observer estimates the quasi-static converter voltage and the power fed to the grid. The control law generates the converter voltage reference based on the estimated grid voltage and power. The disturbance observer also provides the integral action for the control law. This control structure allows seamless switching between grid-forming and grid-following modes [#Nur2024]_. For simplicity, only the grid-forming mode is considered in the following.

Disturbance Observer
^^^^^^^^^^^^^^^^^^^^

Based on :eq:`system_model1`--:eq:`quasi_static_converter_voltage`, a disturbance observer for the grid voltage can be formed [#Nur2024]_, [#Kuk2021]_, [#Fra1997]_

.. math::
    \frac{\mathrm{d} \hat{\boldsymbol{u}}_\mathrm{g}}{\mathrm{d} t} &= \mathrm{j} (\hat{\omega}_\mathrm{g} - \omega_\mathrm{c})\hat{\boldsymbol{u}}_\mathrm{g} + \alpha_\mathrm{o}\left(\boldsymbol{u}_\mathrm{c} - \hat L \frac{\mathrm{d} \boldsymbol{i}_\mathrm{c}}{\mathrm{d} t} - \mathrm{j} \omega_\mathrm{c} \hat L \boldsymbol{i}_\mathrm{c} - \hat{\boldsymbol{u}}_\mathrm{g} \right) \\
    \hat{\boldsymbol{v}}_\mathrm{c} &= \hat{\boldsymbol{u}}_\mathrm{g} + \mathrm{j}\hat{\omega}_\mathrm{g} \hat L \boldsymbol{i}_\mathrm{c} \\
    \hat p_\mathrm{g} &= \frac{3}{2}\mathrm{Re}\{\hat{\boldsymbol{v}}_\mathrm{c}\boldsymbol{i}_\mathrm{c}^*\}
    :label: disturbance_observer_gfm

where :math:`\hat \omega_\mathrm{g}` is the nominal grid angular frequency and estimates are marked with hat.

.. note:: A conventional phase-locked loop (PLL) can be expressed in the same disturbance observer framework, see the :doc:`/control/grid/pll` notes. It can be realized that the measured grid voltage :math:`\boldsymbol{u}_\mathrm{g}` used in the conventional PLL is replaced by its converter-voltage-based estimate :math:`\boldsymbol{u}_\mathrm{c} - \hat L (\mathrm{d} \boldsymbol{i}_\mathrm{c}/\mathrm{d} t) - \mathrm{j} \omega_\mathrm{c} \hat L \boldsymbol{i}_\mathrm{c}` in the disturbance observer :eq:`disturbance_observer_gfm`.

Control Law
^^^^^^^^^^^

A nonlinear state feedback law is used

.. math::
    \boldsymbol{u}_\mathrm{c,ref} = \hat{\boldsymbol{v}}_\mathrm{c} + \boldsymbol{k}_\mathrm{p} (p_\mathrm{g,ref} - \hat p_\mathrm{g}) + \boldsymbol{k}_\mathrm{v} (v_\mathrm{c,ref} - |\hat{\boldsymbol{v}}_\mathrm{c}|)
    :label: control_law_gfm

where :math:`p_\mathrm{g,ref}` is the active power reference, :math:`v_\mathrm{c,ref}` is the converter voltage magnitude reference, and :math:`\boldsymbol{k}_\mathrm{p}` and :math:`\boldsymbol{k}_\mathrm{v}` are the complex gains for the active-power and converter-voltage-magnitude channels, respectively. To ensure robust operation, similar to the reference-feedforward PSC, the complex gains can be selected as

.. math::
    \boldsymbol{k}_\mathrm{p} = \frac{R_\mathrm{a}}{v_\mathrm{c,ref}} \frac{\hat{\boldsymbol{v}}_\mathrm{c}}{|\hat{\boldsymbol{v}}_\mathrm{c}|} \qquad
    \boldsymbol{k}_\mathrm{v} = (1 - \mathrm{j} k_\mathrm{v}) \frac{\hat{\boldsymbol{v}}_\mathrm{c}}{|\hat{\boldsymbol{v}}_\mathrm{c}|}
    :label: gain_selection_gfm

where the gains :math:`R_\mathrm{a} = 0.2` p.u. and :math:`k_\mathrm{v} = \alpha_\mathrm{o}/\omega_\mathrm{g}` can be used.

Implementation Aspects
^^^^^^^^^^^^^^^^^^^^^^

To avoid the derivate on the right-hand side of :eq:`disturbance_observer_gfm`, a new state variable :math:`\hat{\boldsymbol{u}}_\mathrm{g}' = \hat{\boldsymbol{u}}_\mathrm{g} + \alpha_\mathrm{o} \hat L \boldsymbol{i}_\mathrm{c}` can be introduced [#Fra1997]_. Furthermore, the coordinate system for the implementation can be chosen freely. The simplest choice is to use the nominal grid frequency as the coordinate system frequency, :math:`\omega_\mathrm{c} = \hat \omega_\mathrm{g}`. Using these design choices, the whole control system consisting of the disturbance observer :eq:`disturbance_observer_gfm` and the control law :eq:`control_law_gfm` in the state-space form reduces to [#Nur2024]_

.. math::
    \frac{\mathrm{d} \hat{\boldsymbol{u}}'_\mathrm{g}}{\mathrm{d} t} &= \alpha_\mathrm{o} (\boldsymbol{u}_\mathrm{c,ref} - \hat{\boldsymbol{v}}_\mathrm{c} ) \\
    \hat{\boldsymbol{v}}_\mathrm{c} &= \hat{\boldsymbol{u}}_\mathrm{g}' - (\alpha_\mathrm{o} - \mathrm{j}\hat{\omega}_\mathrm{g}) \hat L \boldsymbol{i}_\mathrm{c} \\
    \hat p_\mathrm{g} &= \frac{3}{2}\mathrm{Re}\{\hat{\boldsymbol{v}}_\mathrm{c}\boldsymbol{i}_\mathrm{c}^*\} \\
    \boldsymbol{u}_\mathrm{c,ref} &= \hat{\boldsymbol{v}}_\mathrm{c} + \boldsymbol{k}_\mathrm{p} (p_\mathrm{g,ref} - \hat p_\mathrm{g}) + \boldsymbol{k}_\mathrm{v} (v_\mathrm{c,ref} - |\hat{\boldsymbol{v}}_\mathrm{c}|)
    :label: control_system_gfm

where the gains can be selected according to :eq:`gain_selection_gfm` and the converter voltage appearing in the observer has been replaced with its reference. Various control modes could be easily incorporated into the control system :eq:`control_system_gfm`, simply by changing the feedback correction terms of the control law [#Nur2024]_. The switching between the modes is seamless since the control law does not have memory, but the integral action is provided by the disturbance observer (in addition to synchronization).

The control system implemented in the :class:`motulator.grid.control.ObserverBasedGridFormingControl` class corresponds to :eq:`control_system_gfm`. In the example implementation, a transparent current-control mode is implemented. In the grid-forming mode, the observer bandwidth :math:`\alpha_\mathrm{o} = 1` p.u. can be used. Furthermore, the inductance estimate can be set close to the lowest expected inductance value, e.g., :math:`\hat L = 0.15` p.u. Using this configuration, the robust performance from strong grids to very weak grids can be achieved. This grid-forming control method can also be used with LCL filters, similarly to reference-feedforward PSC.

.. rubric:: References

.. [#Nur2024] Nurminen, Mourouvin, Hinkkanen, Kukkola, "Multifunctional grid-forming converter control based on a disturbance observer, "IEEE Trans. Power Electron., 2024, https://doi.org/10.1109/TPEL.2024.3433503

.. [#Har2020] Harnefors, Rahman, Hinkkanen, Routimo, "Reference-feedforward power-synchronization control," IEEE Trans. Power Electron., 2020, https://doi.org/10.1109/TPEL.2020.2970991

.. [#Kuk2021] Kukkola, Routimo, Hinkkanen, Harnefors, "A voltage-sensorless controller for grid converters," IEEE PES ISGT Europe, 2021, https://doi.org/10.1109/ISGTEurope52324.2021.9640206

.. [#Fra1997] Franklin, Powell, Workman, "Digital Control of Dynamic Systems," 3rd ed., Menlo Park, CA: Addison-Wesley, 1997
