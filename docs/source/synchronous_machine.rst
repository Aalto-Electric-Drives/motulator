Synchronous Machine
===================

A continuous-time synchronous machine model is in the package :mod:`motulator.model.sm`. The model can be parametrized to represent permanent-magnet synchronous machines (PMSMs) and synchronous reluctance machines (SyRMs). Peak-valued complex space vectors are used. 

.. figure:: figs/sm_block_rot.svg
   :width: 100%
   :align: center
   :alt: Synchronous machine model
   :target: .

   Block diagram of the machine model in rotor coordinates. The magnetic model includes the flux equation (or, optionally, saturation characteristics) and the torque equation.

The voltage equation in rotor coordinates is [Jah1986]_

.. math::
    \frac{\mathrm{d}\boldsymbol{\psi}_\mathrm{s}}{\mathrm{d} t} = \boldsymbol{u}_\mathrm{s} - R_\mathrm{s}\boldsymbol{i}_\mathrm{s} - \mathrm{j}\omega_\mathrm{m}\boldsymbol{\psi}_\mathrm{s} 

where :math:`\boldsymbol{u}_\mathrm{s}` is the stator voltage, :math:`\boldsymbol{i}_\mathrm{s}` is the stator current, and :math:`R_\mathrm{s}` is the stator resistance. The electrical angular speed of the rotor is :math:`\omega_\mathrm{m} = n_\mathrm{p}\omega_\mathrm{M}`, where :math:`\omega_\mathrm{M}` is the mechanical angular speed of the rotor and :math:`n_\mathrm{p}` is the number of pole pairs. In the magnetically linear case, the stator flux linkage is 

.. math::
	\boldsymbol{\psi}_\mathrm{s} = L_\mathrm{d}\mathrm{Re}\{\boldsymbol{i}_\mathrm{s}\} + \mathrm{j}L_\mathrm{q}\mathrm{Im}\{\boldsymbol{i}_\mathrm{s}\} + \psi_\mathrm{f} 

where :math:`L_\mathrm{d}` is the d-axis inductance, :math:`L_\mathrm{q}` is the q-axis inductance, :math:`\psi_\mathrm{f}` is the permanent-magnet (PM) flux linkage. As special cases, this model represents a surface-PMSM with :math:`L_\mathrm{d} = L_\mathrm{q}` and SyRM with :math:`\psi_\mathrm{f}=0`.

.. note::
    The linear magnetic model can be replaced with nonlinear saturation characteristics :math:`\boldsymbol{\psi}_\mathrm{s} = \boldsymbol{\psi}_\mathrm{s}(\boldsymbol{i}_\mathrm{s})`, either in a form of flux maps or explicit functions [Hin2017]_, see :class:`motulator.model.sm.SynchronousMachineSaturated`. The module :mod:`motulator.model.sm.flux_maps` contains methods for importing and plotting the flux map data. See also the examples :doc:`/auto_examples/flux_maps/plot_obs_vhz_ctrl_pmsyrm_thor` and :doc:`/auto_examples/vhz/plot_obs_vhz_ctrl_syrm_7kw`.

The electromagnetic torque is

.. math::
    \tau_\mathrm{M} = \frac{3 n_\mathrm{p}}{2}\mathrm{Im} \left\{\boldsymbol{i}_\mathrm{s} \boldsymbol{\psi}_\mathrm{s}^* \right\}

Since the machine is fed and observed from stator coordinates, the quantities are transformed accordingly, as shown in the figure below. The mechanical subsystem closes the loop from :math:`\tau_\mathrm{M}` to :math:`\omega_\mathrm{M}`, see the module :mod:`motulator.model.mechanics`.

.. figure:: figs/sm_block_stat.svg
   :width: 100%
   :align: center
   :alt: Synchronous machine model seen from stator coordinates
   :target: .

   Synchronous machine model seen from stator coordinates.

References
----------

.. [Jah1986] Jahns, Kliman, Neumann, “Interior permanent-magnet synchronous motors for adjustable-speed drives,” IEEE Trans. Ind. Appl., 1986, https://doi.org/10.1109/TIA.1986.4504786

.. [Hin2017] Hinkkanen, Pescetto, Mölsä, Saarakkala, Pellegrino, Bojoi, “Sensorless self-commissioning of synchronous reluctance motors at standstill without rotor locking, ”IEEE Trans. Ind. Appl., 2017, https://doi.org/10.1109/TIA.2016.2644624

