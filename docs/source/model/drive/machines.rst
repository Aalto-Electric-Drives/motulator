Electric Machines
=================

This document describes continuous-time electric machine models.

Induction Machine
-----------------

The induction machine models are provided in the package :mod:`motulator.drive.model`. The models are implemented in stator coordinates. A Γ-equivalent model is used as a base model since it can be extended with the magnetic saturation model in a straightforward manner [#Sle1989]_.

.. figure:: ../figs/im_gamma.svg
    :figclass: only-light
    :width: 100%
    :align: center
    :alt: Gamma-model of an induction machine
    :target: .

    Γ model.

.. figure:: ../figs/im_gamma.svg
    :figclass: invert-colors-dark only-dark
    :width: 100%
    :align: center
    :alt: Gamma-model of an induction machine
    :target: .

    Γ model.

.. figure:: ../figs/im_block.svg
    :figclass: only-light
    :width: 100%
    :align: center
    :alt: Block diagram of an induction machine model
    :target: .

    Block diagram of the machine model. The magnetic model includes the flux equations (or, optionally, saturation characteristics) and the torque equation.

.. figure:: ../figs/im_block.svg
    :figclass: invert-colors-dark only-dark
    :width: 100%
    :align: center
    :alt: Block diagram of an induction machine model
    :target: .

    Block diagram of the machine model. The magnetic model includes the flux equations (or, optionally, saturation characteristics) and the torque equation.

The voltage equations are

.. math::
    \frac{\mathrm{d}\boldsymbol{\psi}_\mathrm{s}^\mathrm{s}}{\mathrm{d} t} &= \boldsymbol{u}_\mathrm{s}^\mathrm{s} - R_\mathrm{s}\boldsymbol{i}_\mathrm{s}^\mathrm{s} \\
    \frac{\mathrm{d}\boldsymbol{\psi}_\mathrm{r}^\mathrm{s}}{\mathrm{d} t} &= -R_\mathrm{r}\boldsymbol{i}_\mathrm{r}^\mathrm{s} + \mathrm{j}\omega_\mathrm{m}\boldsymbol{\psi}_\mathrm{r}^\mathrm{s}
    :label: im_voltage

where :math:`\boldsymbol{u}_\mathrm{s}^\mathrm{s}` is the stator voltage, :math:`\boldsymbol{i}_\mathrm{s}^\mathrm{s}` is the stator current, :math:`\boldsymbol{i}_\mathrm{r}^\mathrm{s}` is the rotor current, :math:`R_\mathrm{s}` is the stator resistance, and :math:`R_\mathrm{r}` is the rotor resistance. The electrical angular speed of the rotor is :math:`\omega_\mathrm{m} = n_\mathrm{p}\omega_\mathrm{M}`, where :math:`\omega_\mathrm{M}` is the mechanical angular speed of the rotor and :math:`n_\mathrm{p}` is the number of pole pairs. The stator flux linkage :math:`\boldsymbol{\psi}_\mathrm{s}^\mathrm{s}` and the rotor flux linkage :math:`\boldsymbol{\psi}_\mathrm{r}^\mathrm{s}`, respectively, are

.. math::
    \boldsymbol{\psi}_\mathrm{s}^\mathrm{s} &= L_\mathrm{s}(\boldsymbol{i}_\mathrm{s}^\mathrm{s} + \boldsymbol{i}_\mathrm{r}^\mathrm{s} ) \\
    \boldsymbol{\psi}_\mathrm{r}^\mathrm{s} &= \boldsymbol{\psi}_\mathrm{s}^\mathrm{s} + L_\ell\boldsymbol{i}_\mathrm{r}^\mathrm{s}
    :label: im_flux

where :math:`L_\mathrm{s}` is the stator inductance and :math:`L_\ell` is the leakage inductance. This linear magnetic model is applied in the class :class:`motulator.drive.model.InductionMachine`. Its parameters are defined in the data class :class:`motulator.drive.model.InductionMachinePars`. The electromagnetic torque is

.. math::
    \tau_\mathrm{M} = \frac{3 n_\mathrm{p}}{2}\mathrm{Im} \left\{\boldsymbol{i}_\mathrm{s}^\mathrm{s} (\boldsymbol{\psi}_\mathrm{s}^\mathrm{s})^* \right\}
    :label: im_torque

The same class can also be used with the main-flux saturation models, such as :math:`L_\mathrm{s} = L_\mathrm{s}(\psi_\mathrm{s})` [#Qu2012]_. See also the example :doc:`/drive_examples/current_vector/plot_vector_ctrl_im_2kw`.

.. note::
    If the magnetic saturation is omitted, the Γ model is mathematically identical to the inverse-Γ and T models. For example, the parameters of the Γ model can be transformed to those of the inverse-Γ model parameters as follows:

    .. math::
        L_\sigma &= \left(\frac{L_\mathrm{s}}{L_\mathrm{s} + L_\ell}\right)L_\ell \\
        R_\mathrm{R} &= \left(\frac{L_\mathrm{s}}{L_\mathrm{s} + L_\ell}\right)^2 R_\mathrm{r} \\
        L_\mathrm{M} &=  L_\mathrm{s} - L_\sigma

    The inverse-Γ model parameters can be given using the data class :class:`motulator.drive.model.InductionMachineInvGammaPars`. Unlike the Γ model, the inverse-Γ model does not support the magnetic saturation model.

    .. figure:: ../figs/im_inv_gamma.svg
        :figclass: only-light
        :width: 100%
        :align: center
        :alt: Inverse-Gamma model of an induction machine
        :target: .

        Inverse-Γ model.

    .. figure:: ../figs/im_inv_gamma.svg
        :figclass: invert-colors-dark only-dark
        :width: 100%
        :align: center
        :alt: Inverse-Gamma model of an induction machine
        :target: .

        Inverse-Γ model.

    Example control methods in the package :mod:`motulator.drive.control.im` are based on the inverse-Γ model.

.. _synchronous-machine:

Synchronous Machine
-------------------

Synchronous machine models are provided in the package :mod:`motulator.drive.model`. The models can be parametrized to represent permanent-magnet synchronous machines (PMSMs) and synchronous reluctance machines (SyRMs).

.. figure:: ../figs/sm_block_rot.svg
    :figclass: only-light
    :width: 100%
    :align: center
    :alt: Synchronous machine model
    :target: .

    Block diagram of the machine model in rotor coordinates. The magnetic model includes the flux equation (or, optionally, saturation characteristics) and the torque equation.

.. figure:: ../figs/sm_block_rot.svg
    :figclass: invert-colors-dark only-dark
    :width: 100%
    :align: center
    :alt: Synchronous machine model
    :target: .

    Block diagram of the machine model in rotor coordinates. The magnetic model includes the flux equation (or, optionally, saturation characteristics) and the torque equation.

The voltage equation in rotor coordinates is [#Jah1986]_

.. math::
    \frac{\mathrm{d}\boldsymbol{\psi}_\mathrm{s}}{\mathrm{d} t} = \boldsymbol{u}_\mathrm{s} - R_\mathrm{s}\boldsymbol{i}_\mathrm{s} - \mathrm{j}\omega_\mathrm{m}\boldsymbol{\psi}_\mathrm{s}
    :label: sm_voltage

where :math:`\boldsymbol{u}_\mathrm{s}` is the stator voltage and :math:`\boldsymbol{i}_\mathrm{s}` is the stator current. In the magnetically linear case, the stator flux linkage is

.. math::
	\boldsymbol{\psi}_\mathrm{s} = L_\mathrm{d}\mathrm{Re}\{\boldsymbol{i}_\mathrm{s}\} + \mathrm{j}L_\mathrm{q}\mathrm{Im}\{\boldsymbol{i}_\mathrm{s}\} + \psi_\mathrm{f}
    :label: sm_flux

where :math:`L_\mathrm{d}` is the d-axis inductance, :math:`L_\mathrm{q}` is the q-axis inductance, :math:`\psi_\mathrm{f}` is the permanent-magnet (PM) flux linkage. As special cases, this model represents a surface-PMSM with :math:`L_\mathrm{d} = L_\mathrm{q}` and SyRM with :math:`\psi_\mathrm{f}=0`.

The electromagnetic torque is

.. math::
    \tau_\mathrm{M} = \frac{3 n_\mathrm{p}}{2}\mathrm{Im} \left\{\boldsymbol{i}_\mathrm{s} \boldsymbol{\psi}_\mathrm{s}^* \right\}
    :label: sm_torque

Since the machine is fed and observed from stator coordinates, the quantities are transformed accordingly, as shown in the figure below. The mechanical subsystem closes the loop from :math:`\tau_\mathrm{M}` to :math:`\omega_\mathrm{M}`, see  :doc:`mechanics`.

.. figure:: ../figs/sm_block_stat.svg
    :figclass: only-light
    :width: 100%
    :align: center
    :alt: Synchronous machine model seen from stator coordinates
    :target: .

    Synchronous machine model seen from stator coordinates.

.. figure:: ../figs/sm_block_stat.svg
    :figclass: invert-colors-dark only-dark
    :width: 100%
    :align: center
    :alt: Synchronous machine model seen from stator coordinates
    :target: .

    Synchronous machine model seen from stator coordinates.

The model is implemented in the class :class:`motulator.drive.model.SynchronousMachine`. The parameters are defined in the data class :class:`motulator.drive.model.SynchronousMachinePars` for the linear magnetics, see :eq:`sm_flux`.

Magnetic Saturation
^^^^^^^^^^^^^^^^^^^

The linear magnetic model :eq:`sm_flux` can be replaced with :math:`\boldsymbol{\psi}_\mathrm{s} = \psi_\mathrm{d} + \mathrm{j}\psi_\mathrm{q}`, where the flux linkage components

.. math::
    \psi_\mathrm{d} &= \psi_\mathrm{d}(i_\mathrm{d}, i_\mathrm{q}) \\
    \psi_\mathrm{q} &= \psi_\mathrm{q}(i_\mathrm{d}, i_\mathrm{q})
    :label: sm_flux_sat

are nonlinear in the components of the current vector :math:`\boldsymbol{i}_\mathrm{s} = i_\mathrm{d} + \mathrm{j} i_\mathrm{q}`. Other equations of the model remain the same. The flux linkage maps are numerically invertible. These maps are often modeled either in a form of lookup tables or explicit functions [#Hin2017]_, [#Lel2024]_. The magnetic saturation can be parametrized using the data class :class:`motulator.drive.model.SaturatedSynchronousMachinePars`. Furthermore, methods for manipulating and plotting the flux map data are provided, see the class :class:`motulator.drive.utils.MagneticModel`, :func:`motulator.drive.utils.plot_maps`, and :func:`motulator.drive.utils.plot_flux_vs_current`. See also the examples :doc:`/drive_examples/flux_vector/plot_flux_vector_pmsyrm_5kw_sat`, :doc:`/drive_examples/flux_vector/plot_flux_vector_syrm_7kw_sat`, and :doc:`/drive_examples/current_vector/plot_vector_ctrl_pmsyrm_thor_sat`.

MTPA and MTPV Conditions
^^^^^^^^^^^^^^^^^^^^^^^^

The maximum-torque-per-ampere (MTPA) condition of a saturable machine can be compactly presented by means of the auxiliary flux vector [#Var2021]_. Applying the current vector :math:`\boldsymbol{i}_\mathrm{s} = i \mathrm{e}^{\mathrm{j}\gamma}` with a given magnitude :math:`i` in the torque expression :eq:`sm_torque`, the MTPA condition is obtained by setting :math:`\partial \tau_\mathrm{M}/\partial \gamma = 0`, resulting in

.. math::
    \text{MTPA:} \quad \mathrm{Re} \left\{\boldsymbol{i}_\mathrm{s} \boldsymbol{\psi}_\mathrm{a}^* \right\} = 0
    :label: sm_mtpa

where the star operator denotes the complex conjugate. The auxiliary flux vector :math:`\boldsymbol{\psi}_\mathrm{a}` is defined as

.. math::
    \boldsymbol{\psi}_\mathrm{a} = \boldsymbol{\psi}_\mathrm{s} - L_\mathrm{qq} i_\mathrm{d} - \mathrm{j} L_\mathrm{dd} i_\mathrm{q} + \mathrm{j} L_\mathrm{dq} \boldsymbol{i}_\mathrm{s}^*
    :label: sm_mtpa_aux

where :math:`L_\mathrm{dd} = \partial \psi_\mathrm{d}/\partial i_\mathrm{d}`, :math:`L_\mathrm{qq} = \partial \psi_\mathrm{q}/\partial i_\mathrm{q}`, and :math:`L_\mathrm{dq} = \partial \psi_\mathrm{d}/\partial i_\mathrm{q}` are the incremental inductances obtained from :eq:`sm_flux_sat`. Notice that the auxiliary flux is a function of the stator current, :math:`\boldsymbol{\psi}_\mathrm{a} =  \boldsymbol{\psi}_\mathrm{a}(\boldsymbol{i}_\mathrm{s})`.

The maximum-torque-per-volt (MTPV) condition can be derived similarly, resulting in

.. math::
    \text{MTPV:} \quad \mathrm{Re} \left\{\boldsymbol{\psi}_\mathrm{s} \boldsymbol{i}_\mathrm{a}^* \right\} = 0
    :label: sm_mtpv

The auxiliary current vector :math:`\boldsymbol{\psi}_\mathrm{a}` is defined as

.. math::
    \boldsymbol{i}_\mathrm{a} = -\boldsymbol{i}_\mathrm{s} + \Gamma_\mathrm{qq} \psi_\mathrm{d} + \mathrm{j} \Gamma_\mathrm{dd} \psi_\mathrm{q} - \mathrm{j} \Gamma_\mathrm{dd} \boldsymbol{\psi}_\mathrm{s}^*
    :label: sm_mtpv_aux

where :math:`\Gamma_\mathrm{dd} = \partial i_\mathrm{d}/\partial \psi_\mathrm{d}`, :math:`\Gamma_\mathrm{qq} = \partial i_\mathrm{q}/\partial \psi_\mathrm{q}`, and :math:`\Gamma_\mathrm{dq} = \partial i_\mathrm{d}/\partial \psi_\mathrm{q}` are the incremental inverse inductances. The auxiliary current is a function of the flux linkage, :math:`\boldsymbol{i}_\mathrm{a} = \boldsymbol{i}_\mathrm{a}(\boldsymbol{\psi}_\mathrm{s})`. The data class :class:`motulator.drive.model.SaturatedSynchronousMachinePars` contains methods for computing auxiliary flux and current vectors based on the given magnetic model. In addition to optimal reference generation, these vectors are useful in sensorless observers and flux-vector control [#Tii2025a]_.

The MTPA and MTPV conditions in :eq:`sm_mtpa` and :eq:`sm_mtpv` are realized in the class :class:`motulator.drive.utils.ControlLoci` that provides methods for computing the MTPA, MTPV, and constant current loci for magnetically linear as well as saturated machines. The class :class:`motulator.drive.utils.MachineCharacteristics` provides methods for visualizing these loci. See also the examples :doc:`/drive_examples/flux_vector/plot_flux_vector_pmsyrm_5kw_sat`, :doc:`/drive_examples/flux_vector/plot_flux_vector_syrm_7kw_sat`, and :doc:`/drive_examples/current_vector/plot_vector_ctrl_pmsyrm_thor_sat`.

.. note::
    Here, we define the auxiliary vectors according to the conventions used in [#Hin2018]_, which differ from the conventions in [#Var2021]_, i.e., the vectors defined here are 90 degrees rotated as compared to [#Var2021]_.

.. rubric:: References

.. [#Sle1989] Slemon, "Modelling of induction machines for electric drives," IEEE Trans. Ind. Appl., 1989, https://doi.org/10.1109/28.44251.

.. [#Qu2012] Qu, Ranta, Hinkkanen, Luomi, "Loss-minimizing flux level control of induction motor drives," IEEE Trans. Ind. Appl., 2012, https://doi.org/10.1109/TIA.2012.2190818

.. [#Jah1986] Jahns, Kliman, Neumann, “Interior permanent-magnet synchronous motors for adjustable-speed drives,” IEEE Trans. Ind. Appl., 1986, https://doi.org/10.1109/TIA.1986.4504786

.. [#Hin2017] Hinkkanen, Pescetto, Mölsä, Saarakkala, Pellegrino, Bojoi, “Sensorless self-commissioning of synchronous reluctance motors at standstill without rotor locking,” IEEE Trans. Ind. Appl., 2017, https://doi.org/10.1109/TIA.2016.2644624

.. [#Lel2024] Lelli, Hinkkanen, Giulii Capponi, "A saturation model based on a simplified equivalent magnetic circuit for permanent magnet machines," Proc. ICEM, 2024, https://doi.org/10.1109/ICEM60801.2024.10700403

.. [#Var2021] Varatharajan, Pellegrino, Armando, Hinkkanen, “Sensorless control of synchronous motor drives: Accurate torque estimation and control under parameter errors,” IEEE J. Emerg. Sel. Topics Power Electron., 2021, https://doi.org/10.1109/JESTPE.2020.3037792

.. [#Tii2025a] Tiitinen, Hinkkanen, Harnefors, "Design framework for sensorless control of synchronous machine drives," IEEE Trans. Ind. Electron., 2025, https://doi.org/10.1109/TIE.2024.3429650

.. [#Hin2018] Hinkkanen, Saarakkala, Awan, Mölsä, Tuovinen, "Observers for sensorless synchronous motor drives: Framework for design and analysis," IEEE Trans. Ind. Appl., 2018, https://doi.org/10.1109/TIA.2018.2858753
