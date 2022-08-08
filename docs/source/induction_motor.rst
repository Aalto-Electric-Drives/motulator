Induction Motor
===============

Continuous-time induction motor models are given in the module :mod:`motulator.model.im`. Peak-valued complex space vectors are used. The models are implemented in stator coordinates. A Γ-equivalent model is used since it can be extended with the magnetic saturation model in a staightforward manner [1]_.

.. figure:: figs/im_gamma.svg
   :width: 100%
   :align: center
   :alt: Gamma-model of an induction motor
   :target: .

   Γ model.

.. figure:: figs/im_block.svg
   :width: 100%
   :align: center
   :alt: Block diagram of an induction motor model
   :target: .

   Block diagram of the motor model. The magnetic model includes the flux equations (or, optionally, saturation characteristics) and the torque equation.

The voltage equations are

.. math::
    \frac{\mathrm{d}\boldsymbol{\psi}_\mathrm{s}^\mathrm{s}}{\mathrm{d} t} &= \boldsymbol{u}_\mathrm{s}^\mathrm{s} - R_\mathrm{s}\boldsymbol{i}_\mathrm{s}^\mathrm{s} \\
    \frac{\mathrm{d}\boldsymbol{\psi}_\mathrm{r}^\mathrm{s}}{\mathrm{d} t} &= -R_\mathrm{r}\boldsymbol{i}_\mathrm{r}^\mathrm{s} + \mathrm{j}\omega_\mathrm{m}\boldsymbol{\psi}_\mathrm{r}^\mathrm{s}

where :math:`\boldsymbol{u}_\mathrm{s}^\mathrm{s}` is the stator voltage, :math:`\boldsymbol{i}_\mathrm{s}^\mathrm{s}` is the stator current, :math:`\boldsymbol{i}_\mathrm{r}^\mathrm{s}` is the rotor current, :math:`R_\mathrm{s}` is the stator resistance, and :math:`R_\mathrm{r}` is the rotor resistance. The electrical angular speed of the rotor is :math:`\omega_\mathrm{m} = p\omega_\mathrm{M}`, where :math:`\omega_\mathrm{M}` is the mechanical angular speed of the rotor and :math:`p` is the number of pole pairs. The stator flux linkage :math:`\boldsymbol{\psi}_\mathrm{s}^\mathrm{s}` and the rotor flux linkage :math:`\boldsymbol{\psi}_\mathrm{r}^\mathrm{s}`, respectively, are 

.. math::
    \boldsymbol{\psi}_\mathrm{s}^\mathrm{s} &= L_\mathrm{s}(\boldsymbol{i}_\mathrm{s}^\mathrm{s} + \boldsymbol{i}_\mathrm{r}^\mathrm{s} ) \\
    \boldsymbol{\psi}_\mathrm{r}^\mathrm{s} &= \boldsymbol{\psi}_\mathrm{s}^\mathrm{s} + L_\ell\boldsymbol{i}_\mathrm{r}^\mathrm{s} 

where :math:`L_\mathrm{s}` is the stator inductance and :math:`L_\ell` is the leakage inductance. This linear magnetic model is applied in the class :class:`motulator.model.im.InductionMotor`. The electromagnetic torque is

.. math::
    \tau_\mathrm{M} = \frac{3p}{2}\mathrm{Im} \left\{\boldsymbol{i}_\mathrm{s}^\mathrm{s} (\boldsymbol{\psi}_\mathrm{s}^\mathrm{s})^* \right\}

The class :class:`motulator.model.im.InductionMotorSaturated` extends the model with the main flux saturation, :math:`L_\mathrm{s} = L_\mathrm{s}(\psi_\mathrm{s})` [2]_.

.. note::
   If the magnetic saturation is omitted, the Γ model is mathematically identical to the inverse-Γ and  
   T models. For example, the parameters of the Γ model can be transformed to those of the inverse-Γ
   model parameters as follows:

   .. math::
       L_\sigma &= \left(\frac{L_\mathrm{s}}{L_\mathrm{s} + L_\ell}\right)L_\ell \\
       R_\mathrm{R} &= \left(\frac{L_\mathrm{s}}{L_\mathrm{s} + L_\ell}\right)^2 R_\mathrm{r} \\
       L_\mathrm{M} &=  L_\mathrm{s} - L_\sigma 

   .. figure:: figs/im_inv_gamma.svg
      :width: 100%
      :align: center
      :alt: Inverse-Gamma model of an induction motor
      :target: .

      Inverse-Γ model.

   Example control methods in the module :mod:`motulator.control.im_vector` are based on the
   inverse-Γ model.

References
----------

.. [1] Slemon, "Modelling of induction machines for electric drives," IEEE Trans. Ind. Appl., 1989, https://doi.org/10.1109/28.44251.

.. [2] Qu, Ranta, Hinkkanen, Luomi, "Loss-minimizing flux level control of
   induction motor drives," IEEE Trans. Ind. Appl., 2021,
   https://doi.org/10.1109/TIA.2012.2190818