

.. _sphx_glr_drive_examples_flux_vector:

Flux-Vector Control
-------------------

These examples demonstrate flux-vector control of electric machine drives [#Pel2009]_. In the implemented control system, decoupling between the stator flux and torque channels are used according to [#Awa2019]_. Furthermore, the stator flux magnitude and the electromagnetic torque are selected as controllable variables. The implementation of sensorless mode corresponds to [#Tii2024]_.

.. rubric:: References

.. [#Pel2009] Pellegrino, Armando, Guglielmi, “Direct flux field-oriented control of IPM drives with variable DC link in the field-weakening region,” IEEE Trans. Ind. Appl., 2009, https://doi.org/10.1109/TIA.2009.2027167

.. [#Awa2019] Awan, Hinkkanen, Bojoi, Pellegrino, "Stator-flux-oriented control of synchronous motors: A systematic design procedure," IEEE Trans. Ind. Appl., 2019, https://doi.org/10.1109/TIA.2019.2927316

.. [#Tii2024] Tiitinen, Hinkkanen, Harnefors, "Design framework for sensorless control of synchronous machine drives," IEEE Trans. Ind. Electron., 2024, https://doi.org/10.1109/TIE.2024.3429650 



.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless flux-vector control of a 2.2-kW PMSM drive.">

.. only:: html

  .. image:: /drive_examples/flux_vector/images/thumb/sphx_glr_plot_flux_vector_pmsm_2kw_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_flux_vector_plot_flux_vector_pmsm_2kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW PMSM</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless flux-vector control of a 2.2-kW induction machine drive.">

.. only:: html

  .. image:: /drive_examples/flux_vector/images/thumb/sphx_glr_plot_flux_vector_im_2kw_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_flux_vector_plot_flux_vector_im_2kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW induction motor</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless stator-flux-vector control of a saturated 6.7-kW synchronous reluctance motor drive. The saturation is not taken into account in the control method (only in the system model). Even if the machine has no magnets, the PM-flux disturbance estimation is enabled [#Tuo2018]_. In this case, this PM-flux estimate lumps the effects of inductance errors. Naturally, the PM-flux estimation can be used in PM machine drives as well.">

.. only:: html

  .. image:: /drive_examples/flux_vector/images/thumb/sphx_glr_plot_flux_vector_syrm_7kw_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_flux_vector_plot_flux_vector_syrm_7kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">6.7-kW SyRM, saturated, disturbance estimation</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless stator-flux-vector control of a 5.5-kW PM-SyRM (Baldor ECS101M0H7EF4) drive. The machine model is parametrized using the algebraic saturation model from [#Lel2024]_, fitted to the flux linkage maps measured using the constant-speed test. For comparison, the measured data is plotted together with the model predictions. Notice that the control system used in this example does not consider the saturation, only the system model does.">

.. only:: html

  .. image:: /drive_examples/flux_vector/images/thumb/sphx_glr_plot_flux_vector_pmsyrm_5kw_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_flux_vector_plot_flux_vector_pmsyrm_5kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">5.5-kW PM-SyRM, saturated</div>
    </div>


.. thumbnail-parent-div-close

.. raw:: html

    </div>


.. toctree::
   :hidden:

   /drive_examples/flux_vector/plot_flux_vector_pmsm_2kw
   /drive_examples/flux_vector/plot_flux_vector_im_2kw
   /drive_examples/flux_vector/plot_flux_vector_syrm_7kw
   /drive_examples/flux_vector/plot_flux_vector_pmsyrm_5kw

