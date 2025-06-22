

.. _sphx_glr_drive_examples_flux_vector:

Flux-Vector Control
-------------------

These examples demonstrate flux-vector control of electric machine drives [#Pel2009]_. In the implemented control system, decoupling between the stator flux and torque channels are used according to [#Awa2019]_. Furthermore, the stator flux magnitude and the electromagnetic torque are selected as controllable variables. The implementations correspond to [#Tii2025a]_ for synchronous machines and [#Tii2025b]_ for induction machines. The magnetic saturation is modeled and taken into account in control.

.. rubric:: References

.. [#Pel2009] Pellegrino, Armando, Guglielmi, “Direct flux field-oriented control of IPM drives with variable DC link in the field-weakening region,” IEEE Trans. Ind. Appl., 2009, https://doi.org/10.1109/TIA.2009.2027167

.. [#Awa2019] Awan, Hinkkanen, Bojoi, Pellegrino, "Stator-flux-oriented control of synchronous motors: A systematic design procedure," IEEE Trans. Ind. Appl., 2019, https://doi.org/10.1109/TIA.2019.2927316

.. [#Tii2025a] Tiitinen, Hinkkanen, Harnefors, "Design framework for sensorless control of synchronous machine drives," IEEE Trans. Ind. Electron., 2025, https://doi.org/10.1109/TIE.2024.3429650

.. [#Tii2025b] Tiitinen, Hinkkanen, Harnefors, "Sensorless flux-vector control framework: An extension for induction machines," IEEE Trans. Ind. Electron., 2025, https://doi.org/10.1109/TIE.2025.3559958



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

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless flux-vector control of a 2.2-kW induction machine.">

.. only:: html

  .. image:: /drive_examples/flux_vector/images/thumb/sphx_glr_plot_flux_vector_im_2kw_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_flux_vector_plot_flux_vector_im_2kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW induction motor</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless stator-flux-vector control of a saturated 6.7-kW synchronous reluctance motor drive.">

.. only:: html

  .. image:: /drive_examples/flux_vector/images/thumb/sphx_glr_plot_flux_vector_syrm_7kw_sat_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_flux_vector_plot_flux_vector_syrm_7kw_sat.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">6.7-kW SyRM, saturated</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless flux-vector control of a saturated 5-kW permanent- magnet synchronous reluctance motor. The flux maps of this example motor, known as THOR, are from the SyR-e project:">

.. only:: html

  .. image:: /drive_examples/flux_vector/images/thumb/sphx_glr_plot_flux_vector_ctrl_pmsyrm_thor_sat_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_flux_vector_plot_flux_vector_ctrl_pmsyrm_thor_sat.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">5-kW PM-SyRM, flux maps from SyR-e</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless stator-flux-vector control of a 5.6-kW PM-SyRM (Baldor ECS101M0H7EF4) drive. The machine model is parametrized using the flux map data, measured using the constant-speed test. The control system is parametrized using the algebraic saturation model from [#Lel2024]_, fitted to the measured data. This saturation model can capture the de-saturation phenomenon of thin iron ribs, see [#Arm2009]_ for details. For comparison, the measured data is plotted together with the model predictions.">

.. only:: html

  .. image:: /drive_examples/flux_vector/images/thumb/sphx_glr_plot_flux_vector_pmsyrm_5kw_sat_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_flux_vector_plot_flux_vector_pmsyrm_5kw_sat.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">5.6-kW PM-SyRM, saturated</div>
    </div>


.. thumbnail-parent-div-close

.. raw:: html

    </div>


.. toctree::
   :hidden:

   /drive_examples/flux_vector/plot_flux_vector_pmsm_2kw
   /drive_examples/flux_vector/plot_flux_vector_im_2kw
   /drive_examples/flux_vector/plot_flux_vector_syrm_7kw_sat
   /drive_examples/flux_vector/plot_flux_vector_ctrl_pmsyrm_thor_sat
   /drive_examples/flux_vector/plot_flux_vector_pmsyrm_5kw_sat

