

.. _sphx_glr_drive_examples_flux_vector:

Flux-Vector Control
-------------------

These examples demonstrate flux-vector control (FVC) of electric machine drives [#Pel2009]_. In the implemented control system, decoupling between the stator flux and torque channels are used according to [#Awa2019]_. Furthermore, the stator flux magnitude and the electromagnetic torque are selected as controllable variables. The implementations correspond to [#Tii2025a]_ for synchronous machines and [#Tii2025b]_ for induction machines. The magnetic saturation is modeled and taken into account in control.

.. rubric:: References

.. [#Pel2009] Pellegrino, Armando, Guglielmi, “Direct flux field-oriented control of IPM drives with variable DC link in the field-weakening region,” IEEE Trans. Ind. Appl., 2009, https://doi.org/10.1109/TIA.2009.2027167

.. [#Awa2019] Awan, Hinkkanen, Bojoi, Pellegrino, "Stator-flux-oriented control of synchronous motors: A systematic design procedure," IEEE Trans. Ind. Appl., 2019, https://doi.org/10.1109/TIA.2019.2927316

.. [#Tii2025a] Tiitinen, Hinkkanen, Harnefors, "Design framework for sensorless control of synchronous machine drives," IEEE Trans. Ind. Electron., 2025, https://doi.org/10.1109/TIE.2024.3429650

.. [#Tii2025b] Tiitinen, Hinkkanen, Harnefors, "Sensorless flux-vector control framework: An extension for induction machines," IEEE Trans. Ind. Electron., 2025, https://doi.org/10.1109/TIE.2025.3559958



.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless flux-vector control (FVC) of a 2.2-kW interior permanent-magnet synchronous machine (IPMSM) drive.">

.. only:: html

  .. image:: /drive_examples/flux_vector/images/thumb/sphx_glr_plot_2kw_ipmsm_fvc_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_flux_vector_plot_2kw_ipmsm_fvc.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IPMSM, FVC</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless flux-vector control (FVC) of a 2.2-kW induction machine (IM) drive. The magnetic saturation is included in the machine model and taken into account in the control system. This example also applies the mechanical-model-based speed observer.">

.. only:: html

  .. image:: /drive_examples/flux_vector/images/thumb/sphx_glr_plot_2kw_im_sat_fvc_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_flux_vector_plot_2kw_im_sat_fvc.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW saturated IM, FVC</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless flux-vector control (FVC) of a saturated 6.7-kW synchronous reluctance machine (SyRM) drive.">

.. only:: html

  .. image:: /drive_examples/flux_vector/images/thumb/sphx_glr_plot_7kw_syrm_sat_fvc_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_flux_vector_plot_7kw_syrm_sat_fvc.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">6.7-kW saturated SyRM, FVC</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless flux-vector control (FVC) of a saturated 5.1-kW permanent-magnet synchronous reluctance machine (PM-SyRM). The flux maps of this example machine, known as THOR, are from the SyR-e project:">

.. only:: html

  .. image:: /drive_examples/flux_vector/images/thumb/sphx_glr_plot_5kw_pmsyrm_thor_sat_fvc_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_flux_vector_plot_5kw_pmsyrm_thor_sat_fvc.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">5.1-kW saturated PM-SyRM, FVC</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless flux-vector control (FVC) of a 5.6-kW permanent-magnet synchronous reluctance machine (PM-SyRM, Baldor ECS101M0H7EF4) drive. The machine model is parametrized using the flux map data, measured using the constant-speed test.">

.. only:: html

  .. image:: /drive_examples/flux_vector/images/thumb/sphx_glr_plot_6kw_pmsyrm_sat_fvc_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_flux_vector_plot_6kw_pmsyrm_sat_fvc.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">5.6-kW saturated PM-SyRM, FVC</div>
    </div>


.. thumbnail-parent-div-close

.. raw:: html

    </div>


.. toctree::
   :hidden:

   /drive_examples/flux_vector/plot_2kw_ipmsm_fvc
   /drive_examples/flux_vector/plot_2kw_im_sat_fvc
   /drive_examples/flux_vector/plot_7kw_syrm_sat_fvc
   /drive_examples/flux_vector/plot_5kw_pmsyrm_thor_sat_fvc
   /drive_examples/flux_vector/plot_6kw_pmsyrm_sat_fvc

