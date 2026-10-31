

.. _sphx_glr_drive_examples_flux_vector:

Flux-Vector Control
-------------------

These examples demonstrate flux-vector control (FVC) of electric machine drives [#Pel2009]_. In the implemented control system, decoupling between the stator flux and torque channels are used according to [#Awa2019]_. Furthermore, the stator flux magnitude and the electromagnetic torque are selected as controllable variables. The implementations correspond to [#Tii2025a]_ for synchronous machines and [#Tii2025b]_ for induction machines. The magnetic saturation is modeled and taken into account in control. The examples with GradNet magnetic models [#Li2026]_, based on gradient networks [#Cha2025]_, which model the current or flux linkage map as the gradient of the magnetic energy or co-energy, use models trained from measured and FEM data with the related package `motulator-gradnet <https://aalto-electric-drives.github.io/motulator-gradnet/>`_.

.. rubric:: References

.. [#Pel2009] Pellegrino, Armando, Guglielmi, "Direct flux field-oriented control of IPM drives with variable DC link in the field-weakening region," IEEE Trans. Ind. Appl., 2009, https://doi.org/10.1109/TIA.2009.2027167

.. [#Awa2019] Awan, Hinkkanen, Bojoi, Pellegrino, "Stator-flux-oriented control of synchronous motors: A systematic design procedure," IEEE Trans. Ind. Appl., 2019, https://doi.org/10.1109/TIA.2019.2927316

.. [#Tii2025a] Tiitinen, Hinkkanen, Harnefors, "Design framework for sensorless control of synchronous machine drives," IEEE Trans. Ind. Electron., 2025, https://doi.org/10.1109/TIE.2024.3429650

.. [#Tii2025b] Tiitinen, Hinkkanen, Harnefors, "Sensorless flux-vector control framework: An extension for induction machines," IEEE Trans. Ind. Electron., 2025, https://doi.org/10.1109/TIE.2025.3559958

.. [#Li2026] Li, Foissner, Martin, Piippo, Hinkkanen, "Gradient networks for universal magnetic modeling of synchronous machines," 2026, https://arxiv.org/abs/2602.14947

.. [#Cha2025] Chaudhari, Pranav, Moura, "Gradient networks," IEEE Trans. Signal Process., 2025, https://doi.org/10.1109/TSP.2024.3496692


.. raw:: html

  <div id='sg-tag-list' class='sphx-glr-tag-list'></div>


.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless flux-vector control (FVC) of a 2.2-kW interior permanent-magnet synchronous machine (IPMSM) drive.">

.. only:: html

  .. image:: /drive_examples/flux_vector/images/thumb/sphx_glr_plot_2kw_ipmsm_fvc_thumb.png
    :alt:

  :doc:`/drive_examples/flux_vector/plot_2kw_ipmsm_fvc`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IPMSM, FVC</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless flux-vector control (FVC) of a 2.2-kW interior permanent-magnet synchronous machine (IPMSM) drive with nonideal current sensors. The phase currents are measured with offsets, gain errors, and noise. Apart from the sensors and the speed reference, the drive is the same as in /drive_examples/flux_vector/plot_2kw_ipmsm_fvc.">

.. only:: html

  .. image:: /drive_examples/flux_vector/images/thumb/sphx_glr_plot_2kw_ipmsm_sensor_errors_fvc_thumb.png
    :alt:

  :doc:`/drive_examples/flux_vector/plot_2kw_ipmsm_sensor_errors_fvc`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IPMSM, current-sensor errors, FVC</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless flux-vector control (FVC) of a 2.2-kW induction machine (IM) drive. The magnetic saturation is included in the machine model and taken into account in the control system. This example also applies the mechanical-model-based speed observer.">

.. only:: html

  .. image:: /drive_examples/flux_vector/images/thumb/sphx_glr_plot_2kw_im_sat_fvc_thumb.png
    :alt:

  :doc:`/drive_examples/flux_vector/plot_2kw_im_sat_fvc`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW saturated IM, FVC</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates flux-vector control (FVC) of a 5.6-kW PM synchronous reluctance machine (Baldor ECS101M0H7EF4) drive. GradNet models trained on FEM data are used for both the machine model and the control system. The machine model uses a GradNet current map with spatial harmonics, while the control system uses a GradNet flux map without spatial harmonics. The models are trained with the related package motulator-gradnet.">

.. only:: html

  .. image:: /drive_examples/flux_vector/images/thumb/sphx_glr_plot_6kw_pmsyrm_gn_fvc_fem_harm_thumb.png
    :alt:

  :doc:`/drive_examples/flux_vector/plot_6kw_pmsyrm_gn_fvc_fem_harm`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">5.6-kW PM-SyRM, GradNet from FEM data, FVC</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates flux-vector control (FVC) of a 5.6-kW PM synchronous reluctance machine (Baldor ECS101M0H7EF4) drive. GradNet models trained on measured data without spatial harmonics are used for both the machine model and the control system. The models are trained with the related package motulator-gradnet.">

.. only:: html

  .. image:: /drive_examples/flux_vector/images/thumb/sphx_glr_plot_6kw_pmsyrm_gn_fvc_meas_thumb.png
    :alt:

  :doc:`/drive_examples/flux_vector/plot_6kw_pmsyrm_gn_fvc_meas`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">5.6-kW PM-SyRM, GradNet from measured data, FVC</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless flux-vector control (FVC) of a saturated 6.7-kW synchronous reluctance machine (SyRM) drive.">

.. only:: html

  .. image:: /drive_examples/flux_vector/images/thumb/sphx_glr_plot_7kw_syrm_sat_fvc_thumb.png
    :alt:

  :doc:`/drive_examples/flux_vector/plot_7kw_syrm_sat_fvc`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">6.7-kW saturated SyRM, FVC</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless flux-vector control (FVC) of a 5.6-kW permanent-magnet synchronous reluctance machine (PM-SyRM, Baldor ECS101M0H7EF4) drive. The machine model is parametrized using the flux map data, measured using the constant-speed test.">

.. only:: html

  .. image:: /drive_examples/flux_vector/images/thumb/sphx_glr_plot_6kw_pmsyrm_sat_fvc_thumb.png
    :alt:

  :doc:`/drive_examples/flux_vector/plot_6kw_pmsyrm_sat_fvc`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">5.6-kW saturated PM-SyRM, FVC</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example compares space-vector PWM (SVPWM) and discontinuous PWM (DPWM) [#Hav1998]_ in flux-vector control (FVC) of a 2.2-kW interior permanent-magnet synchronous machine (IPMSM) drive. The converter is modeled by carrier comparison, so the switching ripple of the currents is included. The drive operates at the nominal torque and a constant speed.">

.. only:: html

  .. image:: /drive_examples/flux_vector/images/thumb/sphx_glr_plot_2kw_ipmsm_dpwm_fvc_thumb.png
    :alt:

  :doc:`/drive_examples/flux_vector/plot_2kw_ipmsm_dpwm_fvc`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IPMSM, DPWM, FVC</div>
    </div>


.. thumbnail-parent-div-close

.. raw:: html

    </div>


.. toctree::
   :hidden:

   /drive_examples/flux_vector/plot_2kw_ipmsm_fvc
   /drive_examples/flux_vector/plot_2kw_ipmsm_sensor_errors_fvc
   /drive_examples/flux_vector/plot_2kw_im_sat_fvc
   /drive_examples/flux_vector/plot_6kw_pmsyrm_gn_fvc_fem_harm
   /drive_examples/flux_vector/plot_6kw_pmsyrm_gn_fvc_meas
   /drive_examples/flux_vector/plot_7kw_syrm_sat_fvc
   /drive_examples/flux_vector/plot_6kw_pmsyrm_sat_fvc
   /drive_examples/flux_vector/plot_2kw_ipmsm_dpwm_fvc

