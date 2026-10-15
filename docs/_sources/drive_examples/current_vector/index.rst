

.. _sphx_glr_drive_examples_current_vector:

Current-Vector Control
----------------------

These examples are for current-vector control (CVC) of induction and synchronous machines. The magnetic saturation model of an induction machine is also demonstrated (:doc:`/drive_examples/current_vector/plot_2kw_im_sat_cvc`) as well as computation of control lookup tables for synchronous machines (:doc:`/drive_examples/current_vector/plot_5kw_pmsyrm_thor_sat_cvc`). The example :doc:`/drive_examples/current_vector/plot_2kw_ipmsm_diode_cvc` includes a diode bridge rectifier, while :doc:`/drive_examples/current_vector/plot_2kw_ipmsm_afe_cvc` and :doc:`/drive_examples/current_vector/plot_2kw_ipmsm_afe_dvf_cvc` include an active front end with grid-following and dual-voltage-forming control, respectively, each with two independent control systems.


.. raw:: html

  <div id='sg-tag-list' class='sphx-glr-tag-list'></div>


.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless current-vector control (CVC) of a 6.7-kW synchronous reluctance machine (SyRM) drive.">

.. only:: html

  .. image:: /drive_examples/current_vector/images/thumb/sphx_glr_plot_7kw_syrm_cvc_thumb.png
    :alt:

  :doc:`/drive_examples/current_vector/plot_7kw_syrm_cvc`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">6.7-kW SyRM, CVC</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates current-vector control (CVC) of a 2.2-kW induction motor (IM) drive in torque-control mode.">

.. only:: html

  .. image:: /drive_examples/current_vector/images/thumb/sphx_glr_plot_2kw_im_cvc_tq_thumb.png
    :alt:

  :doc:`/drive_examples/current_vector/plot_2kw_im_cvc_tq`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IM, CVC, torque-control mode</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless current-vector control (CVC) of a 2.2-kW interior permanent-magnet synchronous machine (IPMSM) drive, equipped with a diode bridge rectifier.">

.. only:: html

  .. image:: /drive_examples/current_vector/images/thumb/sphx_glr_plot_2kw_ipmsm_diode_cvc_thumb.png
    :alt:

  :doc:`/drive_examples/current_vector/plot_2kw_ipmsm_diode_cvc`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IPMSM, diode bridge, CVC</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless current-vector control (CVC) of a 2.2-kW induction motor (IM) drive. The magnetic saturation is included in the machine model, while the control system uses constant parameters.">

.. only:: html

  .. image:: /drive_examples/current_vector/images/thumb/sphx_glr_plot_2kw_im_sat_cvc_thumb.png
    :alt:

  :doc:`/drive_examples/current_vector/plot_2kw_im_sat_cvc`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW saturated IM, CVC</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless current-vector control (CVC) of a 2.2-kW interior permanent-magnet synchronous machine (IPMSM) drive. The PM-flux adaptation is enabled [#Tuo2018]_. To demonstrate adaptation, the initial value of the PM-flux estimate has an error of 25%.">

.. only:: html

  .. image:: /drive_examples/current_vector/images/thumb/sphx_glr_plot_2kw_ipmsm_cvc_adapt_thumb.png
    :alt:

  :doc:`/drive_examples/current_vector/plot_2kw_ipmsm_cvc_adapt`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IPMSM, CVC, PM-flux adaptation</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless current-vector control (CVC) of a saturated 5.1-kW permanent-magnet synchronous reluctance machine (PM-SyRM). The flux maps of this example machine, known as THOR, are from the SyR-e project:">

.. only:: html

  .. image:: /drive_examples/current_vector/images/thumb/sphx_glr_plot_5kw_pmsyrm_thor_sat_cvc_thumb.png
    :alt:

  :doc:`/drive_examples/current_vector/plot_5kw_pmsyrm_thor_sat_cvc`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">5.1-kW saturated PM-SyRM, CVC</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless current-vector control (CVC) of a 2.2-kW induction motor (IM) drive at low speeds. The dead time and the threshold voltage of the converter are modeled, and their effects are compensated for in the control system. The magnetic saturation is included in the machine model and taken into account in the control system. The sensorless observer and the compensation method are similar to [#Hin2010]_.">

.. only:: html

  .. image:: /drive_examples/current_vector/images/thumb/sphx_glr_plot_2kw_im_dead_time_cvc_thumb.png
    :alt:

  :doc:`/drive_examples/current_vector/plot_2kw_im_dead_time_cvc`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IM, dead time, CVC</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example compares the continuous-time and direct discrete-time designs of the current controller in sensorless current-vector control of a 2.2-kW induction machine (IM). The sampling frequency is 5 kHz and the current-control bandwidth is 2π·700 rad/s, which is high compared with the sampling frequency.">

.. only:: html

  .. image:: /drive_examples/current_vector/images/thumb/sphx_glr_plot_2kw_im_cvc_discrete_thumb.png
    :alt:

  :doc:`/drive_examples/current_vector/plot_2kw_im_cvc_discrete`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IM, discrete-time current control</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless current-vector control (CVC) of the same 5.6-kW permanent-magnet synchronous reluctance machine (PM-SyRM) as the corresponding flux- vector control example. See /drive_examples/flux_vector/plot_6kw_pmsyrm_sat_fvc for the machine model, flux-map data, and saturation-model description. This example demonstrates online reference generation in current-vector control, which allows the optimal current references to be generated in real time [#Sar2026]_.">

.. only:: html

  .. image:: /drive_examples/current_vector/images/thumb/sphx_glr_plot_6kw_pmsyrm_sat_cvc_thumb.png
    :alt:

  :doc:`/drive_examples/current_vector/plot_6kw_pmsyrm_sat_cvc`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">5.6-kW saturated PM-SyRM, CVC</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example compares the continuous-time and direct discrete-time designs of the current controller [#Awa2019]_ in sensorless current-vector control of a saturated 5.6-kW permanent-magnet synchronous reluctance machine (PM-SyRM). The sampling frequency is 5 kHz and the current-control bandwidth is 2π·500 rad/s, which is high compared with the sampling frequency.">

.. only:: html

  .. image:: /drive_examples/current_vector/images/thumb/sphx_glr_plot_6kw_pmsyrm_sat_cvc_discrete_thumb.png
    :alt:

  :doc:`/drive_examples/current_vector/plot_6kw_pmsyrm_sat_cvc_discrete`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">5.6-kW saturated PM-SyRM, discrete-time current control</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates a 2.2-kW interior permanent-magnet synchronous machine (IPMSM) drive whose active front end (AFE) uses dual-voltage-forming control [#Mou2025]_ and is connected to a weak grid. The grid-side converter regulates both the DC-bus voltage and its output voltage magnitude without a cascaded DC-bus voltage controller: the external power fed to the DC bus by the drive is estimated by a disturbance observer and fed forward. The machine-side converter uses sensorless current-vector control (CVC). The two control systems are independent: they have different sampling periods, and their carriers are not synchronized. The system model is otherwise the same as in /drive_examples/current_vector/plot_2kw_ipmsm_afe_cvc, except for the weak grid.">

.. only:: html

  .. image:: /drive_examples/current_vector/images/thumb/sphx_glr_plot_2kw_ipmsm_afe_dvf_cvc_thumb.png
    :alt:

  :doc:`/drive_examples/current_vector/plot_2kw_ipmsm_afe_dvf_cvc`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IPMSM, dual-voltage-forming AFE, CVC</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates a 2.2-kW interior permanent-magnet synchronous machine (IPMSM) drive equipped with an active front end (AFE), i.e., a grid-connected converter that controls the DC-bus voltage. The machine-side converter uses sensorless current-vector control (CVC), and the grid-side converter uses grid-following current-vector control with a DC-bus voltage controller. The two control systems are independent: they have different sampling periods, and their carriers are not synchronized.">

.. only:: html

  .. image:: /drive_examples/current_vector/images/thumb/sphx_glr_plot_2kw_ipmsm_afe_cvc_thumb.png
    :alt:

  :doc:`/drive_examples/current_vector/plot_2kw_ipmsm_afe_cvc`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IPMSM, active front end, CVC</div>
    </div>


.. thumbnail-parent-div-close

.. raw:: html

    </div>


.. toctree::
   :hidden:

   /drive_examples/current_vector/plot_7kw_syrm_cvc
   /drive_examples/current_vector/plot_2kw_im_cvc_tq
   /drive_examples/current_vector/plot_2kw_ipmsm_diode_cvc
   /drive_examples/current_vector/plot_2kw_im_sat_cvc
   /drive_examples/current_vector/plot_2kw_ipmsm_cvc_adapt
   /drive_examples/current_vector/plot_5kw_pmsyrm_thor_sat_cvc
   /drive_examples/current_vector/plot_2kw_im_dead_time_cvc
   /drive_examples/current_vector/plot_2kw_im_cvc_discrete
   /drive_examples/current_vector/plot_6kw_pmsyrm_sat_cvc
   /drive_examples/current_vector/plot_6kw_pmsyrm_sat_cvc_discrete
   /drive_examples/current_vector/plot_2kw_ipmsm_afe_dvf_cvc
   /drive_examples/current_vector/plot_2kw_ipmsm_afe_cvc

