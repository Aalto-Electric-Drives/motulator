:orphan:

Drives
======

A collection of example scripts for machine drives.


.. raw:: html

  <div id='sg-tag-list' class='sphx-glr-tag-list'></div>


.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. thumbnail-parent-div-close

.. raw:: html

    </div>

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

Current-Vector Control
----------------------

These examples are for current-vector control (CVC) of induction and synchronous machines. The magnetic saturation model of an induction machine is also demonstrated (:doc:`/drive_examples/current_vector/plot_2kw_im_sat_cvc`) as well as computation of control lookup tables for synchronous machines (:doc:`/drive_examples/current_vector/plot_5kw_pmsyrm_thor_sat_cvc`).


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

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless current-vector control (CVC) of a 2.2-kW interior permanent-magnet synchronous machine (IPMSM) drive, whose converter gates are blocked before the start and during coasting. The converter output voltage is not measured. While the gates are blocked, the control system runs in standby: its observer runs on its open-circuit model, i.e., the flux estimate rotates at the estimated speed, and its current controller computes its output for zero torque. The gates are released at 0.05 s, and the drive accelerates under speed control. At 0.6 s, the gates are blocked again: the stator current decays through the diodes within a millisecond, after which the loaded machine coasts at open circuit, since its voltage is below the DC-bus voltage, and decelerates. At 0.8 s, the gates are released, and the control system catches the rotating machine with a flying start: it initializes its observer from the current response to a zero-voltage period (cf. [#Tan2009]_) and its current controller from the resulting back-EMF, after which the drive returns to the speed reference.">

.. only:: html

  .. image:: /drive_examples/current_vector/images/thumb/sphx_glr_plot_2kw_ipmsm_blocked_cvc_thumb.png
    :alt:

  :doc:`/drive_examples/current_vector/plot_2kw_ipmsm_blocked_cvc`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IPMSM, blocked gates, CVC</div>
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


.. thumbnail-parent-div-close

.. raw:: html

    </div>

V/Hz Control
------------

These examples demonstrate observer-based V/Hz (O-V/Hz) control for synchronous machines [#Tii2025aa]_ and induction machines [#Tii2025bb]_. The example :doc:`/drive_examples/vhz/plot_2kw_ipmsm_2mass_ovhz` demonstrates the use of a two-mass mechanics model. Furthermore, the example :doc:`/drive_examples/vhz/plot_2kw_im_lc_vhz` shows operation of an induction machine under open-loop V/Hz control with an LC filter.

.. rubric:: References

.. [#Tii2025aa] Tiitinen, Hinkkanen, Harnefors, "Design framework for sensorless control of synchronous machine drives," IEEE Trans. Ind. Electron., 2025, https://doi.org/10.1109/TIE.2024.3429650

.. [#Tii2025bb] Tiitinen, Hinkkanen, Harnefors, "Sensorless flux-vector control framework: An extension for induction machines," IEEE Trans. Ind. Electron., 2025, https://doi.org/10.1109/TIE.2025.3559958


.. raw:: html

  <div id='sg-tag-list' class='sphx-glr-tag-list'></div>


.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates observer-based V/Hz (O-V/Hz) control of a 2.2-kW induction machine (IM).">

.. only:: html

  .. image:: /drive_examples/vhz/images/thumb/sphx_glr_plot_2kw_im_ovhz_thumb.png
    :alt:

  :doc:`/drive_examples/vhz/plot_2kw_im_ovhz`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IM, O-V/Hz control</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates observer-based V/Hz (O-V/Hz) control of a 2.2-kW interior permanent-magnet synchronous machine (IPMSM) drive.">

.. only:: html

  .. image:: /drive_examples/vhz/images/thumb/sphx_glr_plot_2kw_ipmsm_ovhz_thumb.png
    :alt:

  :doc:`/drive_examples/vhz/plot_2kw_ipmsm_ovhz`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IPMSM, O-V/Hz control</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates observer-based V/Hz (O-V/Hz) control of a 6.7-kW synchronous reluctance machine (SyRM) drive. The magnetic saturation is included in the machine model, while the control system uses constant parameters.">

.. only:: html

  .. image:: /drive_examples/vhz/images/thumb/sphx_glr_plot_7kw_syrm_sat_ovhz_thumb.png
    :alt:

  :doc:`/drive_examples/vhz/plot_7kw_syrm_sat_ovhz`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">6.7-kW saturated SyRM, O-V/Hz control</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates open-loop V/Hz control of a 2.2-kW induction machine (IM) drive equipped with an output LC filter.">

.. only:: html

  .. image:: /drive_examples/vhz/images/thumb/sphx_glr_plot_2kw_im_lc_vhz_thumb.png
    :alt:

  :doc:`/drive_examples/vhz/plot_2kw_im_lc_vhz`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IM, LC filter, V/Hz control</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates observer-based V/Hz (O-V/Hz) control of a 2.2-kW interior permanent-magnet synchronous machine (IPMSM) drive. The mechanical subsystem is modeled as a two-mass system. The resonance frequency of the mechanics is around 85 Hz. The mechanical parameters correspond to [#Saa2015]_, except that the torsional damping is set to a smaller value in this example.">

.. only:: html

  .. image:: /drive_examples/vhz/images/thumb/sphx_glr_plot_2kw_ipmsm_2mass_ovhz_thumb.png
    :alt:

  :doc:`/drive_examples/vhz/plot_2kw_ipmsm_2mass_ovhz`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IPMSM, 2-mass mechanics, O-V/Hz control</div>
    </div>


.. thumbnail-parent-div-close

.. raw:: html

    </div>

Signal Injection
----------------

These examples demonstrate a square-wave signal injection for low-speed operation based on [#Kim2012]_. Cross-saturation errors are compensated for using flux maps [#You2018]_. A phase-locked loop is used to track the rotor position. For a wider speed range, signal injection could be combined to a model-based observer.

.. rubric:: References

.. [#Kim2012] Kim, Ha, Sul, "PWM switching frequency signal injection sensorless method in IPMSM," IEEE Trans. Ind. Appl., 2012, https://doi.org/10.1109/TIA.2012.2210175

.. [#You2018] Yousefi-Talouki, Pescetto, Pellegrino, Boldea, "Combined active flux and high-frequency injection methods for sensorless direct-flux vector control of synchronous reluctance machines," IEEE Trans. Power Electron., 2018, https://doi.org/10.1109/TPEL.2017.2697209


.. raw:: html

  <div id='sg-tag-list' class='sphx-glr-tag-list'></div>


.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless vector control of a 2.2-kW interior permanent-magnet synchronous machine (IPMSM) drive. Square-wave signal injection with a simple phase-locked loop is used.">

.. only:: html

  .. image:: /drive_examples/signal_inj/images/thumb/sphx_glr_plot_2kw_ipmsm_signal_inj_thumb.png
    :alt:

  :doc:`/drive_examples/signal_inj/plot_2kw_ipmsm_signal_inj`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IPMSM, signal injection</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless vector control of a saturated 6.7-kW synchronous reluctance machine (SyRM). Square-wave signal injection with a simple phase-locked loop is used, as in /drive_examples/signal_inj/plot_2kw_ipmsm_signal_inj. Cross-saturation errors are compensated for using flux maps.">

.. only:: html

  .. image:: /drive_examples/signal_inj/images/thumb/sphx_glr_plot_7kw_syrm_signal_inj_thumb.png
    :alt:

  :doc:`/drive_examples/signal_inj/plot_7kw_syrm_signal_inj`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">6.7-kW saturated SyRM, signal injection</div>
    </div>


.. thumbnail-parent-div-close

.. raw:: html

    </div>

Grid Connection
---------------

These examples demonstrate machines and drives connected to the grid. The example :doc:`/drive_examples/grid_connection/plot_2kw_im_dol` simulates the direct-on-line start of an induction machine. In :doc:`/drive_examples/grid_connection/plot_2kw_im_diode_vhz` and :doc:`/drive_examples/grid_connection/plot_2kw_ipmsm_diode_cvc`, the drive is fed from a diode bridge. The examples :doc:`/drive_examples/grid_connection/plot_2kw_ipmsm_afe_cvc` and :doc:`/drive_examples/grid_connection/plot_2kw_ipmsm_afe_dvf_cvc` include an active front end with grid-following and dual-voltage-forming control, respectively, each with two independent control systems.


.. raw:: html

  <div id='sg-tag-list' class='sphx-glr-tag-list'></div>


.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates the direct-on-line start of a 2.2-kW induction machine (IM), whose stator is connected directly to a stiff grid. The machine drives a fan, whose load torque is proportional to the square of the speed.">

.. only:: html

  .. image:: /drive_examples/grid_connection/images/thumb/sphx_glr_plot_2kw_im_dol_thumb.png
    :alt:

  :doc:`/drive_examples/grid_connection/plot_2kw_im_dol`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IM, direct-on-line start</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless current-vector control (CVC) of a 2.2-kW interior permanent-magnet synchronous machine (IPMSM) drive, fed from a six-pulse diode bridge. The diode bridge is supplied from a stiff grid through an AC choke.">

.. only:: html

  .. image:: /drive_examples/grid_connection/images/thumb/sphx_glr_plot_2kw_ipmsm_diode_cvc_thumb.png
    :alt:

  :doc:`/drive_examples/grid_connection/plot_2kw_ipmsm_diode_cvc`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IPMSM, diode bridge, CVC</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates pure open-loop V/Hz control of a 2.2-kW induction machine (IM) drive, fed from a six-pulse diode bridge. The diode bridge is supplied from a stiff grid through an AC choke.">

.. only:: html

  .. image:: /drive_examples/grid_connection/images/thumb/sphx_glr_plot_2kw_im_diode_vhz_thumb.png
    :alt:

  :doc:`/drive_examples/grid_connection/plot_2kw_im_diode_vhz`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IM, diode bridge, V/Hz control</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates a 2.2-kW interior permanent-magnet synchronous machine (IPMSM) drive whose active front end (AFE) uses dual-voltage-forming control [#Mou2025]_ and is connected to a weak grid. The grid-side converter regulates both the DC-bus voltage and its output voltage magnitude without a cascaded DC-bus voltage controller: the external power fed to the DC bus by the drive is estimated by a disturbance observer and fed forward. The machine-side converter uses sensorless current-vector control (CVC). The two control systems are independent: they have different sampling periods, and their carriers are not synchronized. The system model is otherwise the same as in /drive_examples/grid_connection/plot_2kw_ipmsm_afe_cvc, except for the weak grid.">

.. only:: html

  .. image:: /drive_examples/grid_connection/images/thumb/sphx_glr_plot_2kw_ipmsm_afe_dvf_cvc_thumb.png
    :alt:

  :doc:`/drive_examples/grid_connection/plot_2kw_ipmsm_afe_dvf_cvc`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IPMSM, dual-voltage-forming AFE, CVC</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates a 2.2-kW interior permanent-magnet synchronous machine (IPMSM) drive equipped with an active front end (AFE), i.e., a grid-connected converter that controls the DC-bus voltage. The machine-side converter uses sensorless current-vector control (CVC), and the grid-side converter uses grid-following current-vector control with a DC-bus voltage controller. The two control systems are independent: they have different sampling periods, and their carriers are not synchronized.">

.. only:: html

  .. image:: /drive_examples/grid_connection/images/thumb/sphx_glr_plot_2kw_ipmsm_afe_cvc_thumb.png
    :alt:

  :doc:`/drive_examples/grid_connection/plot_2kw_ipmsm_afe_cvc`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IPMSM, active front end, CVC</div>
    </div>


.. thumbnail-parent-div-close

.. raw:: html

    </div>


.. toctree::
   :hidden:
   :includehidden:


   /drive_examples/flux_vector/index.rst
   /drive_examples/current_vector/index.rst
   /drive_examples/vhz/index.rst
   /drive_examples/signal_inj/index.rst
   /drive_examples/grid_connection/index.rst


.. only:: html

  .. container:: sphx-glr-footer sphx-glr-footer-gallery

    .. container:: sphx-glr-download sphx-glr-download-python

      :download:`Download all examples in Python source code: drive_examples_python.zip </drive_examples/drive_examples_python.zip>`

    .. container:: sphx-glr-download sphx-glr-download-jupyter

      :download:`Download all examples in Jupyter notebooks: drive_examples_jupyter.zip </drive_examples/drive_examples_jupyter.zip>`


.. only:: html

 .. rst-class:: sphx-glr-signature

    `Gallery generated by Sphinx-Gallery <https://sphinx-gallery.github.io>`_
