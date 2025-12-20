:orphan:

Drives
======

A collection of example scripts for machine drives.



.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. thumbnail-parent-div-close

.. raw:: html

    </div>

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

  :doc:`/drive_examples/flux_vector/plot_2kw_ipmsm_fvc`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IPMSM, FVC</div>
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

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless flux-vector control (FVC) of a saturated 6.7-kW synchronous reluctance machine (SyRM) drive.">

.. only:: html

  .. image:: /drive_examples/flux_vector/images/thumb/sphx_glr_plot_7kw_syrm_sat_fvc_thumb.png
    :alt:

  :doc:`/drive_examples/flux_vector/plot_7kw_syrm_sat_fvc`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">6.7-kW saturated SyRM, FVC</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless flux-vector control (FVC) of a saturated 5.1-kW permanent-magnet synchronous reluctance machine (PM-SyRM). The flux maps of this example machine, known as THOR, are from the SyR-e project:">

.. only:: html

  .. image:: /drive_examples/flux_vector/images/thumb/sphx_glr_plot_5kw_pmsyrm_thor_sat_fvc_thumb.png
    :alt:

  :doc:`/drive_examples/flux_vector/plot_5kw_pmsyrm_thor_sat_fvc`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">5.1-kW saturated PM-SyRM, FVC</div>
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


.. thumbnail-parent-div-close

.. raw:: html

    </div>

Current-Vector Control
----------------------

These examples are for current-vector control (CVC) of induction and synchronous machines. The magnetic saturation model of an induction machine is also demonstrated (:doc:`/drive_examples/current_vector/plot_2kw_im_sat_cvc`) as well as computation of control lookup tables for synchronous machines (:doc:`/drive_examples/current_vector/plot_5kw_pmsyrm_thor_sat_cvc`).



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

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless current-vector control (CVC) of a 2.2-kW interior permanent-magnet synchronous machine (IPMSM) drive. The PM-flux adaptation is enabled [#Tuo2018]_. To demonstrate adaptation, the initial value of the PM-flux estimate has an error of 25%.">

.. only:: html

  .. image:: /drive_examples/current_vector/images/thumb/sphx_glr_plot_2kw_ipmsm_cvc_adapt_thumb.png
    :alt:

  :doc:`/drive_examples/current_vector/plot_2kw_ipmsm_cvc_adapt`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IPMSM, CVC, PM-flux adaptation</div>
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

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless current-vector control (CVC) of a saturated 5.1-kW permanent-magnet synchronous reluctance machine (PM-SyRM). The flux maps of this example machine, known as THOR, are from the SyR-e project:">

.. only:: html

  .. image:: /drive_examples/current_vector/images/thumb/sphx_glr_plot_5kw_pmsyrm_thor_sat_cvc_thumb.png
    :alt:

  :doc:`/drive_examples/current_vector/plot_5kw_pmsyrm_thor_sat_cvc`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">5.1-kW saturated PM-SyRM, CVC</div>
    </div>


.. thumbnail-parent-div-close

.. raw:: html

    </div>

V/Hz Control
------------

These examples demonstrate observer-based V/Hz (O-V/Hz) control for synchronous machines [#Tii2025aa]_ and induction machines [#Tii2025bb]_. The example :doc:`/drive_examples/vhz/plot_2kw_ipmsm_2mass_ovhz` demonstrates the use of a two-mass mechanics model. Furthermore, the examples :doc:`/drive_examples/vhz/plot_2kw_im_diode_vhz` and :doc:`/drive_examples/vhz/plot_2kw_im_lc_vhz` show operation of an induction machine under pure open-loop V/Hz control with a diode front-end rectifier and with an LC filter, respectively.

.. rubric:: References

.. [#Tii2025aa] Tiitinen, Hinkkanen, Harnefors, "Design framework for sensorless control of synchronous machine drives," IEEE Trans. Ind. Electron., 2025, https://doi.org/10.1109/TIE.2024.3429650

.. [#Tii2025bb] Tiitinen, Hinkkanen, Harnefors, "Sensorless flux-vector control framework: An extension for induction machines," IEEE Trans. Ind. Electron., 2025, https://doi.org/10.1109/TIE.2025.3559958



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

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates pure open-loop V/Hz control of a 2.2-kW induction machine (IM) drive. A diode bridge, stiff three-phase grid, and a DC link is modeled.">

.. only:: html

  .. image:: /drive_examples/vhz/images/thumb/sphx_glr_plot_2kw_im_diode_vhz_thumb.png
    :alt:

  :doc:`/drive_examples/vhz/plot_2kw_im_diode_vhz`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IM, diode bridge, V/Hz control</div>
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

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless vector control of a saturated 6.7-kW synchronous reluctance machine (SyRM). Square-wave signal injection with a simple phase-locked loop is used. Cross-saturation errors are compensated for using flux maps. Square-wave signal injection with a simple phase-locked loop is used.">

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


.. toctree::
   :hidden:
   :includehidden:


   /drive_examples/flux_vector/index.rst
   /drive_examples/current_vector/index.rst
   /drive_examples/vhz/index.rst
   /drive_examples/signal_inj/index.rst


.. only:: html

  .. container:: sphx-glr-footer sphx-glr-footer-gallery

    .. container:: sphx-glr-download sphx-glr-download-python

      :download:`Download all examples in Python source code: drive_examples_python.zip </drive_examples/drive_examples_python.zip>`

    .. container:: sphx-glr-download sphx-glr-download-jupyter

      :download:`Download all examples in Jupyter notebooks: drive_examples_jupyter.zip </drive_examples/drive_examples_jupyter.zip>`


.. only:: html

 .. rst-class:: sphx-glr-signature

    `Gallery generated by Sphinx-Gallery <https://sphinx-gallery.github.io>`_
