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

These examples demonstrate flux-vector control of electric machine drives [#Pel2009]_. In the implemented control system, decoupling between the stator flux and torque channels are used according to [#Awa2019]_. Furthermore, the stator flux magnitude and the electromagnetic torque are selected as controllable variables. The implementations correspond to [#Tii2025a]_ for synchronous machines and [#Tii2025b]_ for induction machines. The magnetic saturation is modeled and taken into account in control.

.. rubric:: References

.. [#Pel2009] Pellegrino, Armando, Guglielmi, “Direct flux field-oriented control of IPM drives with variable DC link in the field-weakening region,” IEEE Trans. Ind. Appl., 2009, https://doi.org/10.1109/TIA.2009.2027167

.. [#Awa2019] Awan, Hinkkanen, Bojoi, Pellegrino, "Stator-flux-oriented control of synchronous motors: A systematic design procedure," IEEE Trans. Ind. Appl., 2019, https://doi.org/10.1109/TIA.2019.2927316

.. [#Tii2025a] Tiitinen, Hinkkanen, Harnefors, "Design framework for sensorless control of synchronous machine drives," IEEE Trans. Ind. Electron., 2025, https://doi.org/10.1109/TIE.2024.3429650

.. [#Tii2025b] Tiitinen, Hinkkanen, Harnefors, "Sensorless flux-vector control framework: An extension for induction machines," IEEE Trans. Ind. Electron., 2025, in press



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

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless stator-flux-vector control of a 5.5-kW PM-SyRM (Baldor ECS101M0H7EF4) drive. The machine model is parametrized using the flux map data, measured using the constant-speed test. The control system is parametrized using the algebraic saturation model from [#Lel2024]_, fitted to the measured data. This saturation model can capture the de-saturation phenomenon of thin iron ribs, see [#Arm2009]_ for details. For comparison, the measured data is plotted together with the model predictions.">

.. only:: html

  .. image:: /drive_examples/flux_vector/images/thumb/sphx_glr_plot_flux_vector_pmsyrm_5kw_sat_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_flux_vector_plot_flux_vector_pmsyrm_5kw_sat.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">5.5-kW PM-SyRM, saturated</div>
    </div>


.. thumbnail-parent-div-close

.. raw:: html

    </div>

Current-Vector Control
----------------------

These examples are for current-vector control of induction and synchronous machines. The magnetic saturation model of an induction machine is also demonstrated (:doc:`/drive_examples/current_vector/plot_vector_ctrl_im_2kw`) as well as computation of control lookup tables for synchronous machines (:doc:`/drive_examples/current_vector/plot_vector_ctrl_pmsyrm_thor_sat`).



.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless current-vector control of a 6.7-kW SyRM drive.">

.. only:: html

  .. image:: /drive_examples/current_vector/images/thumb/sphx_glr_plot_vector_ctrl_syrm_7kw_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_current_vector_plot_vector_ctrl_syrm_7kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">6.7-kW SyRM</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless current-vector control of a 2.2-kW PMSM drive, equipped with a diode bridge rectifier.">

.. only:: html

  .. image:: /drive_examples/current_vector/images/thumb/sphx_glr_plot_vector_ctrl_pmsm_2kw_diode_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_current_vector_plot_vector_ctrl_pmsm_2kw_diode.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW PMSM, diode bridge</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates current-vector control of a 2.2-kW induction motor drive in torque-control mode.">

.. only:: html

  .. image:: /drive_examples/current_vector/images/thumb/sphx_glr_plot_vector_ctrl_im_2kw_tq_mode_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_current_vector_plot_vector_ctrl_im_2kw_tq_mode.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW induction motor, torque-control mode</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless current-vector control of a 2.2-kW induction motor drive. The magnetic saturation of the machine is also included in the system model, while the control system assumes constant parameters.">

.. only:: html

  .. image:: /drive_examples/current_vector/images/thumb/sphx_glr_plot_vector_ctrl_im_2kw_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_current_vector_plot_vector_ctrl_im_2kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW induction motor, saturated</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless current-vector control of a 2.2-kW PMSM drive. The PM-flux adaptation is enabled [#Tuo2018]_. To demonstrate adaptation, the initial value of the PM-flux estimate has an error of 25%.">

.. only:: html

  .. image:: /drive_examples/current_vector/images/thumb/sphx_glr_plot_vector_ctrl_pmsm_2kw_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_current_vector_plot_vector_ctrl_pmsm_2kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW PMSM, with PM flux adaptation</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless current-vector control of a saturated 5-kW permanent- magnet synchronous reluctance motor. The flux maps of this example motor, known as THOR, are from the SyR-e project:">

.. only:: html

  .. image:: /drive_examples/current_vector/images/thumb/sphx_glr_plot_vector_ctrl_pmsyrm_thor_sat_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_current_vector_plot_vector_ctrl_pmsyrm_thor_sat.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">5-kW PM-SyRM, flux maps from SyR-e</div>
    </div>


.. thumbnail-parent-div-close

.. raw:: html

    </div>

V/Hz Control
------------

These examples demonstrate observer-based V/Hz control for synchronous machines [#Tii2025aa]_ and induction machines [#Tii2025bb]_. The example :doc:`/drive_examples/vhz/plot_obs_vhz_ctrl_pmsm_2kw_two_mass` demonstrates the use of a two-mass mechanics model. Furthermore, the examples :doc:`/drive_examples/vhz/plot_vhz_ctrl_im_2kw` and :doc:`/drive_examples/vhz/plot_vhz_ctrl_im_2kw_lc` show operation of an induction machine under pure open-loop V/Hz control with a diode front-end rectifier and with an LC filter, respectively.

.. rubric:: References

.. [#Tii2025aa] Tiitinen, Hinkkanen, Harnefors, "Design framework for sensorless control of synchronous machine drives," IEEE Trans. Ind. Electron., 2025, https://doi.org/10.1109/TIE.2024.3429650

.. [#Tii2025bb] Tiitinen, Hinkkanen, Harnefors, "Sensorless flux-vector control framework: An extension for induction machines," IEEE Trans. Ind. Electron., 2025, in press



.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates observer-based V/Hz control of a 2.2-kW induction motor.">

.. only:: html

  .. image:: /drive_examples/vhz/images/thumb/sphx_glr_plot_obs_vhz_ctrl_im_2kw_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_vhz_plot_obs_vhz_ctrl_im_2kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW induction motor</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates observer-based V/Hz control of a 2.2-kW PMSM drive.">

.. only:: html

  .. image:: /drive_examples/vhz/images/thumb/sphx_glr_plot_obs_vhz_ctrl_pmsm_2kw_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_vhz_plot_obs_vhz_ctrl_pmsm_2kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW PMSM</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="A diode bridge, stiff three-phase grid, and a DC link is modeled. The control system is configured as pure open-loop V/Hz control.">

.. only:: html

  .. image:: /drive_examples/vhz/images/thumb/sphx_glr_plot_vhz_ctrl_im_2kw_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_vhz_plot_vhz_ctrl_im_2kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW induction motor, diode bridge</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates observer-based V/Hz control of a saturated 6.7-kW synchronous reluctance motor drive. The control method uses constant inductances.">

.. only:: html

  .. image:: /drive_examples/vhz/images/thumb/sphx_glr_plot_obs_vhz_ctrl_syrm_7kw_sat_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_vhz_plot_obs_vhz_ctrl_syrm_7kw_sat.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">6.7-kW SyRM, saturated</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates open-loop V/Hz control of a 2.2-kW induction machine drive equipped with an output LC filter.">

.. only:: html

  .. image:: /drive_examples/vhz/images/thumb/sphx_glr_plot_vhz_ctrl_im_2kw_lc_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_vhz_plot_vhz_ctrl_im_2kw_lc.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW induction motor, LC filter</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates observer-based V/Hz control of a 2.2-kW PMSM drive. The mechanical subsystem is modeled as a two-mass system. The resonance frequency of the mechanics is around 85 Hz. The mechanical parameters correspond to [#Saa2015]_, except that the torsional damping is set to a smaller value in this example.">

.. only:: html

  .. image:: /drive_examples/vhz/images/thumb/sphx_glr_plot_obs_vhz_ctrl_pmsm_2kw_two_mass_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_vhz_plot_obs_vhz_ctrl_pmsm_2kw_two_mass.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW PMSM, 2-mass mechanics</div>
    </div>


.. thumbnail-parent-div-close

.. raw:: html

    </div>

Signal Injection
----------------

These examples demonstrate a square-wave signal injection for low-speed operation based on [#Kim2012]_. A phase-locked loop is used to track the rotor position. For a wider speed range, signal injection could be combined to a model-based observer. The effects of magnetic saturation are not compensated for in this version.

.. rubric:: References

.. [#Kim2012] Kim, Ha, Sul, "PWM switching frequency signal injection sensorless method in IPMSM," IEEE Trans. Ind. Appl., 2012, https://doi.org/10.1109/TIA.2012.2210175



.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless vector control of a 2.2-kW PMSM drive. Square-wave signal injection is used with a simple phase-locked loop.">

.. only:: html

  .. image:: /drive_examples/signal_inj/images/thumb/sphx_glr_plot_signal_inj_pmsm_2kw_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_signal_inj_plot_signal_inj_pmsm_2kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW PMSM</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless vector control of a 6.7-kW SyRM drive. Square-wave signal injection is used with a simple phase-locked loop.">

.. only:: html

  .. image:: /drive_examples/signal_inj/images/thumb/sphx_glr_plot_signal_inj_syrm_7kw_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_signal_inj_plot_signal_inj_syrm_7kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">6.7-kW SyRM</div>
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
