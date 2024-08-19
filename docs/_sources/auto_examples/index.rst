:orphan:

Examples
========

A collection of Python scripts that demonstrate how to use *motulator*.



.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. thumbnail-parent-div-close

.. raw:: html

    </div>

Current-Vector Control
----------------------

These examples are for current-vector control of induction machines and synchronous machines. The magnetic saturation model of an induction machine is also demonstrated (:doc:`/auto_examples/vector/plot_vector_ctrl_im_2kw`) as well as computation of control look-up tables for synchronous machines (:doc:`/auto_examples/vector/plot_vector_ctrl_pmsyrm_thor`). 


.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless current-vector control of a 2.2-kW PMSM  drive. ">

.. only:: html

  .. image:: /auto_examples/vector/images/thumb/sphx_glr_plot_vector_ctrl_pmsm_2kw_thumb.png
    :alt:

  :ref:`sphx_glr_auto_examples_vector_plot_vector_ctrl_pmsm_2kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW PMSM</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless current-vector control of a 6.7-kW SyRM  drive.">

.. only:: html

  .. image:: /auto_examples/vector/images/thumb/sphx_glr_plot_vector_ctrl_syrm_7kw_thumb.png
    :alt:

  :ref:`sphx_glr_auto_examples_vector_plot_vector_ctrl_syrm_7kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">6.7-kW SyRM</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless current-vector control of a 2.2-kW PMSM  drive, equipped with a diode bridge rectifier. ">

.. only:: html

  .. image:: /auto_examples/vector/images/thumb/sphx_glr_plot_vector_ctrl_pmsm_2kw_diode_thumb.png
    :alt:

  :ref:`sphx_glr_auto_examples_vector_plot_vector_ctrl_pmsm_2kw_diode.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW PMSM, diode bridge</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates current-vector control of a 2.2-kW induction motor drive in torque-control mode. ">

.. only:: html

  .. image:: /auto_examples/vector/images/thumb/sphx_glr_plot_vector_ctrl_im_2kw_tq_mode_thumb.png
    :alt:

  :ref:`sphx_glr_auto_examples_vector_plot_vector_ctrl_im_2kw_tq_mode.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW induction motor, torque-control mode</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless current-vector control of a 5-kW permanent- magnet synchronous reluctance motor. Control look-up tables are also plotted.">

.. only:: html

  .. image:: /auto_examples/vector/images/thumb/sphx_glr_plot_vector_ctrl_pmsyrm_thor_thumb.png
    :alt:

  :ref:`sphx_glr_auto_examples_vector_plot_vector_ctrl_pmsyrm_thor.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">5-kW PM-SyRM</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless current-vector control of a 2.2-kW induction  motor drive. The magnetic saturation of the machine is also included in the  system model, while the control system assumes constant parameters. ">

.. only:: html

  .. image:: /auto_examples/vector/images/thumb/sphx_glr_plot_vector_ctrl_im_2kw_thumb.png
    :alt:

  :ref:`sphx_glr_auto_examples_vector_plot_vector_ctrl_im_2kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW induction motor, saturated</div>
    </div>


.. thumbnail-parent-div-close

.. raw:: html

    </div>

V/Hz Control
------------

These examples shows operation of an induction machine under open-loop V/Hz control. Furthermore, a diode front-end rectifier and transition to six-step modulation are also demonstrated. 


.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates V/Hz control of a 2.2-kW induction motor drive. The  six-step overmodulation is enabled, which increases the fundamental voltage as  well as the harmonics. Since the PWM is not synchronized with the stator  frequency, the harmonic content also depends on the ratio between the stator  frequency and the sampling frequency.">

.. only:: html

  .. image:: /auto_examples/vhz/images/thumb/sphx_glr_plot_vhz_ctrl_6step_im_2kw_thumb.png
    :alt:

  :ref:`sphx_glr_auto_examples_vhz_plot_vhz_ctrl_6step_im_2kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW induction motor, 6-step mode</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="A diode bridge, stiff three-phase grid, and a DC link is modeled. The default parameters in this example yield open-loop V/Hz control. ">

.. only:: html

  .. image:: /auto_examples/vhz/images/thumb/sphx_glr_plot_vhz_ctrl_im_2kw_thumb.png
    :alt:

  :ref:`sphx_glr_auto_examples_vhz_plot_vhz_ctrl_im_2kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW induction motor, diode bridge</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates open-loop V/Hz control of a 2.2-kW induction machine drive equipped with an LC filter. ">

.. only:: html

  .. image:: /auto_examples/vhz/images/thumb/sphx_glr_plot_vhz_ctrl_im_2kw_lc_thumb.png
    :alt:

  :ref:`sphx_glr_auto_examples_vhz_plot_vhz_ctrl_im_2kw_lc.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW induction motor, LC filter</div>
    </div>


.. thumbnail-parent-div-close

.. raw:: html

    </div>

Observer-Based V/Hz Control
---------------------------

These examples demonstrate observer-based V/Hz control for induction machines [#Tii2022a]_ and synchronous machines [#Tii2022b]_. The examples :doc:`/auto_examples/obs_vhz/plot_obs_vhz_ctrl_syrm_7kw` and :doc:`/auto_examples/obs_vhz/plot_obs_vhz_ctrl_pmsyrm_thor` also present the use of saturation models. The example :doc:`/auto_examples/obs_vhz/plot_obs_vhz_ctrl_pmsm_2kw_two_mass` demonstrates the use of a two-mass mechanics model. 

.. rubric:: References

.. [#Tii2022a] Tiitinen, Hinkkanen, Harnefors, "Stable and passive observer-based V/Hz control for induction motors," Proc. IEEE ECCE, Detroit, MI, Oct. 2022, https://doi.org/10.1109/ECCE50734.2022.9948057


.. [#Tii2022b] Tiitinen, Hinkkanen, Kukkola, Routimo, Pellegrino, Harnefors, "Stable and passive observer-based V/Hz control for synchronous Motors," Proc. IEEE ECCE, Detroit, MI, Oct. 2022, https://doi.org/10.1109/ECCE50734.2022.9947858



.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates observer-based V/Hz control of a 2.2-kW PMSM drive.">

.. only:: html

  .. image:: /auto_examples/obs_vhz/images/thumb/sphx_glr_plot_obs_vhz_ctrl_pmsm_2kw_thumb.png
    :alt:

  :ref:`sphx_glr_auto_examples_obs_vhz_plot_obs_vhz_ctrl_pmsm_2kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW PMSM</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates observer-based V/Hz control of a 2.2-kW induction motor drive.">

.. only:: html

  .. image:: /auto_examples/obs_vhz/images/thumb/sphx_glr_plot_obs_vhz_ctrl_im_2kw_thumb.png
    :alt:

  :ref:`sphx_glr_auto_examples_obs_vhz_plot_obs_vhz_ctrl_im_2kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW induction motor</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates observer-based V/Hz control of a saturated 6.7-kW synchronous reluctance motor drive. The saturation is not taken into account in  the control method (only in the system model).">

.. only:: html

  .. image:: /auto_examples/obs_vhz/images/thumb/sphx_glr_plot_obs_vhz_ctrl_syrm_7kw_thumb.png
    :alt:

  :ref:`sphx_glr_auto_examples_obs_vhz_plot_obs_vhz_ctrl_syrm_7kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">6.7-kW SyRM, saturated</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates observer-based V/Hz control of a 2.2-kW PMSM drive. The mechanical subsystem is modeled as a two-mass system. The resonance frequency of the mechanics is around 85 Hz. The mechanical parameters correspond to  [#Saa2015]_, except that the torsional damping is set to a smaller value in  this example.">

.. only:: html

  .. image:: /auto_examples/obs_vhz/images/thumb/sphx_glr_plot_obs_vhz_ctrl_pmsm_2kw_two_mass_thumb.png
    :alt:

  :ref:`sphx_glr_auto_examples_obs_vhz_plot_obs_vhz_ctrl_pmsm_2kw_two_mass.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW PMSM, 2-mass mechanics</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates observer-based V/Hz control of a saturated 5-kW permanent-magnet synchronous reluctance motor. The flux maps of this example motor, known as THOR, are from the SyR-e project:">

.. only:: html

  .. image:: /auto_examples/obs_vhz/images/thumb/sphx_glr_plot_obs_vhz_ctrl_pmsyrm_thor_thumb.png
    :alt:

  :ref:`sphx_glr_auto_examples_obs_vhz_plot_obs_vhz_ctrl_pmsyrm_thor.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">5-kW PM-SyRM, flux maps from SyR-e</div>
    </div>


.. thumbnail-parent-div-close

.. raw:: html

    </div>

Flux-Vector Control
-------------------

These examples demonstrate flux-vector control of synchronous machine drives 
[#Pel2009]_. In the implemented controller, rotor coordinates as well as 
decoupling between the stator flux and torque channels are used according to 
[#Awa2019]_. Furthermore, the stator flux magnitude and the electromagnetic 
torque are selected as controllable variables. 

.. rubric:: References

.. [#Pel2009] Pellegrino, Armando, Guglielmi, “Direct flux field-oriented 
   control of IPM drives with variable DC link in the field-weakening 
   region,” IEEE Trans.Ind. Appl., 2009, 
   https://doi.org/10.1109/TIA.2009.2027167

.. [#Awa2019] Awan, Hinkkanen, Bojoi, Pellegrino, "Stator-flux-oriented 
   control of synchronous motors: A systematic design procedure," IEEE Trans. 
   Ind. Appl., 2019, https://doi.org/10.1109/TIA.2019.2927316



.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless flux-vector control of a 2.2-kW PMSM drive.">

.. only:: html

  .. image:: /auto_examples/flux_vector/images/thumb/sphx_glr_plot_flux_vector_pmsm_2kw_thumb.png
    :alt:

  :ref:`sphx_glr_auto_examples_flux_vector_plot_flux_vector_pmsm_2kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW PMSM</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless stator-flux-vector control of a saturated 6.7-kW synchronous reluctance motor drive. The saturation is not taken into account in the control method (only in the system model). Even if the machine  has no magnets, the PM-flux disturbance estimation is enabled [#Tuo2018]_. In  this case, this PM-flux estimate lumps the effects of inductance errors.  Naturally, the PM-flux estimation can be used in PM machine drives as well. ">

.. only:: html

  .. image:: /auto_examples/flux_vector/images/thumb/sphx_glr_plot_flux_vector_syrm_7kw_thumb.png
    :alt:

  :ref:`sphx_glr_auto_examples_flux_vector_plot_flux_vector_syrm_7kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">6.7-kW SyRM, saturated, disturbance estimation</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless stator-flux-vector control of a 5.5-kW  PM-SyRM (Baldor ECS101M0H7EF4) drive. The machine model is parametrized using  the algebraic saturation model from [#Lel2024]_, fitted to the flux linkage  maps measured using the constant-speed test. For comparison, the measured data  is plotted together with the model predictions. Notice that the control system  used in this example does not consider the saturation, only the system model  does.">

.. only:: html

  .. image:: /auto_examples/flux_vector/images/thumb/sphx_glr_plot_flux_vector_pmsyrm_5kw_thumb.png
    :alt:

  :ref:`sphx_glr_auto_examples_flux_vector_plot_flux_vector_pmsyrm_5kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">5.5-kW PM-SyRM, saturated</div>
    </div>


.. thumbnail-parent-div-close

.. raw:: html

    </div>

Signal Injection
----------------

These examples demonstrate a square-wave signal injection for low-speed 
operation based on [#Kim2012]_. A phase-locked loop is used to track the rotor 
position. For a wider speed range, signal injection could be combined to a 
model-based observer. The effects of magnetic saturation are not compensated 
for in this version.

.. rubric:: References

.. [#Kim2012] Kim, Ha, Sul, "PWM switching frequency signal injection 
   sensorless method in IPMSM," IEEE Trans. Ind. Appl., 2012,
   https://doi.org/10.1109/TIA.2012.2210175



.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless vector control of a 2.2-kW PMSM drive. Square-wave signal injection is used with a simple phase-locked loop.">

.. only:: html

  .. image:: /auto_examples/signal_inj/images/thumb/sphx_glr_plot_signal_inj_pmsm_2kw_thumb.png
    :alt:

  :ref:`sphx_glr_auto_examples_signal_inj_plot_signal_inj_pmsm_2kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW PMSM</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless vector control of a 6.7-kW SyRM drive. Square-wave signal injection is used with a simple phase-locked loop.">

.. only:: html

  .. image:: /auto_examples/signal_inj/images/thumb/sphx_glr_plot_signal_inj_syrm_7kw_thumb.png
    :alt:

  :ref:`sphx_glr_auto_examples_signal_inj_plot_signal_inj_syrm_7kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">6.7-kW SyRM</div>
    </div>


.. thumbnail-parent-div-close

.. raw:: html

    </div>

Grid-Following Control
----------------------

These examples demonstrate grid-following control for grid-connected converters.


.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="10-kVA converter, LCL filter">

.. only:: html

  .. image:: /auto_examples/grid_following/images/thumb/sphx_glr_plot_gfl_lcl_10kva_thumb.png
    :alt:

  :ref:`sphx_glr_auto_examples_grid_following_plot_gfl_lcl_10kva.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">10-kVA converter, LCL filter</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="10-kVA converter, DC-bus voltage">

.. only:: html

  .. image:: /auto_examples/grid_following/images/thumb/sphx_glr_plot_gfl_dc_bus_10kva_thumb.png
    :alt:

  :ref:`sphx_glr_auto_examples_grid_following_plot_gfl_dc_bus_10kva.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">10-kVA converter, DC-bus voltage</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="10-kVA converter">

.. only:: html

  .. image:: /auto_examples/grid_following/images/thumb/sphx_glr_plot_gfl_10kva_thumb.png
    :alt:

  :ref:`sphx_glr_auto_examples_grid_following_plot_gfl_10kva.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">10-kVA converter</div>
    </div>


.. thumbnail-parent-div-close

.. raw:: html

    </div>

Grid-Forming Control
--------------------

These examples demonstrate grid-forming control for grid-connected converters. 
The example :doc:`/auto_examples/grid_forming/plot_gfm_rfpsc_13kva` uses a 
power-synchronization loop for synchronizing with the grid [#Har2020]_. In 
:doc:`/auto_examples/grid_forming/plot_gfm_obs_13kva`, disturbance-observer-
based control is used [#Nur2024]_.

.. rubric:: References

.. [#Har2020] Harnefors, Rahman, Hinkkanen, Routimo, "Reference-feedforward
   power-synchronization control," IEEE Trans. Power Electron., 2020,
   https://doi.org/10.1109/TPEL.2020.2970991

.. [#Nur2024] Nurminen, Mourouvin, Hinkkanen, Kukkola, "Multifunctional
   grid-forming converter control based on a disturbance observer, "IEEE
   Trans. Power Electron., 2024, https://doi.org/10.1109/TPEL.2024.3433503


.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="12.5-kVA converter, RFPSC">

.. only:: html

  .. image:: /auto_examples/grid_forming/images/thumb/sphx_glr_plot_gfm_rfpsc_13kva_thumb.png
    :alt:

  :ref:`sphx_glr_auto_examples_grid_forming_plot_gfm_rfpsc_13kva.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">12.5-kVA converter, RFPSC</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="12.5-kVA converter, disturbance observer">

.. only:: html

  .. image:: /auto_examples/grid_forming/images/thumb/sphx_glr_plot_gfm_obs_13kva_thumb.png
    :alt:

  :ref:`sphx_glr_auto_examples_grid_forming_plot_gfm_obs_13kva.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">12.5-kVA converter, disturbance observer</div>
    </div>


.. thumbnail-parent-div-close

.. raw:: html

    </div>


.. toctree::
   :hidden:
   :includehidden:


   /auto_examples/vector/index.rst
   /auto_examples/vhz/index.rst
   /auto_examples/obs_vhz/index.rst
   /auto_examples/flux_vector/index.rst
   /auto_examples/signal_inj/index.rst
   /auto_examples/grid_following/index.rst
   /auto_examples/grid_forming/index.rst


.. only:: html

  .. container:: sphx-glr-footer sphx-glr-footer-gallery

    .. container:: sphx-glr-download sphx-glr-download-python

      :download:`Download all examples in Python source code: auto_examples_python.zip </auto_examples/auto_examples_python.zip>`

    .. container:: sphx-glr-download sphx-glr-download-jupyter

      :download:`Download all examples in Jupyter notebooks: auto_examples_jupyter.zip </auto_examples/auto_examples_jupyter.zip>`


.. only:: html

 .. rst-class:: sphx-glr-signature

    `Gallery generated by Sphinx-Gallery <https://sphinx-gallery.github.io>`_
