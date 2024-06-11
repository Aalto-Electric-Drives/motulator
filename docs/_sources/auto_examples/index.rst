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

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless current-vector control of a 2.2-kW PMSM  drive, equipped with...">

.. only:: html

  .. image:: /auto_examples/vector/images/thumb/sphx_glr_plot_vector_ctrl_pmsm_2kw_diode_thumb.png
    :alt:

  :ref:`sphx_glr_auto_examples_vector_plot_vector_ctrl_pmsm_2kw_diode.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW PMSM, diode bridge</div>
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

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates current-vector control of a 2.2-kW induction motor drive in torque-contr...">

.. only:: html

  .. image:: /auto_examples/vector/images/thumb/sphx_glr_plot_vector_ctrl_im_2kw_tq_mode_thumb.png
    :alt:

  :ref:`sphx_glr_auto_examples_vector_plot_vector_ctrl_im_2kw_tq_mode.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW induction motor, torque-control mode</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless current-vector control of a 5-kW permanent- magnet synchronou...">

.. only:: html

  .. image:: /auto_examples/vector/images/thumb/sphx_glr_plot_vector_ctrl_pmsyrm_thor_thumb.png
    :alt:

  :ref:`sphx_glr_auto_examples_vector_plot_vector_ctrl_pmsyrm_thor.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">5-kW PM-SyRM</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless current-vector control of a 2.2-kW induction  motor drive. Th...">

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

    <div class="sphx-glr-thumbcontainer" tooltip="A diode bridge, stiff three-phase grid, and a DC link is modeled. The default parameters in thi...">

.. only:: html

  .. image:: /auto_examples/vhz/images/thumb/sphx_glr_plot_vhz_ctrl_im_2kw_thumb.png
    :alt:

  :ref:`sphx_glr_auto_examples_vhz_plot_vhz_ctrl_im_2kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW induction motor, diode bridge</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates V/Hz control of a 2.2-kW induction motor drive. The  six-step overmodula...">

.. only:: html

  .. image:: /auto_examples/vhz/images/thumb/sphx_glr_plot_vhz_ctrl_6step_im_2kw_thumb.png
    :alt:

  :ref:`sphx_glr_auto_examples_vhz_plot_vhz_ctrl_6step_im_2kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW induction motor, 6-step mode</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates open-loop V/Hz control of a 2.2-kW induction machine drive equipped with...">

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

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates observer-based V/Hz control of a saturated 6.7-kW synchronous reluctance...">

.. only:: html

  .. image:: /auto_examples/obs_vhz/images/thumb/sphx_glr_plot_obs_vhz_ctrl_syrm_7kw_thumb.png
    :alt:

  :ref:`sphx_glr_auto_examples_obs_vhz_plot_obs_vhz_ctrl_syrm_7kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">6.7-kW SyRM, saturated</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates observer-based V/Hz control of a 2.2-kW PMSM drive. The mechanical subsy...">

.. only:: html

  .. image:: /auto_examples/obs_vhz/images/thumb/sphx_glr_plot_obs_vhz_ctrl_pmsm_2kw_two_mass_thumb.png
    :alt:

  :ref:`sphx_glr_auto_examples_obs_vhz_plot_obs_vhz_ctrl_pmsm_2kw_two_mass.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW PMSM, 2-mass mechanics</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates observer-based V/Hz control of a saturated 5-kW permanent-magnet synchro...">

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

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless stator-flux-vector control of a saturated 6.7-kW synchronous ...">

.. only:: html

  .. image:: /auto_examples/flux_vector/images/thumb/sphx_glr_plot_flux_vector_syrm_7kw_thumb.png
    :alt:

  :ref:`sphx_glr_auto_examples_flux_vector_plot_flux_vector_syrm_7kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">6.7-kW SyRM, saturated, disturbance estimation</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless stator-flux-vector control of a 5.5-kW  PM-SyRM (Baldor ECS10...">

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
model-based observer. The effects of magnetic saturation are not compensated for in 
this version.

.. rubric:: References

.. [#Kim2012] Kim, Ha, Sul, "PWM switching frequency signal injection 
   sensorless method in IPMSM," IEEE Trans. Ind. Appl., 2012,
   https://doi.org/10.1109/TIA.2012.2210175



.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless vector control of a 2.2-kW PMSM drive. Square-wave signal inj...">

.. only:: html

  .. image:: /auto_examples/signal_inj/images/thumb/sphx_glr_plot_signal_inj_pmsm_2kw_thumb.png
    :alt:

  :ref:`sphx_glr_auto_examples_signal_inj_plot_signal_inj_pmsm_2kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW PMSM</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless vector control of a 6.7-kW SyRM drive. Square-wave signal inj...">

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


.. toctree::
   :hidden:
   :includehidden:


   /auto_examples/vector/index.rst
   /auto_examples/vhz/index.rst
   /auto_examples/obs_vhz/index.rst
   /auto_examples/flux_vector/index.rst
   /auto_examples/signal_inj/index.rst


.. only:: html

  .. container:: sphx-glr-footer sphx-glr-footer-gallery

    .. container:: sphx-glr-download sphx-glr-download-python

      :download:`Download all examples in Python source code: auto_examples_python.zip </auto_examples/auto_examples_python.zip>`

    .. container:: sphx-glr-download sphx-glr-download-jupyter

      :download:`Download all examples in Jupyter notebooks: auto_examples_jupyter.zip </auto_examples/auto_examples_jupyter.zip>`


.. only:: html

 .. rst-class:: sphx-glr-signature

    `Gallery generated by Sphinx-Gallery <https://sphinx-gallery.github.io>`_
