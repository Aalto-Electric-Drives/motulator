motulator.grid.control.grid_following
=====================================

.. py:module:: motulator.grid.control.grid_following

.. autoapi-nested-parse::

   
   This package contains example controllers for grid following converters.
















   ..
       !! processed by numpydoc !!


Classes
-------

.. autoapisummary::

   motulator.grid.control.grid_following.CurrentController
   motulator.grid.control.grid_following.CurrentRefCalc
   motulator.grid.control.grid_following.GFLControl
   motulator.grid.control.grid_following.GFLControlCfg


Package Contents
----------------

.. py:class:: CurrentController(cfg)

   Bases: :py:obj:`motulator.common.control.ComplexPIController`


   
   2DOF PI current controller for grid converters.

   This class provides an interface for a current controller for grid
   converters. The gains are initialized based on the desired closed-loop
   bandwidth and the filter inductance.

   :param cfg:
               Control configuration parameters:

                   filter_par.L_fc : float
                       Converter-side filter inductance (H).
                   alpha_c : float
                       Closed-loop bandwidth (rad/s) of the current controller.
   :type cfg: GFLControlCfg















   ..
       !! processed by numpydoc !!

.. py:class:: CurrentRefCalc(cfg)

   
   Current controller reference generator

   This class is used to generate the current references for the current
   controllers based on the active and reactive power references. The current
   limiting algorithm is used to limit the current references.















   ..
       !! processed by numpydoc !!

   .. py:method:: get_current_reference(ref)

      
      Current reference generator.
















      ..
          !! processed by numpydoc !!


.. py:class:: GFLControl(cfg)

   Bases: :py:obj:`motulator.grid.control.GridConverterControlSystem`


   
   Grid-following control for power converters.

   :param cfg: Control configuration.
   :type cfg: GFLControlCfg

   .. attribute:: current_ctrl

      Current controller.

      :type: CurrentController

   .. attribute:: pll

      Phase locked loop.

      :type: PLL

   .. attribute:: current_reference

      Current reference calculator.

      :type: CurrentRefCalc















   ..
       !! processed by numpydoc !!

   .. py:method:: get_feedback_signals(mdl)

      
      Get the feedback signals.
















      ..
          !! processed by numpydoc !!


   .. py:method:: output(fbk)

      
      Extend the base class method.
















      ..
          !! processed by numpydoc !!


   .. py:method:: update(fbk, ref)

      
      Extend the base class method.
















      ..
          !! processed by numpydoc !!


.. py:class:: GFLControlCfg

   
   Grid-following control configuration

   :param grid_par: Grid model parameters.
   :type grid_par: GridPars
   :param filter_par: Filter parameters.
   :type filter_par: FilterPars
   :param max_i: Maximum current (A).
   :type max_i: float
   :param T_s: Sampling period (s). The default is 100e-6.
   :type T_s: float, optional
   :param alpha_c: Current-control bandwidth (rad/s). The default is 2*pi*400.
   :type alpha_c: float, optional
   :param alpha_ff: Low-pass-filtering bandwidth (rad/s) for the voltage-feedforward term.
                    The default is 2*pi*200.
   :type alpha_ff: float, optional
   :param w0_pll: Undamped natural frequency of the PLL. The default is 2*pi*20.
   :type w0_pll: float, optional
   :param zeta_pll: Damping ratio of the PLL. The default is 1.
   :type zeta_pll: float, optional
   :param C_dc: DC-bus capacitance (F). The default is None.
   :type C_dc: float, optional















   ..
       !! processed by numpydoc !!

