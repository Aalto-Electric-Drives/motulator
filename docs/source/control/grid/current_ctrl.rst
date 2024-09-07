Current Control
===============

Synchronous-frame two-degrees-of-freedom (2DOF) proportional-integral (PI) current control can be used in grid converters [#Har2015]_. This control structure allows to compensate for the cross-coupling originating from rotating coordinates as well as to improve disturbance rejection. A 2DOF PI current controller is available in the :class:`motulator.grid.control.CurrentController` class, whose base class is :class:`motulator.common.control.ComplexPIController`. 

.. note:: This controller design assumes an L filter, but it can also be applied with LCL filters (see the :doc:`/grid_examples/grid_following/plot_gfl_lcl_10kva` example). If LCL-resonance damping and very low sampling frequencies are needed, the controller could be designed directly in the discrete-time domain taking the LCL filter dynamics into account [#Rah2021]_.

2DOF PI Controller
------------------

The design of synchronous-frame 2DOF PI current control is considered in the continuous-time domain, even though the actual implementation is discrete. Two typical gain selections for this control type are known as the internal-model-control (IMC) design [#Har1998]_ and the complex-vector design [#Bri1999]_. Here, only the complex-vector design is considered, see :ref:`complex-vector-2dof-pi-controller`, which is compatible with the :class:`motulator.common.control.ComplexPIController` base class. The controller can be expressed in a state-space form as

.. math::
	\frac{\mathrm{d} \boldsymbol{u}_\mathrm{i}}{\mathrm{d} t} &= (\boldsymbol{k}_\mathrm{i} + \mathrm{j}\omega_\mathrm{c}\boldsymbol{k}_\mathrm{t} )\left(\boldsymbol{i}_\mathrm{c,ref} - \boldsymbol{i}_\mathrm{c}\right) \\
    \boldsymbol{u}_\mathrm{c,ref} &= \boldsymbol{k}_\mathrm{t}\boldsymbol{i}_\mathrm{c,ref} - \boldsymbol{k}_\mathrm{p}\boldsymbol{i}_\mathrm{c} + \boldsymbol{u}_\mathrm{i} 
    :label: grid_cc

where :math:`\boldsymbol{u}_\mathrm{c,ref}` is the output of the controller, i.e., the converter voltage reference, :math:`\boldsymbol{i}_\mathrm{c}` is the measured converter current, :math:`\boldsymbol{i}_\mathrm{c,ref}` is the converter current reference, :math:`\boldsymbol{u}_\mathrm{i}` is the the integral state, and :math:`\omega_\mathrm{c}` is the angular speed of the coordinate system. Furthermore, :math:`\boldsymbol{k}_\mathrm{t}` is the reference-feedforward gain, :math:`\boldsymbol{k}_\mathrm{p}` is the proportional gain, and :math:`\boldsymbol{k}_\mathrm{i}` is the integral gain. 

Closed-Loop System 
^^^^^^^^^^^^^^^^^^

Consider the grid model in synchronous coordinates

.. math::
   L\frac{\mathrm{d}\boldsymbol{i}_\mathrm{c}}{\mathrm{d} t} = \boldsymbol{u}_\mathrm{c} - \boldsymbol{u}_\mathrm{g} - \mathrm{j} \omega_\mathrm{c} L \boldsymbol{i}_\mathrm{c}
   :label: L_filt_sync

where :math:`\boldsymbol{u}_\mathrm{c}` is the converter output voltage, :math:`\boldsymbol{u}_\mathrm{g}` is the grid (or PCC) voltage, and :math:`L` is the inductance. Ideal converter voltage production is assumed, :math:`\boldsymbol{u}_\mathrm{c} = \boldsymbol{u}_\mathrm{c,ref}`. Using :eq:`grid_cc` and :eq:`L_filt_sync`, the closed-loop system in the Laplace domain becomes

.. math::
	\boldsymbol{i}_\mathrm{c} = \boldsymbol{G}_\mathrm{c}(s)\boldsymbol{i}_\mathrm{c,ref} - \boldsymbol{Y}_\mathrm{c}(s)\boldsymbol{u}_\mathrm{g}

The disturbance rejection depends on the closed-loop admittance

.. math::
    \boldsymbol{Y}_\mathrm{c}(s) = \frac{s}{L s^2 + (\boldsymbol{k}_\mathrm{p} + \mathrm{j}\omega_\mathrm{c} L) s + \boldsymbol{k}_\mathrm{i} + \mathrm{j}\omega_\mathrm{c} \boldsymbol{k}_\mathrm{t}} 
    :label: Yc_grid

The closed-loop poles can be arbitrarily placed by means of the gains. The reference-tracking transfer function is

.. math::
	\boldsymbol{G}_\mathrm{c}(s) = \frac{(s + \mathrm{j}\omega_\mathrm{c}) \boldsymbol{k}_\mathrm{t} + \boldsymbol{k}_\mathrm{i} }{L s^2 + (\boldsymbol{k}_\mathrm{p} + \mathrm{j}\omega_\mathrm{c} L) s + \boldsymbol{k}_\mathrm{i} + \mathrm{j}\omega_\mathrm{c} \boldsymbol{k}_\mathrm{t}}     
    :label: Gc_grid

whose zero can be placed by means of the reference-feedforward gain :math:`\boldsymbol{k}_\mathrm{t}`.

Gain Selection
^^^^^^^^^^^^^^

Consider the gains

.. math::                
    \boldsymbol{k}_\mathrm{p} = 2\alpha_\mathrm{c} \hat L \qquad\qquad
    \boldsymbol{k}_\mathrm{i} = \alpha_\mathrm{c}^2\hat L  \qquad \qquad
    \boldsymbol{k}_\mathrm{t} = \alpha_\mathrm{c} \hat L

where :math:`\hat L` is the inductance estimate. Assuming accurate parameter estimates, the closed-loop transfer functions :eq:`Yc_grid` and :eq:`Gc_grid` reduce to

.. math::
    \boldsymbol{Y}_\mathrm{c}(s) = \frac{s}{L (s + \alpha_\mathrm{c})(s + \alpha_\mathrm{c} + \mathrm{j}\omega_\mathrm{c})}
    \qquad\qquad
    \boldsymbol{G}_\mathrm{c}(s) = \frac{\alpha_\mathrm{c}}{s + \alpha_\mathrm{c}} 

It can be seen that this design results in the first-order reference-tracking dynamics. Furthermore, one pole is placed at the real axis at :math:`s=-\alpha_\mathrm{c}` and another pole at :math:`s= -\alpha_\mathrm{c} - \mathrm{j}\omega_\mathrm{c}`. This gain selection is used in the :class:`motulator.grid.control.CurrentController` class. 

The converter output voltage is limited in practice due to the limited DC-bus voltage of the converter. Consequently, the realized (limited) voltage reference is

.. math::
    \bar{\boldsymbol{u}}_\mathrm{c,ref} = \mathrm{sat}(\boldsymbol{u}_\mathrm{c,ref})

where :math:`\mathrm{sat}(\cdot)` is the saturation function. The limited voltage can be obtained from a pulse-width modulation (PWM) algorithm (see the :class:`motulator.common.control.PWM` class). The anti-windup of the integrator is included in the implementation of the :class:`motulator.common.control.ComplexPIController` base class.

.. rubric:: References

.. [#Har2015] Harnefors, Yepes, Vidal, Doval-Gandoy, "Passivity-based controller design of grid-connected VSCs for prevention of electrical resonance instability," IEEE Trans. Ind. Electron., 2015, https://doi.org/10.1109/TIE.2014.2336632

.. [#Rah2021] Rahman, Pirsto, Kukkola, Hinkkanen, Pérez-Estévez, Doval-Gandoy, "Equivalence of the integrator-based and disturbance-observer-based state-space current controllers for grid converters," IEEE Trans. Ind. Electron., 2021, https://doi.org/10.1109/TIE.2020.2988194

.. [#Har1998] Harnefors, Nee, "Model-based current control of AC machines using the internal model control method," IEEE Trans. Ind. Appl., 1998, https://doi.org/10.1109/28.658735

.. [#Bri1999] Briz del Blanco, Degner, Lorenz, “Dynamic analysis of current regulators for AC motors using complex vectors,” IEEE Trans. Ind. Appl., 1999, https://doi.org/10.1109/28.806058



