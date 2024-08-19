DC-Bus Voltage Control
======================
The DC-bus voltage control uses a two-degrees-of-freedom (2DOF) proportional-integral (PI) controller. 
The PI controller is designed to control the energy of the DC-bus capacitance 
and not the DC-bus voltage in order to have a linear closed-loop system [#Hur2001]_.

.. figure:: ../figs/DC_bus_dynamics.svg
   :width: 100%
   :align: center
   :alt: DC-Bus DC_bus_dynamics
   :target: .
   
   DC-Bus dynamics.

In this model the converter is assumed to be ideal and the DC-bus capacitance is modeled as a first-order system

.. math::
   C \frac{\mathrm{d}u_{\mathrm{dc}}}{\mathrm{d} t} = i_{\mathrm{dc}} - \frac{p_{\mathrm{c}}}{u_{\mathrm{dc}}}
   :label: du/dt

which can be rewritten using the capacitor energy as

.. math::
   \frac{\mathrm{d}W_{\mathrm{dc}}}{\mathrm{d} t} = u_{\mathrm{dc}}i_{\mathrm{dc}} - p_{\mathrm{c}} = u_{\mathrm{dc}}i_{\mathrm{dc}} - \frac{\mathrm{d} W_\mathrm{f}}{\mathrm{d} t} - p_{\mathrm{g}}\\
   :label: dW/dt

where

.. math::
   p_{\mathrm{c}}=\frac{3}{2}\mathrm{Re}\{\boldsymbol{u}_{\mathrm{c}}\boldsymbol{i}_{\mathrm{c}}^*\} \qquad
   W_\mathrm{{dc}} = \frac{1}{2}C u_{\mathrm{dc}}^2 \qquad
   W_\mathrm{f} = \frac{3}{4}L_\mathrm{f} i_{\mathrm{dc}}^2 

In ideal power control :math:`p_{\mathrm{g}} = p_{\mathrm{g,ref}}`. 
Also, :math:`\frac{\mathrm{d}W}{\mathrm{d} t}=0` since the filter inductor energy is assumed to be constant. 
The power :math:`u_{\mathrm{dc}}i_{\mathrm{dc}}` act as a load disturbance.
    
The DC-bus voltage control is implemented in the class :class:`motulator.grid.control.DCBusVoltageController`.

.. rubric:: References
    
.. [#Hur2001] Hur, Jung, Nam, "A Fast Dynamic DC-Link Power-Balancing
       Scheme for a PWM Converter–Inverter System," IEEE Trans. Ind. Electron.,
       2001, https://doi.org/10.1109/41.937412