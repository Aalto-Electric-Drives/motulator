Tutorial
========
Welcome to motulator's tutorial!
--------------------------------
This tutorial is an in depth version of :doc:`usage`. This tutorial features a how-to guide for motulator features, such as running multiple simulations in parallel and the configuring of the motor and control parameters.

How to configure parameters
-----------------------------------------
Motulator simulation tool is based on the control of an induction motor
or synchronous motor drive. Motulator includes scalar control (or
Volts-per-Hertz-control) and rotor-flux-oriented vector control. In
order to build the link between the continuous-time design and
discrete-time implementation, all the control algorithms are implemented
in the discrete-time domain based on forward Euler approximation.

The module motulator/model/im.py describes the induction motor model
data according to the Γ model (Gamma model). The Γ model is chosen,
since it can be extended with the magnetic saturation model in a
staightforward manner. If the magnetic saturation is omitted, the Γ
model is mathematically identical to the inverse-Γ model of the
induction motor, which is shown below:

.. image:: Inverse-gamma_model.png
   :scale: 100%
   :alt: alternate text
   :target: .

Motor model can be created by constructing the induction motor object as
shown below:

.. code::

    import motulator as mt

.. code::

    # Motor model with custom parameters
    motor = mt.InductionMotor(R_s=3.7, R_r=2.5, L_ell=.023, L_s=.245, p=2)

Here the motor parameters mean the following: R_s is stator resistance,
R_r is rotor resistance, L_ell is leakage inductance, L_s is stator
inductance and p is the number of pole pairs. The value of parameters
R_s, R_r, L_ell, L_s and p can be configured by changing their values
when passing to mt.InductionMotor constructor.

The rated values of the motor are important for configuring how figures
are plotted:

.. code::

    # Compute base values based on the nominal values (just for figures)
    base = mt.BaseValues(U_nom=400,        # Line-line rms voltage
                         I_nom=5,          # Rms current
                         f_nom=50,         # Frequency
                         tau_nom=14.6,     # Torque
                         P_nom=2.2e3,      # Power
                         p=2)              # Number of pole pairs

To configure the mechanics model and converter model parameters, the
following parameters are passed into their class constructors:

.. code::

    mech = mt.Mechanics(J=.015)         # Mechanics model with custom parameters
    conv = mt.Inverter(u_dc0=540)       # Inverter model with custom parameters

Here J is the total moment of inertia of the motor and u_dc0 is the
DC-bus voltage of inverter. After all of that has been configured, the
continuous-time model for an induction motor drive can be constructed
with the following command:

.. code::

    mdl = mt.InductionMotorDrive(motor, mech, conv)  # System model

This interconnects the subsystems of an induction motor drive and
provides an interface for the solver. More complicated systems could be
modeled using a similar template.

Motulator includes Volts-per-Hertz-control and rotor-flux-oriented
vector control, which are configured similarly to how the system model
is configured. In the case of vector control, the configuration for the
parameters of the control system can look something like this:

.. code::

    import numpy as np

.. code::

    # Control system
    ctrl = mt.InductionMotorVectorCtrl(mt.InductionMotorVectorCtrlPars(
        sensorless=True,                # Enable sensorless mode
        T_s=250e-6,                     # Sampling period
        delay=1,                        # Amount of computational delay
        alpha_c=2*np.pi*200,            # Current-control bandwidth
        alpha_o=2*np.pi*40,             # Observer bandwidth
        alpha_s=2*np.pi*4,              # Speed-control bandwidth
        psi_R_nom=.9,                   # Nominal rotor flux
        i_s_max=1.5*base.i,             # Current limit
        tau_M_max=1.5*base.tau_nom,     # Torque limit (for the speed ctrl)
        J=.015,                         # Inertia estimate (for the speed ctrl)
        p=2,                            # Number of pole pairs
        # Inverse-Gamma model parameter estimates
        R_s=3.7, R_R=2.1, L_sgm=.021, L_M=.224))

Speed reference and the external load torque for the induction motors
can be configured in this way:

.. code::

    # Set the speed reference and the external load torque
    ctrl.w_m_ref = lambda t: (t > .2)*(.5*base.w)
    mdl.mech.tau_L_ext = lambda t: (t > .75)*base.tau_nom

Simulation object has a simulate function, that solves the
continuous-time model and calls the discrete-time controller. Base
values for plotting figures is determined by base parameter and the
simulation stop time is determined by t_stop parameter (which is 1 by
default). Simulation object is created as follows:

.. code::

    # Create the simulation object
    sim = mt.Simulation(mdl, ctrl, base=base, t_stop=1.5)

To simulate the simulation object, run the command:

.. code::

    sim.simulate()

Plotting can be done either by plotting figures in SI units:

.. code::

    mt.plot(sim)

Or alternatively plotting figures in per units:

.. code::

    mt.plot_pu(sim)

In this tutorial, induction motor was used to simulate the model. However, motulator also supports functionality for synchronous motors
and more. There are example scripts in `examples <https://github.com/Aalto-Electric-Drives/motulator/tree/main/examples>`_ folder that show
similar configurations for different motor types. More detailed information on configuration parameters can be found from :ref:`contents`.

.. note::

   The full version of this tutorial is not finished yet. The tutorial continues from here as more functionalities are later added to motulator.
