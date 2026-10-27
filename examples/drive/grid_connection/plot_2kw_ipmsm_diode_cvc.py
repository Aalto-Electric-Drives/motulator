"""
2.2-kW IPMSM, diode bridge, CVC
===============================

This example simulates sensorless current-vector control (CVC) of a 2.2-kW interior
permanent-magnet synchronous machine (IPMSM) drive, fed from a six-pulse diode bridge.
The diode bridge is supplied from a stiff grid through an AC choke.

"""

# %%
from math import pi, sqrt

import motulator.drive.control.sm as control
from motulator.common.model import CompositeModel, DCBus, Simulation, ThreePhaseSource
from motulator.drive import model as drive_model
from motulator.drive import utils as drive_utils
from motulator.grid import model as grid_model
from motulator.grid import utils as grid_utils

# %%
# Compute base values based on the nominal values.

nom = drive_utils.NominalValues(U=370, I=4.3, f=75, P=2.2e3, tau=14)
base = drive_utils.BaseValues.from_nominal(nom, n_p=3)

# %%
# Configure the system model. The diode bridge and the inverter share the DC bus, and
# their models are combined into a composite model.

dc_bus = DCBus(C_dc=235e-6, u_dc=sqrt(2) * 400)
grid_sys = grid_model.GridConverterSystem(
    grid_model.DiodeBridge(),
    grid_model.LFilter(L_f=0.5e-3),
    ThreePhaseSource(w_g=2 * pi * 50, e_g=sqrt(2 / 3) * 400),
    dc_bus=dc_bus,
)

par = drive_model.SynchronousMachinePars(
    n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545
)
machine = drive_model.SynchronousMachine(par)
mechanics = drive_model.MechanicalSystem(J=0.015)
converter = drive_model.VoltageSourceConverter()
drive_sys = drive_model.Drive(machine, mechanics, converter, pwm=True, dc_bus=dc_bus)

mdl = CompositeModel({"grid": grid_sys, "drive": drive_sys})

# %%
# Configure the control system.

est_par = par  # Assume accurate model parameter estimates
cfg = control.CurrentVectorControllerCfg(
    i_s_max=1.5 * base.i, alpha_o=2 * pi * 100, T_s=250e-6
)
vector_ctrl = control.CurrentVectorController(est_par, cfg)
speed_ctrl = control.SpeedController(J=0.015, alpha_s=2 * pi * 4)
ctrl = control.VectorControlSystem(vector_ctrl, speed_ctrl)

# %%
# Set the speed reference and the external load torque.

ctrl.set_speed_ref(lambda t: (t > 0.2) * base.w_M)
mechanics.set_external_load_torque(lambda t: (t > 0.6) * nom.tau)

# %%
# Create the simulation object, with the control system of the drive by the model
# name, and simulate. Plot the drive results in per-unit values.

sim = Simulation(mdl, {"drive": ctrl})
res = sim.simulate(t_stop=1)
res_drive = drive_model.SimulationResults(res.mdl.drive, res.ctrl.drive)
res_grid = grid_model.SimulationResults(res.mdl.grid, res.ctrl.grid)
# sphinx_gallery_thumbnail_number = 3
drive_utils.plot(res_drive, base)
drive_utils.plot_stator_waveforms(res_drive, base, t_lims=(0.8, 0.825))

# %%
# Plot the grid voltages and currents. The diode bridge conducts in pulses, as the
# DC-bus voltage is close to the peak of the line-to-line grid voltage.

grid_utils.plot_grid_waveforms(res_grid, t_lims=(0.8, 0.825), plot_pcc_voltage=False)
