# pylint: disable=C0103
"""
This file contains the configuration data for the 2.2-kW induction motor drive.

"""
# %%
import numpy as np
from model.im_drive import Motor, MotorSaturated, SaturationModel
from model.im_drive import Drive, Datalogger
from model.im_drive import DriveWithDiodeBridge, DataloggerExtended
from model.interfaces import PWM, Delay
from model.mech import Mechanics
from model.converter import Inverter, FrequencyConverter


# %% Selectors
saturated = not True  # Enable the magnetic saturation model
constant_dc_voltage = True  # Enable constant DC-bus voltage

# %% Computational delay and PWM
delay = Delay(1)
pwm = PWM(enabled=False)

# %% Mechanics
mech = Mechanics(J=.015, B=0)

# %% Motor model
if not saturated:
    # Inverse-Gamma model (no saturation)
    motor_data = {'R_s': 3.7,
                  'R_R': 2.1,
                  'L_sgm': 0.021,
                  'L_M': 0.224,
                  'p': 2}
    motor = Motor(**motor_data)
else:
    # Saturable Gamma umodel with the main-flux saturation model
    L_M = SaturationModel(L_unsat=.34, beta=.84, S=7)
    motor_data = {'R_s': 3.7,
                  'R_R': 2.5,
                  'L_sgm': 0.023,
                  'L_M': L_M,
                  'p': 2}
    motor = MotorSaturated(**motor_data)

# %% Drive model
if constant_dc_voltage:
    converter = Inverter(u_dc=540)
    datalog = Datalogger()
    mdl = Drive(motor, mech, converter, delay, pwm, datalog)
else:
    # Note: Braking chopper is not modeled, so regenerating may increase
    # the DC-bus voltage to unrealistically high values
    converter_data = {'C': 235e-6,
                      'L': 2e-3,
                      'u_g': np.sqrt(2/3)*400,
                      'w_g': 2*np.pi*50}
    converter = FrequencyConverter(**converter_data)
    datalog = DataloggerExtended()
    mdl = DriveWithDiodeBridge(motor, mech, converter, delay, pwm, datalog)

