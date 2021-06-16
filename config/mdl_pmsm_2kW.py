# pylint: disable=C0103
"""
This file contains the configuration data for the 2.2-kW permanent-magnet
synchronous motor drive.

"""
# %%
import numpy as np
from helpers import Sequence
from model.sm_drive import Motor, Drive, Datalogger
from model.interfaces import PWM, Delay
from model.mech import Mechanics
from model.converter import Inverter

# %% Selectors
delay_length = 1  # Computational delay
pwm_modeled = not True  # Enable the carrier comparison

# %% Mechanics
# Define the external load torque
times = np.array([0, .5, .5, 3.5, 3.5, 4])
values = np.array([0, 0, 1, 1, 0, 0])*14.6
T_L_ext = Sequence(times, values)  # T_L_ext = Step(1, 14.6)
mech_data = {'J': 0.015,
             'B': 0,
             'T_L_ext': T_L_ext}
mech = Mechanics(**mech_data)

# %% Computational delay and PWM
delay = Delay(delay_length)
if pwm_modeled:
    pwm = PWM()
else:
    pwm = None

# %% Motor model
motor_data = {'R': 3.6,
              'L_d': 0.036,
              'L_q': 0.051,
              'psi_f': 0.545,
              'p': 3}
motor = Motor(mech, **motor_data)

# %% Drive model
converter = Inverter(u_dc=540)
datalog = Datalogger()
mdl = Drive(motor, mech, converter, delay, pwm, datalog)

# %% Print the system data
print('\nSystem: 2.2-kW permanent-magnet synchronous motor drive')
print('-------------------------------------------------------')
print(delay)
if pwm_modeled:
    print(pwm)
else:
    print('PWM model:\n    disabled')
print(converter)
print(motor)
print(mech)
