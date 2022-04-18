# pylint: disable=C0103
"""
This file contains the configuration data for the 2.2-kW permanent-magnet
synchronous motor drive.

"""
# %%
from model.sm_drive import Motor, Drive, Datalogger
from model.interfaces import PWM, Delay
from model.mech import Mechanics
from model.converter import Inverter

# %% Computational delay and PWM
delay = Delay(1)
pwm = PWM(enabled=False)

# %% Mechanics
mech = Mechanics(J=.015, B=0)

# %% Motor model
motor_data = {'R_s': 3.6,
              'L_d': 0.036,
              'L_q': 0.051,
              'psi_f': 0.545,
              'p': 3}
motor = Motor(mech, **motor_data)

# %% Drive model
converter = Inverter(u_dc=540)
datalog = Datalogger()
mdl = Drive(motor, mech, converter, delay, pwm, datalog)
print(mdl)
