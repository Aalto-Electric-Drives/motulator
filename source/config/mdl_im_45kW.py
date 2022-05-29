# pylint: disable=C0103
"""
This file contains the configuration data for the 45-kW induction motor drive.

"""
# %%
from model.im_drive import Motor, MotorSaturated, SaturationModel
from model.im_drive import Drive, Datalogger
from model.interfaces import PWM, Delay
from model.mech import Mechanics
from model.converter import Inverter

# %% Selectors
saturated = not True  # Enable the magnetic saturation model

# %% Computational delay and PWM
delay = Delay(1)
pwm = PWM(enabled=False)

# %% Mechanics
mech = Mechanics(J=1.66*.49, B=0)

# %% Motor model
if not saturated:
    # Inverse-Gamma model (no saturation)
    motor_data = {'R_s': 0.057,
                  'R_R': 0.029,
                  'L_sgm': 2.2e-3,
                  'L_M': 24.5e-3,
                  'p': 2}
    motor = Motor(**motor_data)
else:
    # Saturable Gamma umodel with the main-flux saturation model
    L_M = SaturationModel(L_unsat=29.4e-3, beta=.68, S=6.5)
    motor_data = {'R_s': .057,
                  'R_R': .034,
                  'L_sgm': 2.4e-3,
                  'L_M': L_M,
                  'p': 2}
    motor = MotorSaturated(**motor_data)

# %% Drive model
converter = Inverter(u_dc=540)
datalog = Datalogger()
mdl = Drive(motor, mech, converter, delay, pwm, datalog)
print(mdl)
