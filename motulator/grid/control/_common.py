"""Common control functions and classes."""

import numpy as np
#from motulator.common.utils._utils import (abc2complex, complex2abc)
from motulator.common.control._control import (Clock, PICtrl)
from gritulator._utils import Bunch


# %%
class DCBusVoltCtrl(PICtrl):
    """
    PI DC-bus voltage controller.

    This provides an interface for a DC-bus controller. The gains are
    initialized based on the desired closed-loop bandwidth and the DC-bus
    capacitance estimate. The PI controller is designed to control the energy
    of the DC-bus capacitance and not the DC-bus voltage in order to have a
    linear closed-loop system [#Hur2001]_.

    Parameters
    ----------
    zeta : float
        Damping ratio of the closed-loop system.
    alpha_dc : float
        Closed-loop bandwidth (rad/s). 
    p_max : float, optional
        Maximum converter power (W). The default is inf.
        
    References
    ----------
    .. [#Hur2001] Hur, Jung, Nam, "A Fast Dynamic DC-Link Power-Balancing
       Scheme for a PWM Converter–Inverter System," IEEE Trans. Ind. Electron.,
       2001, https://doi.org/10.1109/41.937412

    """

    def __init__(self, zeta, alpha_dc, p_max=np.inf):
        k_p = -2*zeta*alpha_dc
        k_i = -(alpha_dc**2)
        k_t = k_p
        super().__init__(k_p, k_i, k_t, p_max)


# %%
class Ctrl:
    """Base class for the control system."""

    def __init__(self):
        self.data = Bunch()  # Data store
        self.clock = Clock()  # Digital clock

    def __call__(self, mdl):
        """
        Run the main control loop.

        The main control loop is callable that returns the sampling
        period `T_s` (float)  and the duty ratios `d_abc` (ndarray, shape (3,)) 
        for the next sampling period.

        Parameters
        ----------
        mdl : Model
            System model containing methods for getting the feedback signals.

        """
        raise NotImplementedError

    def save(self, data):
        """
        Save the internal date of the control system.

        Parameters
        ----------
        data : bunch or dict
            Contains the data to be saved.

        """
        for key, value in data.items():
            self.data.setdefault(key, []).extend([value])

    def post_process(self):
        """
        Transform the lists to the ndarray format.

        This method can be run after the simulation has been completed in order 
        to simplify plotting and analysis of the stored data.

        """
        for key in self.data:
            self.data[key] = np.asarray(self.data[key])
