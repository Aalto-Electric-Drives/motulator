"""Import simulation environment."""
import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# pylint: disable=wrong-import-position
from motulator.simulation import Simulation

# Import system models
from motulator.model.mech import Mechanics
from motulator.model.converter import (
    Inverter,
    FrequencyConverter,
)
from motulator.model.im import (
    InductionMotor,
    InductionMotorSaturated,
    InductionMotorInvGamma,
)
from motulator.model.sm import (
    SynchronousMotor,
    SynchronousMotorSaturated,
    SynchronousMotorSaturatedLUT,
)
from motulator.model.im_drive import (
    InductionMotorDrive,
    InductionMotorDriveDiode,
)
from motulator.model.sm_drive import SynchronousMotorDrive

from motulator.model.sm_flux_maps import (
    import_syre_data,
    plot_flux_map,
    plot_flux_vs_current,
    invert_flux_map,
    downsample_flux_map,
)

# Import controllers
from motulator.control.im_vhz import (
    InductionMotorVHzCtrl,
    InductionMotorVHzCtrlPars,
)
from motulator.control.im_vector import (
    InductionMotorVectorCtrl,
    InductionMotorVectorCtrlPars,
)
from motulator.control.sm_vector import (
    SynchronousMotorVectorCtrl,
    SynchronousMotorVectorCtrlPars,
    TorqueCharacteristics,
)
from motulator.control.sm_flux_vector import (
    SynchronousMotorFluxVectorCtrl,
    SynchronousMotorFluxVectorCtrlPars,
)
from motulator.control.sm_signal_inj import (
    SynchronousMotorSignalInjectionCtrl,
    SynchronousMotorSignalInjectionCtrlPars,
)

from motulator.control.im_obs_vhz import (
    InductionMotorVHzObsCtrl,
    InductionMotorObsVHzCtrlPars,
)

from motulator.control.sm_obs_vhz import (
    SynchronousMotorVHzObsCtrl,
    SynchronousMotorVHzObsCtrlPars,
)

# Import other useful stuff
from motulator.helpers import (
    BaseValues,
    abc2complex,
    complex2abc,
    Sequence,
    Step,
)

# Import some default plotting functions
from motulator.plots import (
    plot,
    plot_extra,
)

# Delete imported modules
del sys, os
