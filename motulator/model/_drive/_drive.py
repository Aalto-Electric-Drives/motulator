"""
Continuous-time models for electric machine drives.

Peak-valued complex space vectors are used.

"""
from motulator.model._simulation import Model


# %%
class Drive(Model):
    """
    Continuous-time model for an induction machine drive.

    This interconnects the subsystems of an induction machine drive and 
    provides an interface to the solver. 

    Parameters
    ----------
    converter : Inverter
        Inverter model.
    machine : InductionMachine 
        Induction machine model.
    mechanics : Mechanics
        Mechanics model.

    """

    def __init__(self, converter=None, machine=None, mechanics=None):
        super().__init__()
        self.converter = converter
        self.machine = machine
        self.mechanics = mechanics
        self.subsystems = [self.converter, self.machine, self.mechanics]

    def interconnect(self, _):
        """Interconnect the subsystems."""
        self.converter.inp.i_cs = self.machine.out.i_ss
        self.machine.inp.u_ss = self.converter.out.u_cs
        self.machine.inp.w_M = self.mechanics.out.w_M
        self.mechanics.inp.tau_M = self.machine.out.tau_M

    def post_process(self):
        """Post-process the solution."""
        # Post-processing based on the states
        super().post_process_states()
        # Add the input data to the subsystems for post-processing
        self.converter.data.i_cs = self.machine.data.i_ss
        self.machine.data.u_ss = self.converter.data.u_cs
        self.machine.data.w_M = self.mechanics.data.w_M
        self.mechanics.data.tau_M = self.machine.data.tau_M
        # Post-processing based on the inputs and the states
        super().post_process_with_inputs()


# %%
class DriveWithLCFilter(Model):
    """
    Induction machine drive with an output LC filter.

    Parameters
    ----------
    machine : InductionMachine | InductionMachineSaturated
        Induction machine model.
    mechanics : Mechanics
        Mechanics model.
    converter : Inverter
        Inverter model.
    lc_filter : LCFilter
        LC-filter model.

    """

    def __init__(
            self,
            converter=None,
            machine=None,
            mechanics=None,
            lc_filter=None):
        super().__init__()
        self.converter = converter
        self.machine = machine
        self.mechanics = mechanics
        self.lc_filter = lc_filter
        self.subsystems = [
            self.converter, self.machine, self.mechanics, self.lc_filter
        ]

    def interconnect(self, _):
        """Interconnect the subsystems."""
        self.converter.inp.i_cs = self.lc_filter.out.i_cs
        self.lc_filter.inp.i_fs = self.machine.out.i_ss
        self.lc_filter.inp.u_cs = self.converter.out.u_cs
        self.machine.inp.u_ss = self.lc_filter.out.u_fs
        self.machine.inp.w_M = self.mechanics.out.w_M
        self.mechanics.inp.tau_M = self.machine.out.tau_M

    def post_process(self):
        """Post-process the solution."""
        # Post-processing based on the states
        super().post_process_states()
        # Add the input data to the subsystems for post-processing
        self.converter.data.i_cs = self.lc_filter.data.i_cs
        self.lc_filter.data.i_fs = self.machine.data.i_ss
        self.lc_filter.data.u_cs = self.converter.data.u_cs
        self.machine.data.u_ss = self.lc_filter.data.u_fs
        self.machine.data.w_M = self.mechanics.data.w_M
        self.mechanics.data.tau_M = self.machine.data.tau_M
        # Post-processing based on the inputs and the states
        super().post_process_with_inputs()
