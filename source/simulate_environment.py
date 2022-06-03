# pylint: disable=C0103
'''
(c) Aalto Electric Drives (contact: marko.hinkkanen@aalto.fi)

motulator: Open-Source Simulator for Motor Drives and Power Converters

'''
# %% Imports
from time import time
import matplotlib.pyplot as plt
from simulation import SimEnv#, Simulation

try:
    from tqdm import tqdm, trange  # Can play around with tqdm.gui aswell, prints progress bars using matplotlib
except ImportError:
    print('tqdm could not be imported, because the tqdm library is missing')

# %%
# Import control and model settings from the config files
import config.ctrl_vector_im_2kW as vec_im_2kW
import config.ctrl_vector_pmsm_2kW as vec_pmsm_2kW
import config.ctrl_vector_syrm_7kW as vec_syrm_7kW
import config.ctrl_vhz_im_2kW as vhz_im_2kW


# %% Main program
def main():
    start_time = time()

    # Syntax for simulating just one model
    # sim = Simulation(vec_im_2kW.ctrl, vec_im_2kW.mdl, vec_im_2kW.base, vec_im_2kW.pars, 'ctrl_vector_im_2kW')
    # sim.simulate()
    # sim.plot_figure()
    # plt.show()

    # Syntax for simulating several models simultaneously
    # Initialize a simulation environment
    Simulator = SimEnv()

    # Add some simulation instances
    Simulator.add_sim(vec_im_2kW.ctrl, vec_im_2kW.mdl, vec_im_2kW.base, vec_im_2kW.pars, 'ctrl_vector_im_2kW')
    Simulator.add_sim(vec_pmsm_2kW.ctrl, vec_pmsm_2kW.mdl, vec_pmsm_2kW.base, vec_pmsm_2kW.pars, 'ctrl_vector_pmsm_2kW')
    Simulator.add_sim(vec_syrm_7kW.ctrl, vec_syrm_7kW.mdl, vec_syrm_7kW.base, vec_syrm_7kW.pars, 'ctrl_vector_syrm_7kW')
    Simulator.add_sim(vhz_im_2kW.ctrl, vhz_im_2kW.mdl, vhz_im_2kW.base, vhz_im_2kW.pars, 'ctrl_vhz_im_2kW')

    # Prints out the configs for each simulation
    # Simulator.print_simulation_configs()

    # To disable the progress bars (for example if they don't work properly on your IDE, set "disable=True" in the
    # keyword arguments on line 55
    Simulator.simulate_all()

    print('Execution time: {:.2f} s'.format(time() - start_time))

    Simulator.plot_figures()

    # Display all plotted figures
    plt.show()


# %%
if __name__ == '__main__':
    main()
