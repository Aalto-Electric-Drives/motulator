from time import time
import matplotlib.pyplot as plt

from simulation import Simulation
# from source.config.ctrl_vector_im_2kW import ctrl, pars, mdl, base
# from source.config.ctrl_vector_pmsm_2kW import ctrl, pars, mdl, base
from config.ctrl_vector_syrm_7kW import ctrl, mdl, base
# from config.ctrl_vhz_im_2kW import ctrl, pars, mdl, base

# %% Main program
if __name__ == '__main__':

    # Start computing the execution time
    start_time = time()

    sim = Simulation(mdl, ctrl, base)
    sim.simulate()
    sim.plot_figure()
    plt.show()

    # sim.save_mat()

    # Print the execution time
    print('\nExecution time: {:.2f} s'.format((time() - start_time)))
