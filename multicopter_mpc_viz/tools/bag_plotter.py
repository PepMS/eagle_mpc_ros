import numpy as np
import matplotlib.pyplot as plt

from multicopter_mpc.utils.plots import PlotControls, PlotStates

import multicopter_mpc_bag_utils

BAG_PATH = '/home/pepms/wsros/mpc-ws/src/multicopter_mpc/multicopter_mpc_bag/rosbags'

bag_list = ['passthrough_euler_dt10_n100', 'passthrough_euler_dt10_n100_sb']
legend = ['Euler, dt=10, N=100', 'Euler, dt=10, N=100, sb']

MISSION_NAME = "passthrough"

YAML_PATH = "/home/pepms/wsros/mpc-ws/src/multicopter_mpc/multicopter_mpc_yaml"
MISSION_PATH = YAML_PATH + "/missions/passthrough.yaml"
MPC_MAIN_PATH = YAML_PATH + "/mpc_main/mpc-main.yaml"

mc_bag = []
for bag in bag_list:
    mc = multicopter_mpc_bag_utils.MulticopterBag(BAG_PATH + '/' + bag + '.bag', MISSION_PATH)
    mc_bag.append(mc)

# Solving time
plt.figure()
for mc in mc_bag:
    plt.plot(mc.time, [solve_iter.solving_time for solve_iter in mc.solver_iters])
plt.legend(legend)
plt.title('Solving time')
plt.grid(linestyle='--', linewidth=0.5)

plt.figure()
for mc in mc_bag:
    plt.plot(mc.time, [solve_iter.iters for solve_iter in mc.solver_iters])
plt.legend(legend)
plt.title('Iterations')
plt.grid(linestyle='--', linewidth=0.5)

plot_states, plot_controls, plot_time = [], [], []
for mc in mc_bag:
    plot_states.append(np.stack(mc.states, axis=0).T)
    plot_controls.append(np.stack(mc.controls, axis=0).T)
    plot_time.append(np.array(mc.time))

PlotControls(plot_controls, plot_time, legend=legend)
PlotStates(plot_states, plot_time, wp_list=mc_bag[0].mission.waypoints, legend=legend)

plt.show()
