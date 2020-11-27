import matplotlib.pyplot as plt

import multicopter_mpc_bag_utils

BAG_PATH = '/home/pepms/wsros/mpc-ws/src/multicopter_mpc/multicopter_mpc_bag/rosbags'
FILE_NAME = 'passthrough.yaml_2020-11-27-13-08-44'

# ROBOT_NAME = "iris"
# MISSION_NAME = "passthrough"
# TRAJECTORY_GENERATOR_YAML = "/home/pepms/wsros/mpc-ws/src/multicopter_mpc_ros/yaml/trajectory-generator.yaml"

YAML_PATH = "/home/pepms/wsros/mpc-ws/src/multicopter_mpc/multicopter_mpc_yaml"
MISSION_PATH = YAML_PATH + "/missions/passthrough.yaml"
MPC_MAIN_PATH = YAML_PATH + "/mpc_main/mpc-main.yaml"

mc_bag = multicopter_mpc_bag_utils.MulticopterBag(BAG_PATH + '/' + FILE_NAME + '.bag')

mc_bag.fillSolverPerformanceIndicators()
mc_bag.fillBagWholeBodyStateTrajectory()
mc_bag.problemReconstruction(MISSION_PATH, MPC_MAIN_PATH)

