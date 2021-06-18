:warning: **Disclaimer** :warning:

This is a work-in-progress library. As such, it only contains basic features. For any doubt, bug, problem or suggestion feel free to open an issue. 

# Eagle MPC - ROS
## Introduction
This library contains several ROS-packages to control *unmanned aerial manipulators* (UAMs) using *model predictive control* (MPC) techniques.
It depends on its homonymous library [EagleMPC](https://github.com/PepMS/eagle-mpc) and it has been tested with [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu).
The content of each package is detailed below.

### EagleMPC - controller
Contains the node to run the MPC controllers from the [EagleMPC](https://github.com/PepMS/eagle-mpc) library.

By now, it works in a simulated environment brought by [this modified version for UAMs](https://github.com/PepMS/rotors_simulator) of the [RotorS simulator](https://github.com/ethz-asl/rotors_simulator).

### EagleMPC - msgs
This package contains specific messages used throughout the other packages.

### EagleMPC - rviz_plugins
This package contains rviz plugins that help to visualize the content of the optimal control problem (OCP) solved in the MPC.

**Acknowledgments:** These plugins are modified from [this library](https://github.com/loco-3d/whole_body_state_rviz_plugin).

### EagleMPC - simulator
Simulation related tools such as UAMs spawning or the triggering of external disturbances for the simulation.

### EagleMPC - viz
This package contains visualization tools. It contains several nodes and launch files that allows you to visualize offline generated trajectories as well as rosbags from simulations involving the MPC controllers.

### EagleMPC - yaml
This package contains YAML files that describe multicopter platforms geomteries, trajectories and the tunning for the MPC controllers.

## Installation
### Dependencies

#### EagleMPC
Install this library following the instructions detailed [here](https://github.com/PepMS/eagle-mpc)

#### RotorS simulator
You should install [this forked version](https://github.com/PepMS/rotors_simulator.git) including modifications to be used with UAMs.

### EagleMPC - ROS
Clone this repo inside your workspace and run `catkin_make`
```console
cd <ros_ws>/src
git clone https://github.com/PepMS/eagle_mpc_ros.git
cd ..
catkin_make
```

## Running an example
```
roslaunch eagle_mpc_ros eagle_mpc_controller
```


