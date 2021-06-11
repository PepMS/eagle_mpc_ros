############################################################################## 
# BSD 3-Clause License
# Copyright (c) 2021, Institut de Robotica i Informatica Industrial (CSIC-UPC)
# All rights reserved. 
##############################################################################

import numpy as np

import crocoddyl

import eagle_mpc
from eagle_mpc.utils import simulator


class MpcController():
    def __init__(self, trajectoryPath, trajectoryDt, trajectorySolver, mpcPath, mpcType):
        trajectory = eagle_mpc.Trajectory()
        trajectory.autoSetup(trajectoryPath)
        squash = False
        if trajectorySolver == 'SolverSbFDDP':
            squash = True

        self.trajectory_dt = trajectoryDt
        problem = trajectory.createProblem(self.trajectory_dt, squash, "IntegratedActionModelEuler")

        if trajectorySolver == 'SolverSbFDDP':
            solver = eagle_mpc.SolverSbFDDP(problem, trajectory.squash)
        elif trajectorySolver == 'SolverBoxFDDP':
            solver = crocoddyl.SolverBoxFDDP(problem)

        solver.setCallbacks([crocoddyl.CallbackVerbose()])
        solver.solve([], [], 100)

        if mpcType == 'Rail':
            self.mpcController = eagle_mpc.RailMpc(solver.xs, self.trajectory_dt, mpcPath)
        elif mpcType == 'Weighted':
            self.mpcController = eagle_mpc.WeightedMpc(trajectory, self.trajectory_dt, mpcPath)
        else:
            self.mpcController = eagle_mpc.CarrotMpc(trajectory, solver.xs, self.trajectory_dt, mpcPath)
        self.mpcController.updateProblem(0)
        self.mpcController.solver.solve(solver.xs[:self.mpcController.problem.T + 1],
                                        solver.us[:self.mpcController.problem.T])

        self.mpcController.solver.convergence_init = 1e-3

        self.xs = solver.xs
        self.us = solver.us_squash

        self.xss = []
        self.uss = []

        self.nTraj = len(solver.xs)
        self.simDt = 2
        self.simulator = simulator.AerialSimulator(self.mpcController.robot_model, self.mpcController.platform_params,
                                                   self.simDt, solver.xs[0])

    def compute_mpc_trajectory(self):
        time = 0
        while time <= (self.nTraj) * self.trajectory_dt + 1000:
            self.mpcController.problem.x0 = self.simulator.states[-1]
            self.mpcController.updateProblem(time)
            self.mpcController.solver.solve(self.mpcController.solver.xs, self.mpcController.solver.us,
                                            self.mpcController.iters)
            control = np.copy(self.mpcController.solver.us_squash[0])
            self.simulator.simulateStep(control)
            self.xss.append(self.mpcController.solver.xs)
            self.uss.append(self.mpcController.solver.us_squash)
            time += self.simDt
        self.xs = self.simulator.states
        self.us = self.simulator.controls

        self.xss.append(self.xss[-1])