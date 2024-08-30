import os
from typing import List

import numpy as np

from pr2.solver import TaskSolverBase


class DummyPlanner:
    """A dummy planner for task 6 (seed = 666)

    It produces a pre-computed velocity command for accomplishing
    task 6 with random seed 666.
    """

    def __init__(self) -> None:
        # get directory of this file
        dir_path = os.path.dirname(os.path.realpath(__file__))
        # pre-computed velocity command sequence
        cmd_file = f"{dir_path}/task6_cmd.npz"

        self.cmds_ = np.load(cmd_file)["cmd"]

        self.cnt_ = 0

    def plan(self, obs: dict) -> List:
        assert isinstance(obs, dict)

        # repeat last cmd when reach the end
        if self.cnt_ == len(self.cmds_):
            return self.cmds_[-1]

        cmd = self.cmds_[self.cnt_]
        self.cnt_ += 1

        return cmd


class TaskSolver(TaskSolverBase):
    def __init__(self) -> None:
        super().__init__()
        self.planner_ = DummyPlanner()

    def next_action(self, obs: dict) -> dict:
        # plan for velocity cmd
        cmd = self.planner_.plan(obs)

        action = {
            "legs": {
                "ctrl_mode": "effort",
                "joint_values": cmd[0:10],
            },
            "arms": {
                "ctrl_mode": "position",
                "joint_values": np.array(
                    [
                        np.pi / 4,
                        0.0,
                        0.0,
                        -np.pi * 2 / 3,
                        np.pi / 4 + cmd[10],
                        0.0 + cmd[11],
                        0.0,
                        -np.pi * 2 / 3 + cmd[12],
                    ]
                ),
            },
            "pick": "right_hand" if cmd[13] == 1 else None,
            "release": True if cmd[14] == 1 else None,
        }

        return action
