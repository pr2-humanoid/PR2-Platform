import json
import os
from typing import List

import numpy as np

from pr2.solver import TaskSolverBase
from pr2.tcp import PersistentTcpClient, json2bin


class DummyPlanner:
    """A dummy planner for task 2 (seed = 266)

    It produces a pre-computed command for accomplishing
    task 2 with random seed 266.
    """

    def __init__(self, flag) -> None:
        # get directory of this file
        dir_path = os.path.dirname(os.path.realpath(__file__))
        # pre-computed velocity command sequence
        if flag:
            cmd_file = f"{dir_path}/task2_cmd.npz"
        else:
            cmd_file = f"{dir_path}/task2_cmd2.npz"

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


class BipedWalkingCtrlClient(PersistentTcpClient):
    def send_request(self, msg):
        data_bin = json2bin(msg)
        return json.loads(self.send(data_bin).decode("ascii"))

    def get_cmd(self, obs, v_x, v_y, theta, state):
        obs_agent = obs["agent"]
        q_leg = obs_agent["joint_state"]["legs_positions"]
        dq_leg = obs_agent["joint_state"]["legs_velocities"]

        q_arm = obs_agent["joint_state"]["arms_positions"]
        dq_arm = obs_agent["joint_state"]["arms_velocities"]

        p_wb = obs_agent["body_state"]["world_pos"]
        quat_wb = obs_agent["body_state"]["world_orient"]
        v_wb = obs_agent["body_state"]["linear_velocities"]
        w_wb = obs_agent["body_state"]["angular_velocities"]

        msg = {
            "q_leg": q_leg.tolist(),
            "dq_leg": dq_leg.tolist(),
            "q_arm": q_arm.tolist(),
            "dq_arm": dq_arm.tolist(),
            "p_wb": p_wb.tolist(),
            "quat_wb": quat_wb.tolist(),
            "v_wb": v_wb.tolist(),
            "w_wb": w_wb.tolist(),
            "command": [v_x, v_y, theta],
            "change_state": state,
        }
        joint_efforts = self.send_request(msg)

        return joint_efforts


class TaskSolver(TaskSolverBase):
    def __init__(self) -> None:
        super().__init__()
        self.flag = True
        self.planner_ = DummyPlanner(flag=self.flag)
        self.ctrl_client_ = BipedWalkingCtrlClient(ip="0.0.0.0", port=8800)

    def next_action(self, obs: dict) -> dict:
        # plan for velocity cmd
        velocity_cmd = self.planner_.plan(obs)

        # call bipedal controller to get joint effort given a target velocity
        joint_efforts = self.ctrl_client_.get_cmd(
            obs, velocity_cmd[0], velocity_cmd[1], velocity_cmd[2], velocity_cmd[3]
        )

        # wrap joint effort into pr2 action format
        if self.flag:
            action = {
                "legs": {
                    "ctrl_mode": joint_efforts["mode"],
                    "joint_values": joint_efforts["effort"],
                    "stiffness": None,
                    "dampings": None,
                },
                "arms": {
                    "ctrl_mode": "position",
                    "joint_values": np.array(
                        [
                            np.pi / 4 + velocity_cmd[4],
                            0.0 + velocity_cmd[5],
                            0.0,
                            -np.pi * 2 / 3 + velocity_cmd[6],
                            np.pi / 4,
                            0.0,
                            0.0,
                            -np.pi * 2 / 3,
                        ]
                    ),
                },
                "pick": "left_hand" if velocity_cmd[7] == 1 else None,
            }
            if obs["pick"] is True:
                action["arms"]["stiffness"] = np.array([10] * 4 + [20] * 4)
                action["arms"]["dampings"] = np.array([1] * 4 + [2] * 4)
        else:
            action = {
                "legs": {
                    "ctrl_mode": "effort",
                    "joint_values": velocity_cmd[0:10],
                    "stiffness": None,
                    "dampings": None,
                },
                "arms": {
                    "ctrl_mode": "position",
                    "joint_values": np.array(
                        [
                            np.pi / 4 + velocity_cmd[10],
                            0.0 + velocity_cmd[11],
                            0.0,
                            -np.pi * 2 / 3 + velocity_cmd[12],
                            np.pi / 4,
                            0.0,
                            0.0,
                            -np.pi * 2 / 3,
                        ]
                    ),
                },
                "pick": "left_hand" if velocity_cmd[13] == 1 else None,
                "release": True if velocity_cmd[14] == 1 else None,
            }
            if velocity_cmd[13] == 1:
                action["arms"]["stiffness"] = np.array([10] * 8)
                action["arms"]["dampings"] = np.array([1] * 8)
                action["arms"]["joint_values"][1] += np.pi / 8

        return action