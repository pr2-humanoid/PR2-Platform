import json
import os
from typing import List

import numpy as np

from pr2.solver import TaskSolverBase
from pr2.tcp import PersistentTcpClient, json2bin


class DummyPlanner:
    """A dummy planner for task 1 (seed = 66)

    It produces a pre-computed velocity command for accomplishing
    task 1 with random seed 66.
    """

    def __init__(self) -> None:
        # get directory of this file
        dir_path = os.path.dirname(os.path.realpath(__file__))
        # pre-computed velocity command sequence
        cmd_file = f"{dir_path}/task1_cmd.npz"

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

    def get_cmd(self, obs, vx, vy, theta, state):
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
            "command": [vx, vy, theta],
            "change_state": state,
        }
        joint_efforts = self.send_request(msg)

        return joint_efforts


class TaskSolver(TaskSolverBase):
    def __init__(self) -> None:
        super().__init__()
        self.planner_ = DummyPlanner()
        self.ctrl_client_ = BipedWalkingCtrlClient(ip="0.0.0.0", port=8800)

    def next_action(self, obs: dict) -> dict:
        # plan for velocity cmd
        velocity_cmd = self.planner_.plan(obs)

        # call bipedal controller to get joint effort given a target velocity
        joint_efforts = self.ctrl_client_.get_cmd(
            obs, velocity_cmd[0], velocity_cmd[1], velocity_cmd[2], velocity_cmd[3]
        )

        # wrap joint effort into tongverse-lite action format
        action = {
            "legs": {
                "ctrl_mode": joint_efforts["mode"],
                "joint_values": joint_efforts["effort"],
                "stiffness": None,
                "dampings": None,
            }
        }

        return action
