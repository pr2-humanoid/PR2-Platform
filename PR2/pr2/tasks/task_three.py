from typing import Dict, Tuple

import numpy as np

import pr2
from pr2.tasks.base_task import BaseTask
from pr2.utils import Falling, PositionComparator, TimeOut

# pylint: disable=duplicate-code
OBSTACLE_CFG = {"constant_vel_Y": 1.0, "start_y": -5.85, "end_y": -7.5}

TERMINATION_CFG = {
    "distance_tol": 0.2,
    "falling_threshold": 0.26,
    "max_steps": 50000,
    "comparator_offset": 0.4,
    "distance_axis": "y",
    "comparator": ">",
}
START_POS_CFG = {
    "x1": 0.47,
    "x2": 0.47,
    "y1": -7,
    "y2": -6.5,
    "z": 0.61,
    "roll": 0,
    "pitch": 10,
    "yaw": 90,
}  # unit: degree


class TaskThree(BaseTask):
    """
    The goal is to dodge the obstacle.
    """

    def __init__(self, env) -> None:
        super().__init__(env)
        self.obstacle = None

    def initialize(self) -> None:
        """
        Create termination condition,
        Set agent's starting point,
        Set obstacle
        """
        # Set obstacle
        self._set_obstacle("road_42_link")
        self._set_view_port(
            eye=(4.874000, -6.41873808, 1.5881982837),
            target=(0.23457050, -6.42993, 0.176872998),
        )

        # Generate starting point for agent
        self.start_pos, self.start_ori = self.sample_initial_pose(
            START_POS_CFG, yaw_flag=False
        )
        agent = self._env.agent
        agent.set_world_poses(positions=self.start_pos, orientations=self.start_ori)

        # Create termination condition
        self._termination_conditions = {
            "TimeOut": TimeOut(TERMINATION_CFG["max_steps"]),
            "Falling": Falling(TERMINATION_CFG["falling_threshold"]),
            "PositionComparator": PositionComparator(
                TERMINATION_CFG["distance_axis"], TERMINATION_CFG["comparator"]
            ),
        }

        self._load = True

    def _check_termination(self) -> Tuple[bool, bool, bool]:
        is_timed_out = self._termination_conditions["TimeOut"].check(
            self._current_sim_step
        )
        is_falling_detected = self._termination_conditions["Falling"].check(
            self._env.agent
        )

        is_obstacle_behind = self._termination_conditions["PositionComparator"].check(
            self._env.agent.get_world_poses(clone=False)[0].cpu().numpy()[0]
            + np.array(
                [
                    0,
                    -TERMINATION_CFG["comparator_offset"],
                    0,
                ]
            ),
            self.obstacle.get_position_orientation()[0].cpu().numpy(),
        )

        return is_timed_out, is_obstacle_behind, is_falling_detected

    def check(self) -> Tuple[bool, bool]:
        """
        Check the completion status of the task and its success

        Returns:
            2-tuple:
                - bool: True if the task is completed, irrespective of success
                    or failure. False otherwise
                - bool: True if the task has been successfully finished.
                    False otherwise
        """
        (
            is_timed_out,
            is_obstacle_behind,
            is_falling_detected,
        ) = self._check_termination()
        done = is_timed_out or is_obstacle_behind or is_falling_detected
        success = is_obstacle_behind and (not is_falling_detected)

        return done, success

    def reset(self):
        done, success, obs = super().reset()
        self._set_obstacle("road_42_link")
        obs["obstacle"] = self.obstacle.get_obs()
        return done, success, obs

    def step(self, action: Dict) -> Dict:
        """
        Perform task-specific step for every timestep
        1. Apply action
        2. Get observation
        3. Check the task's status

        Returns:
            3-tuple:
                - bool: True if the task is completed, irrespective of success
                    or failure. False otherwise
                - bool: True if the task has been successfully finished.
                    False otherwise
                - dict: Current observation
        """

        self._env.agent.act(action)
        self._step_obstacle()
        if self._current_sim_step % 30 == 0:
            pr2.sim.step()
        else:
            pr2.sim.step(render=False)

        # Get agent info
        obs = self._get_obs()
        # Get obstacle info
        obs["obstacle"] = self.obstacle.get_obs()

        # Check the task status
        done, success = self.check()
        self._current_sim_step += 1
        return done, success, obs

    # Internal Helper
    def _set_obstacle(self, obj_name: str = "road_42_link") -> None:
        self.obstacle = self._env.get_object_by_name(obj_name)
        self.obstacle.set_kinematic(True)

        cur_pos = self.obstacle.get_position_orientation()[0]
        cur_pos[1] = OBSTACLE_CFG["start_y"]
        self.obstacle.set_position_orientation(pos=cur_pos)
        self.obstacle.set_linear_velocity(np.zeros(3))

    def _step_obstacle(self):
        cur_pos = self.obstacle.get_position_orientation()[0]
        next_y_pos = (
            cur_pos[1] - OBSTACLE_CFG["constant_vel_Y"] * pr2.sim.get_physics_dt()
        )
        if next_y_pos > OBSTACLE_CFG["end_y"]:
            cur_pos[1] = next_y_pos
            self.obstacle.set_position_orientation(pos=cur_pos)
        else:
            self.obstacle.set_linear_velocity(np.zeros(3))
