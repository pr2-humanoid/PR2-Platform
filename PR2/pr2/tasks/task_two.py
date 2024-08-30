from typing import Dict, Tuple

import numpy as np
from omni.isaac.core.utils.prims import delete_prim

import pr2
from pr2.objects.object import Object
from pr2.tasks.base_task import BaseTask
from pr2.utils import (
    Falling,
    Rotation,
    TimeOut,
)

# pylint: disable=duplicate-code
TERMINATION_CFG = {
    "falling_threshold": 0.26,
    "max_steps": 120000,
    "radian_threshold": 0.785,
}
PICK_CFG = {
    "radius_threshold": 0.30,
    "z_upper_limit": 0.89,
    "z_lower_limit": 0.78,
}
EE_LINKS = ["r_arm_shpere", "l_arm_sphere"]

START_POS_CFG = {
    "x1": 1.095,
    "x2": 1.255,
    "y1": -9.73,
    "y2": -8.57,
    "z": 0.60635,
    "roll": 0,
    "pitch": 10,
    "yaw": 180,
}  # unit: degree


class TaskTwo(BaseTask):
    """ "
    The goal is to rotate the valve
    """

    def __init__(self, env) -> None:
        super().__init__(env)
        self._valve = None
        self._pick = False

    def initialize(self) -> None:
        """
        Create termination condition and set agent's starting point.
        """
        self._set_view_port(
            eye=(0.14875291020841971, -11.72209149833846, 2.0102491219982292),
            target=(0.0936800017952919, -9.393560409545898, 0.618149995803833),
        )
        self._valve = self._env.get_object_by_name("valve_47_link")

        # Generate starting point for agent
        self.start_pos, self.start_ori = self.sample_initial_pose(START_POS_CFG)
        agent = self._env.agent
        agent.set_world_poses(positions=self.start_pos, orientations=self.start_ori)

        self._termination_conditions = {
            "Rotation": Rotation(self._valve, TERMINATION_CFG["radian_threshold"]),
            "TimeOut": TimeOut(TERMINATION_CFG["max_steps"]),
            "Falling": Falling(TERMINATION_CFG["falling_threshold"]),
        }
        self._load = True

    def _check_termination(self) -> Tuple[bool, bool, bool]:
        is_rotated = self._termination_conditions["Rotation"].check()
        is_timed_out = self._termination_conditions["TimeOut"].check(
            self._current_sim_step
        )
        is_falling_detected = self._termination_conditions["Falling"].check(
            self._env.agent
        )

        return is_timed_out, is_rotated, is_falling_detected

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
        is_timed_out, is_rotated, is_falling_detected = self._check_termination()
        done = is_timed_out or is_rotated or is_falling_detected
        success = is_rotated and (not is_falling_detected)

        return done, success

    def reset(self) -> Tuple[bool, bool, dict]:
        if self._pick:
            self.release()
        done, success, obs = super().reset()
        self._valve.initialize()
        obs["pick"] = self._pick
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
        if action.get("pick", None) is not None and not self._pick:
            if action.get("pick") == "left_hand":
                self.pick("l_arm_sphere", self._valve)
            if action.get("pick") == "right_hand":
                self.pick("r_arm_sphere", self._valve)

        if action.get("release", False) is True and self._pick:
            self.release()

        if self._current_sim_step % 30 == 0:
            pr2.sim.step()
        else:
            pr2.sim.step(render=False)
        # Get agent info
        obs = self._get_obs()
        obs["pick"] = self._pick
        # Check the task status
        done, success = self.check()
        self._current_sim_step += 1
        return done, success, obs

    def pick(self, body_name: str, obj: Object) -> None:
        # Only pick when distance within the threshold
        if self._check_pick(body_name=body_name, obj=obj):
            self._create_d6_joint(
                body_name=body_name,
                obj=obj,
                trans_x=True,
                trans_y=True,
                trans_z=True,
                rot_x=False,
                rot_y=False,
                rot_z=False,
            )
            self._pick = True

    def release(self) -> None:
        delete_prim("/World/tmp")
        self._pick = False

    # Internal helper
    def _check_pick(self, body_name: str, obj: Object) -> bool:
        agent = self._env.agent
        hand_pos = agent.bodies[body_name].get_world_poses()[0].numpy()[0]
        hand_pos_z = hand_pos[2]

        if (
            hand_pos_z > PICK_CFG["z_upper_limit"]
            or hand_pos_z < PICK_CFG["z_lower_limit"]
        ):
            return False

        object_pos = obj.get_position_orientation()[0].numpy()
        # Pick when L2 distance below threshold
        dist = np.linalg.norm(object_pos[0:2] - hand_pos[0:2])

        return dist <= PICK_CFG["radius_threshold"]
