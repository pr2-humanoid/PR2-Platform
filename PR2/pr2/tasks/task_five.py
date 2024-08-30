from typing import Dict, Tuple

import pr2
from pr2.tasks.base_task import BaseTask
from pr2.utils import (
    Contact,
    Falling,
    TimeOut,
)

# pylint: disable=duplicate-code
TERMINATION_CFG = {
    "goal_position": (0.23457, -1.44135, 0.593),
    "distance_tol": 0.2,
    "falling_threshold": 0.41,
    "max_steps": 180000,
    "button_contact_sensor": {
        "object_name": "button_4_link",
        "max_step": 200,
        "sensor_radius": -1,
        "sensor_offset": (0, 0, 0),
    },
}
START_POS_CFG = {
    "x1": -0.218,
    "x2": 0.653,
    "y1": -2.633,
    "y2": -2.625,
    "z": 0.754656,
    "roll": 0,
    "pitch": 10,
    "yaw": 90,
}  # unit: degree


class TaskFive(BaseTask):
    """ "
    The goal is to press the button
    """

    def __init__(self, env) -> None:  # pylint: disable=useless-parent-delegation
        super().__init__(env)

    def initialize(self) -> None:
        """
        Create termination condition and set agent's starting point.
        """
        self._set_view_port(
            eye=(5.396751424747548, -1.456024842010389, 1.5402921578501494),
            target=(0.15, -1.6, 1.6),
        )

        # Generate starting point for agent
        self.start_pos, self.start_ori = self.sample_initial_pose(
            START_POS_CFG, yaw_flag=False
        )
        agent = self._env.agent
        agent.set_world_poses(positions=self.start_pos, orientations=self.start_ori)
        self._termination_conditions = {
            "ContactGoal": Contact(**TERMINATION_CFG["button_contact_sensor"]),
            "TimeOut": TimeOut(TERMINATION_CFG["max_steps"]),
            "Falling": Falling(TERMINATION_CFG["falling_threshold"]),
        }
        self._load = True

    def _check_termination(self) -> Tuple[bool, bool, bool]:
        is_contacted = self._termination_conditions["ContactGoal"].check()
        is_timed_out = self._termination_conditions["TimeOut"].check(
            self._current_sim_step
        )
        is_falling_detected = self._termination_conditions["Falling"].check(
            self._env.agent
        )

        return is_timed_out, is_contacted, is_falling_detected

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
        is_timed_out, is_contacted, is_falling_detected = self._check_termination()
        done = is_timed_out or is_contacted or is_falling_detected
        success = is_contacted and (not is_falling_detected)

        return done, success

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
        if self._current_sim_step % 30 == 0:
            pr2.sim.step()
        else:
            pr2.sim.step(render=False)

        # Get agent info
        obs = self._get_obs()

        # Check the task status
        done, success = self.check()
        self._current_sim_step += 1
        return done, success, obs
