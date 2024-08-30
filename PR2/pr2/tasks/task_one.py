from typing import Dict, Tuple

import pr2
from pr2.tasks.base_task import BaseTask
from pr2.utils import (
    Falling,
    ReachingGoal,
    TimeOut,
)

# pylint: disable=duplicate-code
TERMINATION_CFG = {
    "goal_position": (1.94795, -9.1162, 0.1),
    "distance_tol": 0.4,
    "falling_threshold": 0.26,
    "max_steps": 500000,
}
START_POS_CFG = {
    "x1": 6.91,
    "x2": 7.07,
    "y1": -9.77,
    "y2": -8.61,
    "z": 0.571,
    "roll": 0,
    "pitch": 10,
    "yaw": 180,
}  # unit: degree


class TaskOne(BaseTask):
    """ "
    The goal is to navigate from a random starting point to a specified goal position
    """

    def __init__(self, env) -> None:  # pylint: disable=useless-parent-delegation
        super().__init__(env)

    def initialize(self) -> None:
        """
        Create termination condition and set agent's starting point.
        """
        self._set_view_port(
            eye=[1.0000000000000002, -21.689689999999995, 4.1487], target=[1, 1, 1]
        )

        # Generate starting point for agent
        self.start_pos, self.start_ori = self.sample_initial_pose(START_POS_CFG)
        agent = self._env.agent
        agent.set_world_poses(positions=self.start_pos, orientations=self.start_ori)

        self._termination_conditions = {
            "ReachingGoal": ReachingGoal(
                TERMINATION_CFG["goal_position"], TERMINATION_CFG["distance_tol"]
            ),
            "TimeOut": TimeOut(TERMINATION_CFG["max_steps"]),
            "Falling": Falling(TERMINATION_CFG["falling_threshold"]),
        }
        self._load = True

    def _check_termination(self) -> Tuple[bool, bool, bool]:
        """
        Check if satisfies termination conditions

        Returns:
            3-tuple:
                - bool: True if the agent has reached the specified position.
                    False otherwise.
                - bool: True if the agent has exceeded the maximum allowed time steps.
                    False otherwise.
                - bool: True if the agent has fallen below the threshold.
                    False otherwise.
        """

        is_reached_goal = self._termination_conditions["ReachingGoal"].check(
            self._env.agent
        )
        is_timed_out = self._termination_conditions["TimeOut"].check(
            self._current_sim_step
        )
        is_falling_detected = self._termination_conditions["Falling"].check(
            self._env.agent
        )

        return is_timed_out, is_reached_goal, is_falling_detected

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
        is_timed_out, is_reached_goal, is_falling_detected = self._check_termination()
        done = is_timed_out or is_reached_goal or is_falling_detected
        success = is_reached_goal and (not is_falling_detected)

        return done, success

    def step(self, action: Dict) -> Tuple:
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
