import random
from typing import Dict, Tuple

import numpy as np
from omni.isaac.core.utils.prims import delete_prim

import pr2
from pr2.objects.object import Object
from pr2.sensor import Sensor
from pr2.tasks.base_task import BaseTask
from pr2.utils import Falling, PositionComparator, TimeOut

# pylint: disable=duplicate-code


TERMINATION_CFG = {
    "container": "coffeetable_9_link",
    "falling_threshold": 0.78506,
    "max_steps": 300000,
}

PICK_CFG = {
    "goal_objects": {
        "teddy_bear": {
            "linkname": "toy_45_link",
            "start_pos_cfg": {
                "x1": -4.3,
                "x2": -4.1,
                "y1": 4.5,
                "y2": 4.61,
                "z": 1.36008,
                "roll": 0,
                "pitch": 0,
                "yaw": 0,
            },
        },
        "coffee_mug": {
            "linkname": "coffeemug_8_link",
            "start_pos_cfg": {
                "x1": -2.45,
                "x2": -2.3,
                "y1": 1.1,
                "y2": 1.7,
                "z": 1.36008,
                "roll": 0,
                "pitch": 0,
                "yaw": 0,
            },
        },
        "red_cube": {
            "linkname": "building_block",
            "start_pos_cfg": {
                "x1": 4.45,
                "x2": 4.6,
                "y1": 9.3,
                "y2": 9.6,
                "z": 1.41961,
                "roll": 0,
                "pitch": 0,
                "yaw": 0,
            },
        },
    },
    "radius_threshold": 0.2,
}

START_POS_CFG = {
    "x1": -0.2,
    "x2": 0.2,
    "y1": 0.3,
    "y2": 0.7,
    "z": 1.0984,
    "roll": 0,
    "pitch": 10,
    "yaw": 180,
}

CAMERA_CFG = [
    {
        "name": "cam",
        "cam_params": {
            "focal_length": 22.4,
            "f_stop": 0,
            "position": (-0.41, 0, 1.2),  # XYZ coordinates in world space.
            "rotation": (180, -135, 0),  # Euler angles in degrees in XYZ order.
            "focus_distance": 20,
            "horizontal_aperture": 20.955,
            "parent": "/World/Kuavo/torso",
        },
        "resolution": (1080, 720),
    }
]


class TaskSix(BaseTask):
    """ "
    The goal is to pick an object and place it on the table
    """

    def __init__(self, env) -> None:
        super().__init__(env)
        # Create Task Specific Camera
        self._cams = []
        self._target_object = None
        self._target_object_name = None
        self._pick = False
        for cam_cfg in CAMERA_CFG:
            self._cams.append(Sensor(**cam_cfg))

    def initialize(self) -> None:
        """
        Create termination condition and set agent's starting point.
        """
        self._set_view_port(
            eye=[0.4596420311975781, 4.87, 19.71280218252985],
            target=(-0.31226855516433716, 4.877008438110352, 0.9879316687583923),
        )
        target_objects = list(PICK_CFG["goal_objects"].keys())
        self._target_object_name = target_objects[random.randint(0, 2)]
        self._target_object = self._env.get_object_by_name(
            PICK_CFG["goal_objects"][self._target_object_name]["linkname"]
        )

        self._termination_conditions = {
            "PlaceGoal": PositionComparator(),
            "TimeOut": TimeOut(TERMINATION_CFG["max_steps"]),
            "Falling": Falling(TERMINATION_CFG["falling_threshold"]),
        }

        # sample agent initial pose
        self.start_pos, self.start_ori = self.sample_initial_pose(START_POS_CFG)
        agent = self._env.agent
        agent.set_world_poses(positions=self.start_pos, orientations=self.start_ori)

        # sample target object candidates initial pose
        for _, obj_cfg in PICK_CFG["goal_objects"].items():
            start_pos, start_ori = self.sample_initial_pose(obj_cfg["start_pos_cfg"])
            start_pos, start_ori = start_pos[0], start_ori[0]
            obj = self._env.get_object_by_name(obj_cfg["linkname"])
            obj.set_position_orientation(pos=start_pos, orient=start_ori)

        self._load = True

    def _check_termination(self) -> Tuple[bool, bool, bool]:
        if not self._pick:
            is_inside = self._termination_conditions["PlaceGoal"].check_inside(
                self._target_object.get_position_orientation()[0].cpu().numpy(),
                f'/World/Scene/{TERMINATION_CFG["container"]}',
            )
        else:
            is_inside = False

        is_timed_out = self._termination_conditions["TimeOut"].check(
            self._current_sim_step
        )
        is_falling_detected = self._termination_conditions["Falling"].check(
            self._env.agent
        )

        return is_timed_out, is_inside, is_falling_detected

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
        is_timed_out, is_inside, is_falling_detected = self._check_termination()
        done = is_timed_out or is_inside or is_falling_detected
        success = is_inside and (not is_falling_detected)

        return done, success

    def reset(self):
        if self._pick:
            self.release()
        done, success, obs = super().reset()
        obs[
            "goal"
        ] = f'Please put the {self._target_object_name} on the {TERMINATION_CFG["container"]}'  # noqa: E501  # pylint: disable=C0301
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
                self.pick("l_arm_sphere", self._target_object)
            if action.get("pick") == "right_hand":
                self.pick("r_arm_sphere", self._target_object)

        if action.get("release", False) is True and self._pick:
            self.release()

        if self._current_sim_step % 30 == 0:
            pr2.sim.step()
            obs = self._get_obs()
            for cam in self._cams:
                obs[cam.get_name()] = cam.get_obs()
        else:
            pr2.sim.step(render=False)
            obs = self._get_obs()

        obs[
            "goal"
        ] = f'Please put {self._target_object_name} on the {TERMINATION_CFG["container"]}'
        obs["pick"] = self._pick
        # Check the task status
        done, success = self.check()
        self._current_sim_step += 1
        return done, success, obs

    def pick(self, body_name: str, obj: Object) -> None:
        # Only pick when distance within the threshold
        if self._check_pick(body_name=body_name, obj=obj):
            self._create_fixed_joint(body_name=body_name, obj=obj)
            self._pick = True

    def release(self) -> None:
        delete_prim("/World/tmp")
        self._pick = False

    # Internal helper
    def _check_pick(self, body_name: str, obj: Object) -> bool:
        agent = self._env.agent
        hand_pos = agent.bodies[body_name].get_world_poses()[0].numpy()[0]
        object_pos = obj.get_position_orientation()[0].numpy()
        # Pick when L2 distance below threshold
        dist = np.linalg.norm(object_pos - hand_pos)
        return dist <= PICK_CFG["radius_threshold"]
