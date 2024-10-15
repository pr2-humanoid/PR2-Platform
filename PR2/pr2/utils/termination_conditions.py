from typing import Tuple

import numpy as np
import omni
import torch
from omni.isaac.core.articulations.articulation import Articulation
from omni.isaac.core.utils.bounds import compute_aabb, create_bbox_cache
from omni.isaac.sensor import _sensor
from scipy.spatial.transform import Rotation as R
from pxr import PhysxSchema
from omni.isaac.core.utils.stage import get_current_stage
class ReachingGoal:
    """
    Task terminates when the agent reaches the
    goal position within the specified distance tolerance

    Args:
        distance_tol (float): acceptable range between the
        goal position and the agent baselink position

    """

    def __init__(
        self, goal_position: Tuple[float, float, float], distance_tol: float
    ) -> None:
        self._distance_tol = distance_tol
        self._goal_position = goal_position

    def check(self, agent) -> bool:
        # Terminate when the point goal is reached (L2 distance below threshold)
        curpos = agent.get_world_poses()[0]
        if isinstance(curpos, torch.Tensor):
            curpos = curpos.numpy()[0]
        dist = np.linalg.norm(curpos[0:2] - np.array(self._goal_position)[0:2])
        return dist < self._distance_tol


class TimeOut:
    """
    Task terminates when the maximum allowed number of steps has been reached

    Args:
        max_steps (int): Maximum allowable number of steps for this task.
    """

    def __init__(self, max_steps: int) -> None:
        self._max_steps = max_steps

    def check(self, step: int) -> bool:
        # Terminate when current number of steps exceeds the threshold
        return step >= self._max_steps


class Falling:
    """
    Task terminates when agent's z-axis falls below a specified threshold.

    Args:
        threshold (float): the specified threshold
    """

    def __init__(self, threshold: float) -> None:
        self._distance_tol = threshold

    def check(self, agent) -> bool:
        # Terminate when agent falls below threshold
        curpos = agent.get_world_poses()[0]
        if isinstance(curpos, torch.Tensor):
            curpos = curpos.numpy()[0]

        return curpos[-1] < self._distance_tol


class Contact:
    """
    Task terminates when the target object has been contacted by other entities
    within the specified distance for a specified duration.

    Args:
         object_name (str): object's baselink
         max_steps (int): specified duration.
         sensor_radius (float): specified distance
         sensor_offset (Tuple[float]): sensor offset from the center of the object

    """

    def __init__(
        self,
        object_name: str,
        max_step: int,
        sensor_radius: float = 1.0,
        sensor_offset: Tuple[float, float, float] = (0, 0, 0.5),
    ):
        self._cnt = 0
        self._max_step = max_step
        self._cs = _sensor.acquire_contact_sensor_interface()
        self._object_name = object_name
        # Add contact sensor to the object
        stage = get_current_stage()
        contactprim = stage.GetPrimAtPath(f"/World/Scene/{object_name}")
        PhysxSchema.PhysxContactReportAPI.Apply(contactprim)

        omni.kit.commands.execute(
            "IsaacSensorCreateContactSensor",
            path="/contact_sensor",
            parent=f"/World/Scene/{object_name}",
            min_threshold=0,
            max_threshold=10000000,
            color=(1, 0, 0, 1),
            radius=sensor_radius,
            sensor_period=-1,
            translation=sensor_offset 
        )

    def check(self) -> bool:
        # Terminate when the object has been contacted
        # for more than the specified duration
        result = self._cs.get_sensor_reading(
            f"/World/Scene/{self._object_name}/contact_sensor"
        ).inContact

        self._cnt += 1 if result else 0
        return self._cnt >= self._max_step


class PositionComparator:
    def __init__(self, distance_axes=None, comparator=None) -> None:
        if distance_axes is not None:
            self.axes_idx = [i for i, axis in enumerate("xyz") if axis in distance_axes]
        if comparator is not None:
            self.comparators = comparator
        self._cnt = 0
        self._max_step = 500

    def check_inside(self, object_pos, container_prim_path) -> bool:
        cache = create_bbox_cache()
        # pylint: disable=unused-variable
        min_x, min_y, min_z, max_x, max_y, max_z = compute_aabb(
            cache, prim_path=container_prim_path
        )
        result = min_x < object_pos[0] < max_x and min_y < object_pos[1] < max_y
        self._cnt += 1 if result else 0
        return self._cnt >= self._max_step

    def check(self, objecta_pos, objectb_pos) -> bool:
        flags = 0
        idx = 0
        comparison = zip(self.axes_idx, self.comparators)
        for axis, comparator in comparison:
            pos_a = objecta_pos[axis]
            pos_b = objectb_pos[axis]
            if comparator == "<":
                flags += pos_a < pos_b
            if comparator == ">":
                flags += pos_a > pos_b
            if comparator == "==":
                flags += pos_a == pos_b
            idx += 1
        return flags == idx and flags > 0


class Rotation:
    """
    Task terminates when the target object has been rotated beyond the
    specified degree threshold

    Args:
         obj (Object): target object
         radian (float): the specified radian threshold


    """

    def __init__(self, obj: Articulation, radian: float) -> None:
        self._obj = obj
        self._threshold = radian
        q1 = obj.get_position_orientation()[1] 
        self._r1 = R.from_quat(q1)

    def check(self) -> bool:
        q2 = self._obj.get_position_orientation()[1]
        r2 = R.from_quat(q2)
        r_relative = r2 * self._r1.inv()
        angle_degrees = r_relative.magnitude() 
        return abs(angle_degrees) >= self._threshold
