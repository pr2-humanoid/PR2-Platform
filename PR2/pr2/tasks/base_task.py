import math
import random
from abc import abstractmethod
from typing import Dict, List, Tuple

import omni.isaac.core.utils.prims as prim_utils
import torch
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.viewports import set_camera_view
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics
from scipy.spatial.transform import Rotation

import pr2
from pr2.objects.object import Object
from pr2.utils import convert_data_format, is_isaac_sim_2023


class BaseTask:
    def __init__(self, env):
        self._done = None
        self._env = env
        self._termination_conditions = None
        self.start_pos, self.start_ori = None, None
        self._load = False
        self._current_sim_step = 0

    @abstractmethod
    def initialize(self) -> None:
        """
        Initialize the task
        """
        raise NotImplementedError()

    def _get_obs(self) -> dict:
        """
        Returns:
            dict: information about the agent's current state
        """
        obs = {}
        obs["current_sim_step"] = self._current_sim_step
        obs["agent"] = self._env.agent.get_obs()
        obs["agent"]["start_pos"] = self.start_pos.cpu().numpy()[0]
        obs["agent"]["start_orient"] = self.start_ori.cpu().numpy()[0]

        return obs

    # pylint: disable=inconsistent-return-statements
    def reset(self) -> Tuple[bool, bool, dict]:
        """
        Reset task-specific internal variables and agent.

        Returns:
            3-tuple:
                - bool: False
                - bool: False
                - dict: the observation at the reset state.
        """
        if not self._load:
            return
        pr2.sim.reset()
        agent = self._env.agent
        # reset agent
        agent.initialize()
        agent.set_world_poses(positions=self.start_pos, orientations=self.start_ori)
        # reset task variable
        self._current_sim_step = 0
        obs = self._get_obs()
        self._current_sim_step += 1
        return False, False, obs

    @abstractmethod
    def check(self) -> Tuple[bool, bool]:
        raise NotImplementedError()

    @abstractmethod
    def step(self, action: Dict) -> Dict:
        raise NotImplementedError()

    def sample_initial_pose(
        self, start_cfg, yaw_flag=True
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        Given task-specific (x1,y1) and (x2,y2,
        generate a random position and quaternion

        Returns:
            2-tuple:
                - torch.Tensor: agent's initial position with respect to the world's
                            frame. shape is (3, )
                - torch.Tensor: agent's initial quaternion orientation with respect
                            to the world's frame. quaternion is scalar-first
                            (w, x, y, z). shape is (4, ).

        """
        random_x = random.uniform(start_cfg["x1"], start_cfg["x2"])
        random_y = random.uniform(start_cfg["y1"], start_cfg["y2"])
        initial_pos = self._env.agent.get_world_poses()[0]
        initial_pos[:, 0] = random_x
        initial_pos[:, 1] = random_y
        initial_pos[:, 2] = start_cfg["z"]

        roll = math.radians(start_cfg["roll"])
        pitch = math.radians(start_cfg["pitch"])
        delta_yaw = random.uniform(-30, 30) if yaw_flag else 0.0
        yaw = math.radians(start_cfg["yaw"] + delta_yaw)

        initial_quat = Rotation.from_euler("xyz", [roll, pitch, yaw]).as_quat()[
            [3, 0, 1, 2]
        ]

        backend = pr2.sim.instance().backend
        device = pr2.sim.instance().device
        backend_utils = pr2.sim.instance().backend_utils

        initial_quat = convert_data_format(backend, device, backend_utils, initial_quat)

        return initial_pos, initial_quat

    def _set_view_port(self, eye: List[float], target: List[float]) -> None:
        set_camera_view(eye=eye, target=target, camera_prim_path="/OmniverseKit_Persp")

        # Add light
        light_attr = {"intensity": 1000.0, "color": (1.0, 1.0, 1.0), "diffuse": 3}
        if is_isaac_sim_2023():
            light_attr = {f"inputs:{k}": v for k, v in light_attr.items()}

        prim_utils.create_prim(
            "/World/Light/DistantLight",
            "DistantLight",
            translation=[0, 0, 0],
            attributes=light_attr,
        )

    def _create_d6_joint(
        self,
        body_name: str,
        obj: Object,
        trans_x: bool = True,
        trans_y: bool = True,
        trans_z: bool = True,
        rot_x: bool = True,
        rot_y: bool = True,
        rot_z: bool = True,
    ) -> None:
        joint_path = "/World/tmp"

        d6joint = UsdPhysics.Joint.Define(pr2.sim.stage, joint_path)

        ee_link_path = f"/World/Kuavo/{body_name}"
        object_path = obj.prim_path

        d6joint.CreateBody0Rel().SetTargets([Sdf.Path(ee_link_path)])
        d6joint.CreateBody1Rel().SetTargets([Sdf.Path(object_path)])

        xf_cache = UsdGeom.XformCache()
        child_pose = xf_cache.GetLocalToWorldTransform(obj.prim)
        ee_prim_link = get_prim_at_path(ee_link_path)
        parent_pose = xf_cache.GetLocalToWorldTransform(ee_prim_link)

        rel_pose = parent_pose * child_pose.GetInverse()
        rel_pose = rel_pose.RemoveScaleShear()
        pos2 = Gf.Vec3f(rel_pose.ExtractTranslation())
        rot2 = Gf.Quatf(rel_pose.ExtractRotationQuat())

        pr2.sim.render()

        d6joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        d6joint.CreateLocalRot0Attr().Set(Gf.Quatf(1.0))
        d6joint.CreateLocalPos1Attr().Set(pos2)
        d6joint.CreateLocalRot1Attr().Set(rot2)

        # lock all DOF (lock - low is greater than high)
        d6prim = d6joint.GetPrim()
        if trans_x:
            limitapi = UsdPhysics.LimitAPI.Apply(d6prim, UsdPhysics.Tokens.transX)
            limitapi.CreateLowAttr(1.0)
            limitapi.CreateHighAttr(-1.0)
        if trans_y:
            limitapi = UsdPhysics.LimitAPI.Apply(d6prim, UsdPhysics.Tokens.transY)
            limitapi.CreateLowAttr(1.0)
            limitapi.CreateHighAttr(-1.0)
        if trans_z:
            limitapi = UsdPhysics.LimitAPI.Apply(d6prim, UsdPhysics.Tokens.transZ)
            limitapi.CreateLowAttr(1.0)
            limitapi.CreateHighAttr(-1.0)
        if rot_x:
            limitapi = UsdPhysics.LimitAPI.Apply(d6prim, UsdPhysics.Tokens.rotX)
            limitapi.CreateLowAttr(1.0)
            limitapi.CreateHighAttr(-1.0)
        if rot_y:
            limitapi = UsdPhysics.LimitAPI.Apply(d6prim, UsdPhysics.Tokens.rotY)
            limitapi.CreateLowAttr(1.0)
            limitapi.CreateHighAttr(-1.0)
        if rot_z:
            limitapi = UsdPhysics.LimitAPI.Apply(d6prim, UsdPhysics.Tokens.rotZ)
            limitapi.CreateLowAttr(1.0)
            limitapi.CreateHighAttr(-1.0)

    def _create_fixed_joint(self, body_name: str, obj: Object) -> None:
        joint_path = "/World/tmp"
        fixed_joint = UsdPhysics.FixedJoint.Define(pr2.sim.stage, joint_path)
        # Get the prim pointed to at this path
        joint_prim = get_prim_at_path(joint_path)

        # Apply joint API interface
        PhysxSchema.PhysxJointAPI.Apply(joint_prim)

        ee_link_path = f"/World/Kuavo/{body_name}"
        ee_prim_link = get_prim_at_path(ee_link_path)
        object_path = obj.prim_path
        fixed_joint.CreateBody0Rel().SetTargets([Sdf.Path(ee_link_path)])
        fixed_joint.CreateBody1Rel().SetTargets([Sdf.Path(object_path)])

        xf_cache = UsdGeom.XformCache()
        child_pose = xf_cache.GetLocalToWorldTransform(obj.prim)
        parent_pose = xf_cache.GetLocalToWorldTransform(ee_prim_link)

        rel_pose = child_pose * parent_pose.GetInverse()
        rel_pose = rel_pose.RemoveScaleShear()
        pos1 = Gf.Vec3f(rel_pose.ExtractTranslation())
        rot1 = Gf.Quatf(rel_pose.ExtractRotationQuat())

        fixed_joint.CreateLocalPos0Attr().Set(pos1)
        fixed_joint.CreateLocalRot0Attr().Set(rot1)
        fixed_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0))
        fixed_joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0))
