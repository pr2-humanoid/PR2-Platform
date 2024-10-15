from typing import Optional, Sequence, Tuple, Union

import numpy as np
import torch
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.semantics import add_update_semantics
from pxr import Gf, PhysxSchema, UsdPhysics


class Object:
    def __init__(self, name: str) -> None:
        self._name = name
        self._prim_path = f"/World/Scene/{self._name}"
        self._prim = get_prim_at_path(self._prim_path)
        self._xprim = XFormPrim(self._prim_path)
        self._rigid_api = None
        self._physx_rigid_api = None
        self._mass_api = None

    def initialize(self) -> None:
        # Apply rigid body and mass APIs
        self._rigid_api = (
            UsdPhysics.RigidBodyAPI(self._prim)
            if self._prim.HasAPI(UsdPhysics.RigidBodyAPI)
            else UsdPhysics.RigidBodyAPI.Apply(self._prim)
        )
        self._physx_rigid_api = (
            PhysxSchema.PhysxRigidBodyAPI(self._prim)
            if self._prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI)
            else PhysxSchema.PhysxRigidBodyAPI.Apply(self._prim)
        )
        self._mass_api = (
            UsdPhysics.MassAPI(self._prim)
            if self._prim.HasAPI(UsdPhysics.MassAPI)
            else UsdPhysics.MassAPI.Apply(self._prim)
        )
        # Add semantics
        add_update_semantics(
            prim=self._prim,
            semantic_label=self._name,
            type_label="class",
        )

    def set_position_orientation(
        self,
        pos: Optional[Sequence[float]] = None,
        orient: Optional[Sequence[float]] = None,
    ):
        """
        Sets prim’s pose with respect to the world’s frame.

        Args:
            position (Optional[Sequence[float]], optional) –
            position in the world frame of the prim. shape is (3, ).
            Defaults to None, which means left unchanged.

            orientation (Optional[Sequence[float]], optional) –
            quaternion orientation in the world frame of the prim.
            quaternion is scalar-first (w, x, y, z). shape is (4, ).
            Defaults to None, which means left unchanged.
        """
        return self._xprim.set_world_pose(position=pos, orientation=orient)

    def get_position_orientation(
        self,
    ) -> Union[Tuple[np.ndarray, np.ndarray], Tuple[torch.Tensor, torch.Tensor]]:
        """
        Gets object's pose with respect to the world's frame.

        Returns:
            2-tuple:
                - 3-array: (x,y,z) position in the world frame
                - 4-array: (w, x, y, z) quaternion orientation in
                the world frame
        """
        return self._xprim.get_world_pose()

    def set_kinematic(self, flag: bool):
        if flag:
            # zero out velocity
            self.set_angular_velocity(velocity=np.zeros(3))
            self.set_linear_velocity(velocity=np.zeros(3))
        self._prim.GetAttribute("physics:kinematicEnabled").Set(flag)

    def set_angular_velocity(self, velocity: np.ndarray):
        self._rigid_api.GetAngularVelocityAttr().Set(Gf.Vec3f(velocity.tolist()))

    def set_linear_velocity(self, velocity: np.ndarray):
        self._rigid_api.GetVelocityAttr().Set(Gf.Vec3f(velocity.tolist()))

    def get_angular_velocity(self) -> np.ndarray:
        return np.array(self._rigid_api.GetAngularVelocityAttr().Get())

    def get_linear_velocity(self) -> np.ndarray:
        return np.array(self._rigid_api.GetVelocityAttr().Get())

    def get_obs(self):
        return {
            "position": self.get_position_orientation()[0].numpy(),
            "orientation": self.get_position_orientation()[1].numpy(),
        }

    @property
    def prim_path(self):
        return self._prim_path

    @property
    def prim(self):
        return self._prim
