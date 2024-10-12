from typing import Dict, List, Optional, Sequence, Union

import numpy as np
import omni.isaac.core.utils.torch as torch_utils
import torch
from omni.isaac.core.articulations.articulation_view import ArticulationView
from omni.isaac.core.prims import RigidPrimView
from omni.isaac.core.utils.prims import get_prim_at_path
from omni.isaac.core.utils.stage import add_reference_to_stage

import pr2 
from pr2.utils import convert_data_format

from .robot_cfg import KuavoCfg

AGENT_USD_PATH = str(pr2.ROOT_PATH.parent / "data" / "agent" / "main.usd")


class Robot(ArticulationView):

    """Configuration for the robot."""

    def __init__(
        self,
        position: Optional[Sequence[float]] = (0, 0, 0),
        orientation: Optional[Sequence[float]] = (1, 0, 0, 0),
        scale: Optional[Sequence[float]] = None,
    ):
        self.cfg = KuavoCfg
        self._prim_path = "/World/Kuavo"
        # Load Agent
        add_reference_to_stage(usd_path=AGENT_USD_PATH, prim_path=self._prim_path)
        self._prim = get_prim_at_path(self._prim_path)
        if pr2.sim.instance() is not None:
            self._backend = pr2.sim.instance().backend
            self._device = pr2.sim.instance().device
            self._backend_utils = pr2.sim.instance().backend_utils
        else:
            self._backend = "torch"
            self._device = None
            self._backend_utils = torch_utils

        position = convert_data_format(
            self._backend, self._device, self._backend_utils, position
        )
        orientation = convert_data_format(
            self._backend, self._device, self._backend_utils, orientation
        )
        scale = convert_data_format(
            self._backend, self._device, self._backend_utils, scale
        )

        super().__init__(
            prim_paths_expr=self._prim_path,
            name="Kuavo",
            positions=position,
            orientations=orientation,
            scales=scale,
        )
        self._bodies = {}

    def initialize(self, physics_sim_view=None):
        super().initialize(physics_sim_view)
        for body_name in self.body_names:
            body = RigidPrimView(
                prim_paths_expr=f"{self._prim_path}/{body_name}",
                name={body_name},
                reset_xform_properties=False,
            )
            body.initialize()
            self._bodies[body_name] = body

        # -- meta-information
        self._process_info_cfg()
        # -- set default gains
        self.set_gains(kps=self._default_dof_stiffness, kds=self._default_dof_damping)
        # -- set default state
        self.set_joints_default_state(
            positions=self._default_dof_pos,
            velocities=self._default_dof_vel,
            efforts=self._default_dof_effort,
        )
        self.set_linear_velocities(torch.Tensor([[0] * 3]))
        self.set_angular_velocities(torch.Tensor([[0] * 3]))
        self.post_reset()

    def get_obs(self):
        return {
            "joint_state": self.get_joint_state(),
            "body_state": self.get_body_state(),
            "stiffness": self.get_gains(clone=False)[0].cpu().numpy()[0],
            "dampings": self.get_gains(clone=False)[1].cpu().numpy()[0],
        }

    def act(self, command_dict: Dict[str, Union[str, List, np.ndarray]]) -> None:
        if command_dict is None:
            return
        self._act(self._arm_idx, command_dict.get("arms", None))
        self._act(self._leg_idx, command_dict.get("legs", None))

    def get_joint_state(self) -> Dict:
        """
        Returns:
            Dict: keyword mapping agent's joint state

        Note:
            joint_names: ordered names of joints
            joint_pos: joint positions of agent in the view.
            joint_vel: joint velocities of agent in the view.

        """
        joint_positions = self.get_joint_positions(clone=False).cpu().numpy()[0]
        joint_velocities = self.get_joint_velocities(clone=False).cpu().numpy()[0]
        joint_applied_efforts = (
            self.get_applied_joint_efforts(clone=False).cpu().numpy()[0]
        )
        arms_positions = joint_positions[self._arm_idx]
        arms_velocities = joint_velocities[self._arm_idx]
        arms_applied_effort = joint_applied_efforts[self._arm_idx]
        legs_positions = joint_positions[self._leg_idx]
        legs_velocities = joint_velocities[self._leg_idx]
        legs_applied_effort = joint_applied_efforts[self._leg_idx]

        return {
            "arms_positions": arms_positions,
            "arms_velocities": arms_velocities,
            "arms_applied_effort": arms_applied_effort,
            "legs_positions": legs_positions,
            "legs_velocities": legs_velocities,
            "legs_applied_effort": legs_applied_effort,
        }

    def get_body_state(self) -> Dict:
        """
        Returns:
            Dict: keyword mapping agent's body state

        Note:
            body_names: ordered names of bodies
            world_pos: positions in the view with respect to the world’s frame
            world_orient: quaternion orientations in the world frame of the prims.
                        quaternion is scalar-first (w, x, y, z).
            linear_velocities: linear velocities of agent
            angular velocities: angular velocities of prims in the view.
        """
        return {
            "world_pos": self.get_world_poses(clone=False)[0].cpu().numpy()[0],
            "world_orient": self.get_world_poses(clone=False)[1].cpu().numpy()[0],
            "linear_velocities": self.get_linear_velocities(clone=False)
            .cpu()
            .numpy()[0],
            "angular_velocities": self.get_angular_velocities(clone=False)
            .cpu()
            .numpy()[0],
        }

    def get_arm_idx(self) -> List[int]:
        return self._arm_idx

    def get_leg_idx(self) -> List[int]:
        return self._leg_idx

    @property
    def bodies(self):
        return self._bodies

    # ===============
    # Internal helper.
    # ===============

    # pylint: disable=attribute-defined-outside-init
    def _process_info_cfg(self) -> None:
        """Post processing of configuration parameters."""
        # -- meta_info: joint index
        self._arm_idx = self.cfg.meta_info.arm_idx
        self._leg_idx = self.cfg.meta_info.leg_idx 

        # -- dof state
        self._default_dof_pos = convert_data_format(
            self._backend,
            self._device,
            self._backend_utils,
            self.cfg.default_state.dof_pos,
        )

        self._default_dof_vel = convert_data_format(
            self._backend,
            self._device,
            self._backend_utils,
            self.cfg.default_state.dof_vel,
        )

        self._default_dof_effort = convert_data_format(
            self._backend,
            self._device,
            self._backend_utils,
            self.cfg.default_state.dof_effort,
        )

        self._default_dof_stiffness = convert_data_format(
            self._backend,
            self._device,
            self._backend_utils,
            self.cfg.default_state.stiffness,
        )
        self._default_dof_damping = convert_data_format(
            self._backend,
            self._device,
            self._backend_utils,
            self.cfg.default_state.damping,
        )

    def _act(self, idx: Sequence, command_dict: dict) -> None:
        if not command_dict:
            return

        if command_dict.get("ctrl_mode", None) not in ["position", "effort"]:
            raise ValueError("control mode can only be position or effort")

        if self._is_exist(command_dict.get("stiffness", None)) or self._is_exist(
            command_dict.get("dampings", None)
        ):
            self.set_gains(
                kps=convert_data_format(
                    self._backend,
                    self._device,
                    self._backend_utils,
                    command_dict.get("stiffness", None),
                ),
                kds=convert_data_format(
                    self._backend,
                    self._device,
                    self._backend_utils,
                    command_dict.get("dampings", None),
                ),
                joint_indices=idx,
            )

        if command_dict.get("ctrl_mode", None) == "position":
            self.switch_control_mode("position", joint_indices=idx)
            if self._is_exist(command_dict.get("joint_values", None)):
                self.set_joint_position_targets(
                    positions=torch.FloatTensor(command_dict.get("joint_values", None)),
                    joint_indices=idx,
                )

        else:
            self.switch_control_mode("effort", joint_indices=idx)
            if self._is_exist(command_dict.get("joint_values", None)):
                self.set_joint_efforts(
                    efforts=torch.FloatTensor(command_dict.get("joint_values", None)),
                    joint_indices=idx,
                )

    def _is_exist(self, val):
        if val is None:
            return False
        if isinstance(val, list) and not val:
            return False
        if isinstance(val, np.ndarray) and val.size == 0:
            return False
        return True
