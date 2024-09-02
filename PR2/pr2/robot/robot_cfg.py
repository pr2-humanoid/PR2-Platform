from dataclasses import dataclass
from typing import List, Union

import numpy as np


@dataclass
class KuavoCfg:
    """Configuration parameters for a robot."""

    class MetaInfoCfg:
        """Meta-information about the robot."""

        # NOTE: DO NOT modify the order of elements in the list
        # ('l_shoulder_y', 'l_shoulder_z', 'l_shoulder_x', 'l_elbow',
        #  'r_shoulder_y', 'r_shoulder_z', 'r_shoulder_x', 'r_elbow')
        arm_idx: List = [1, 5, 9, 13, 3, 7, 11, 15]
        # ('l_hip_z', 'l_hip_x', 'l_hip_y', 'l_knee', 'l_ankle',
        #  'r_hip_z', 'r_hip_x', 'r_hip_y', 'r_knee', 'r_ankle')
        leg_idx: List = [0, 4, 8, 12, 16, 2, 6, 10, 14, 17]
        dof_names: List = [
            "l_hip_z",
            "l_shoulder_y",
            "r_hip_z",
            "r_shoulder_y",
            "l_hip_x",
            "l_shoulder_z",
            "r_hip_x",
            "r_shoulder_z",
            "l_hip_y",
            "l_shoulder_x",
            "r_hip_y",
            "r_shoulder_x",
            "l_knee",
            "l_elbow",
            "r_knee",
            "r_elbow",
            "l_ankle",
            "r_ankle",
        ]
        body_names: List = [
            "torso",
            "l_uleg_z",
            "l_uarm_y",
            "r_uleg_z",
            "r_uarm_y",
            "l_uleg_x",
            "l_uarm_z",
            "r_uleg_x",
            "r_uarm_z",
            "l_uleg",
            "l_uarm",
            "r_uleg",
            "r_uarm",
            "l_lleg",
            "l_larm",
            "r_lleg",
            "r_larm",
            "l_foot",
            "l_arm_sphere",
            "r_foot",
            "r_arm_sphere",
        ]

    class DefaultStateCfg:
        """Initial state of the robot."""

        # dof state
        dof_pos: Union[List[float], np.ndarray] = [
            -0.005795,
            np.pi / 4,
            0.005606,
            np.pi / 4,
            -0.03286,
            0.0,
            0.03178,
            0.0,
            -1.492,
            0.0,
            -1.492,
            0.0,
            2.075,
            -np.pi * 2 / 3,
            2.074,
            -np.pi * 2 / 3,
            -0.7569,
            -0.7569,
        ]
        """DOF positions of all joints."""
        dof_vel: Union[List[float], np.ndarray] = np.zeros(18)
        """DOF velocities of all joints."""
        dof_effort: Union[List[float], np.ndarray] = np.zeros(18)
        """DOF velocities of all joints."""

        stiffness: Union[List[float], np.ndarray] = [500.0, 50.0] * 8 + [500, 500]
        """
        Stiffness gains of the DOFs
        """
        damping: Union[List[float], np.ndarray] = [10, 2] * 8 + [10, 10]
        """
        Damping gains of the DOFs
        """

    ##
    # Initialize configurations.
    ##
    meta_info: MetaInfoCfg = MetaInfoCfg()
    """Meta-information about the robot."""
    default_state: DefaultStateCfg = DefaultStateCfg()
    """Initial state of the robot."""
