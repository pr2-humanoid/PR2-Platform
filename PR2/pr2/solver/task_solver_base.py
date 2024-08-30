from abc import ABC, abstractmethod


class TaskSolverBase(ABC):
    @abstractmethod
    def next_action(self, obs: dict) -> dict:
        """Generate next step action given current observation

        An example of the observation space is provided below,
        where H and W are the height and width of camera images,
        N and M are the number of arm and leg joints.

        Args:
            obs(dict) = {
                "agent": {
                    "joint_state": {
                        "arms_positions":  np.ndarray of shape (N,)
                        "arms_velocities": np.ndarray of shape (N,)
                        "arms_applied_effort": np.ndarray of shape (N,)
                        "legs_positions": np.ndarray of shape (M,)
                        "legs_velocities": np.ndarray of shape (M,)
                        "legs_applied_effort": np.ndarray of shape (M,)
                    },
                    "body_state": {
                        "world_pos": np.ndarray np.ndarray of shape (3,)
                        "world_orient": np.ndarray of shape (4,)
                        "linear_velocities": np.ndarray of shape (3,)
                        "angular_velocities":np.ndarray of shape (3,)
                    },
                },
                'cam': {
                    'rgb': np.ndarray of shape (H, W, 3) ,
                    "distance_to_image_plane": np.ndarray of shape (H, W)
                },
                "goal": str,
                "pick": bool,
                . . .
                . .
                .
            }

        Returns:
            An example action command is shown below, where N and M
            are the number of arm and leg joints.

            dict: {
                "arms": {
                    "ctrl_mode": "position", // "velocity"/"effort"
                    "joint_values": None, // np.ndarray of shape (N,)
                    "stiffness": None, // np.ndarray of shape (N,)
                    "dampings": None, // np.ndarray of shape (N,)
                },
                "legs": {
                    "ctrl_mode": "effort", // "position"/"velocity"
                    "joint_values":  np.array([val] * M), // None
                    "stiffness": None,  // np.ndarray of shape (M,)
                    "dampings": None,  //  np.ndarray of shape (M,)
                },
                "pick": None, // "left_hand"/"right_hand"
                "release": False // True
            }
        """
        return {}
