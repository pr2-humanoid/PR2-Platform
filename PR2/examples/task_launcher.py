import sys

import pr2  # noqa # pylint: disable=wrong-import-order # isort: skip
from pr2.env import Env

# pylint: disable=pointless-string-statement
"""
NOTE:
call env.reset() to reset both the task and agent to their initial states.

action(dict): control command from the controller
  1. Control mode can only be position or effort.
  2. The order of values in the 'joint_values' corresponds to the order of elements
  in the 'agent_cfg' arm_idx and leg_idx.
  3. Ensure the key in action remains unchanged.
  4. For Task2 and Task6, set the value in the "pick" key to either "left_hand"
  or "right_hand" to specify the hand you are using if you are ready to attach
  the valve or object. To release, set the "release" key to True if you are
  ready to release the valve or object.

  An example action command is shown below, where N and M are the number of arm and leg joints.
    action = {
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
 

observations(dict):  dictionary containing information about the current state
    1. In Task6, the observation(`obs`) can include RGBD information from the camera,
    stored in the key "cam" and the task goal which is randomly generated.
    2. In both Task2 and Task6, the observation(`obs`) will indicate whether the object
    or valve is attached to the hand, with the status stored in the key "pick".
    If the attachment is successful, the value will be True, otherwise,
    it will be False.
    3. In Task3, the observation(`obs`) includes obstacle pose information

    An example of the observation space is provided below,
    where H and W are the height and width of camera images,
    N and M are the number of arm and leg joints.

    Args:
        obs = {
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
            "obstacle":{'position':np.ndarray of shape (3,), 
            'orientation': np.ndarray of shape (4,)}
        
        }

 
"""  # pylint: disable=pointless-string-statement

TASK_SEED = {1: 66, 2: 266, 3: 88, 4: 66, 5: 666, 6: 666}


def main(task_id):
    # The seed will be randomly generated in the actual challenge.
    # If you assign 'None' to the seed, the task will be configured randomly.
    # You can test the robutness of your model by having `seed = None`.
    env = Env(task_num=task_id, seed=TASK_SEED[task_id])
    action = {}

    # create solver for task 1
    # pylint: disable=import-outside-toplevel,exec-used
    exec(f"from task_{task_id}_solver.task_solver import TaskSolver", globals())
    # pylint: disable=undefined-variable
    solver = TaskSolver()  # noqa: F821

    # Reset the environment before calling step
    # Important: Please do not remove this line.
    env.reset()

    while True:
        # If simulation is stopped, then exit.
        # Important: Please do not remove this line.
        if pr2.sim.is_stopped():
            break

        # If simulation is paused, then skip.
        # Important: Please do not remove this line.
        if not pr2.sim.is_playing():
            pr2.sim.render()
            continue

        # an empty action will be applied at the very begining
        # stage to get an observation
        done, success, obs = env.step(action)

        if done:
            if success:
                print("Task passed")
            else:
                print("Task failed")
            break

        # pass observation to solver and get action for next step
        action = solver.next_action(obs)

    pr2.shutdown()


if __name__ == "__main__":
    main(int(sys.argv[1]))
