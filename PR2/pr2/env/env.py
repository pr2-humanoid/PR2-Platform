from typing import Dict, Tuple

import pr2  
from pr2.robot.robot import Robot
from pr2.objects.object import Object
from pr2.scene.scene import Scene
from pr2.tasks.task_five import TaskFive
from pr2.tasks.task_four import TaskFour
from pr2.tasks.task_one import TaskOne
from pr2.tasks.task_six import TaskSix
from pr2.tasks.task_three import TaskThree
from pr2.tasks.task_two import TaskTwo
from pr2.utils import set_global_seed

TASKS = {1: TaskOne, 2: TaskTwo, 3: TaskThree, 4: TaskFour, 5: TaskFive, 6: TaskSix}


class Env:
    """
    Core environment class that handles loading scene, agent and task
    """

    def __init__(self, task_num: int = 1, seed: int = None) -> None:
        self._objects = None
        self._scene = None
        self._agent = None
        self._cam = None
        self._task_num = task_num
        self._task = None
        self._sim_time = 0
        self._seed = seed
        # Set the global random seed
        self.set_random_seed(self._seed)
        # Set the simulator settings
        pr2.sim.set_simulation_dt(physics_dt=0.001, rendering_dt=0.001)
        # Load this environment
        self.load()

    def load(self) -> None:
        """
        Load the scene, agent, and task
        """
        self._load_scene()
        self._load_agent()
        self._load_task()
        pr2.sim.reset(soft=False)

        # initialize scene, task and agent
        self.scene.initialize()
        self.task.initialize()
        self.agent.initialize()

    def get_object_by_name(self, baselink: str = None) -> Object:
        """
        Args:
            baselink (str): objects link name in usd
        Returns:
            object (Object): object in this scene

        """
        if self._objects.get(baselink, None) is not None:
            return self._objects.get(baselink)

        return None

    def reset(self) -> Tuple[bool, bool, dict]:
        """
        Reset scene, agent and task
        """
        # set random seed before reset scene and task
        self.set_random_seed(self._seed)

        done, success, obs = self.task.reset()
        pr2.sim.step()

        return done, success, obs

    def set_random_seed(self, seed: int) -> None:
        """
        Set environment global random seed

        Args:
            seed (int): global random seed
        """
        self._seed = seed

        if self._seed is not None:
            set_global_seed(self._seed)

    def step(self, action: dict) -> Tuple[bool, bool, Dict]:
        """
        1. Apply action
        2. Get observation
        3. Check the task's status

        Args:
            action (Dict): each entry should
                map robot's joint name to corresponding control mode and val

        Returns:
            3-tuple:
                - bool: if the task is completed, irrespective of success or failure
                - bool: if the task has been successfully finished.
                - Dict: current observation
        """
        done, success, obs = self._task.step(action)
        self._sim_time += pr2.sim.get_physics_dt()
        return done, success, obs

    @property
    def agent(self) -> Robot:
        """
        Returns:
            Robot: robot in the env
        """
        return self._agent

    @property
    def scene(self) -> Scene:
        """
        Returns:
            Scene: scene in the env
        """
        return self._scene

    @property
    def task(self) -> Scene:
        """
        Returns:
            Scene: scene in the env
        """
        return self._task

    @property
    def total_sim_time(self) -> float:
        """
        Returns:
            float: total simulation time
        """
        return self._sim_time

    # Internal Helper

    def _load_scene(self) -> None:
        """
        Import scene
        """
        self._scene = Scene()
        self._objects = self._scene.get_objects()

    def _load_agent(self) -> None:
        """
        Import agent
        """
        self._agent = Robot()

    def _load_task(self) -> None:
        """
        Load Task
        """
        self._task = TASKS[self._task_num](env=self)
