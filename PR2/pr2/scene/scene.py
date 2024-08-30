import json
from typing import Dict

from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.stage import add_reference_to_stage

import pr2 
from pr2.objects.articulated_object import ArticulatedObject
from pr2.objects.object import Object

SCENE_USD_PATH = str(pr2.ROOT_PATH.parent / "data" / "scene" / "main.usd")
SCENE_METADATA_PATH = str(pr2.ROOT_PATH.parent / "data" / "scene" / "obj_cfg.json")


class Scene:
    def __init__(self):
        self._prim_path = "/World/Scene"
        self._prim = XFormPrim(self._prim_path)
        self._objects = {}
        # load scene to the stage
        add_reference_to_stage(usd_path=SCENE_USD_PATH, prim_path=self._prim_path)

    def initialize(self) -> None:
        self._prim.set_world_pose(position=(0, 0, 0), orientation=(1, 0, 0, 0))
        # Load objects from json file
        with open(SCENE_METADATA_PATH, encoding="utf-8") as f:
            data = json.load(f)
            for obj_baselink, config in data["objects"].items():
                # Add rigid objects only
                if config["rigidbody"] is True:
                    if config["articulated"] is True:
                        obj = ArticulatedObject(name=obj_baselink)
                    else:
                        obj = Object(name=obj_baselink)
                    self._objects[obj_baselink] = obj
                    obj.initialize()

    def get_objects(self) -> Dict:
        return self._objects
