from omni.isaac.core.articulations.articulation import Articulation

from pr2.objects.object import Object


# pylint: disable=not-an-iterable
class ArticulatedObject(Object):
    def __init__(self, name: str = "valve_47_link"):
        super().__init__(name)
        self._art = Articulation(prim_path=self._prim_path)

    def initialize(self):
        super().initialize()
        self._art.initialize()

    def get_joint_positions(self):
        return self._art.get_joint_positions()
