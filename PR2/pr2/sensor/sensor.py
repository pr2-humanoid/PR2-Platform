from typing import Dict 
import omni.replicator.core as rep 

# Camera param: https://docs.omniverse.nvidia.com/py/replicator/1.10.10/source/extensions/omni.replicator.core/docs/API.html#cameras  # noqa # pylint: disable=C0301


class Sensor:
    def __init__(self, name, cam_params, resolution):
        # Setup camera
        self._name = name
        self.camera = rep.create.camera(name=name, **cam_params)

        # init render product
        rp = rep.create.render_product(self.camera, resolution=resolution)
        self.rgb = rep.AnnotatorRegistry.get_annotator("rgb")

        self.rgb.attach([rp])
        self.distance_to_image_plane = rep.AnnotatorRegistry.get_annotator(
            "distance_to_image_plane"
        )
        self.distance_to_image_plane.attach([rp])

    def get_name(self) -> str:
        """
        Returns:
            str: camera_name
        """
        return self._name

    def get_obs(self) -> Dict:
        """
        Returns:
            "rgb": low dynamic range output image as an array of type np.uint8
            with shape (width, height, 4), where the four channels correspond
            to R,G,B,A.
            "distance_to_image_plane": Outputs a depth map from objects to image
            plane of the camera. The distance_to_image_plane annotator produces
            a 2d array of types np.float32 with 1 channel.

        Note:
            "distance_to_image_plane":
            The unit for distance to image plane is in meters (For example, if the
            object is 1000 units from the image plane of the camera, and the
            meters_per_unit variable of the scene is 100, the distance to c
            amera would be 10).

            0 in the 2d array represents infinity (which means there is no object
            in that pixel).
        """

        res = {
            "rgb": self.rgb.get_data(),
            "distance_to_image_plane": self.distance_to_image_plane.get_data(),
        }

        return res
