__version__ = "0.0.2"
import sys
from pathlib import Path

from isaacsim import SimulationApp

app = SimulationApp(
    {"headless": False, "renderer": "RayTracedLighting", "multi_gpu": False}
)

ROOT_PATH = Path(__file__).parent


def start():
    # pylint: disable=import-outside-toplevel
    from isaacsim.core.api import SimulationContext
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("omni.isaac.version")
    # enable_extension("omni.physx.commands")
    # enable_extension("omni.usdphysics.ui")

    simulator = SimulationContext(backend="torch", set_defaults=True)
    # defaults settings:
    #    [physics_dt = 1.0/ 60.0,
    #    stage units in meters = 0.01 (i.e in cms),
    #    rendering_dt = 1.0 / 60.0,
    #    gravity = -9.81 m / s
    #    ccd_enabled,
    #    stabilization_enabled,
    #    gpu dynamics turned off,
    #    broadcast type is MBP,
    #    solver type is TGS].
    return simulator


def shutdown():
    # shutdown app
    app.close()
    # clean exit
    sys.exit(0)


sim = start()
