import random

import numpy as np
from omni.isaac.version import get_version

from .termination_conditions import (
    Contact,
    Falling,
    PositionComparator,
    ReachingGoal,
    Rotation,
    TimeOut,
)


def is_isaac_sim_4_2_0():
    # ruff: noqa: PLR2004 
    return get_version()[0] == '4.2.0'


def set_global_seed(seed):
    random.seed(seed)
    np.random.seed(seed)


def convert_data_format(backend, device, backend_utils, data):
    if data is not None:
        data = backend_utils.convert(data, device)
        data = backend_utils.expand_dims(data, 0)
        if backend == "torch":
            data = data.float()
    return data
