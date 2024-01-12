import sys
from dataclasses import dataclass

import numpy as np


def addROSPath(installation_path):
    """
    Add the ROS installation path to the python path.

    Parameters:
        installation_path (str): Path to the ROS installation directory.
    """
    sys.path.append(installation_path + "/lib/python3/dist-packages")


class NumpyMemMapDataPipe:
    def __init__(self, channel_name, force=False, dtype="uint8", shape=(640, 480, 3)):
        self.channel_name = channel_name
        self.force = force
        self.dtype = dtype
        self.shape = shape
        if force:
            self.shm = np.memmap(
                "/dev/shm/" + channel_name, dtype=self.dtype, mode="w+", shape=shape
            )
        else:
            self.shm = np.memmap(
                "/dev/shm/" + channel_name, dtype=self.dtype, mode="r+", shape=shape
            )

    def write(self, data, match_length=True):
        if match_length:
            self.shm[: data.shape[0], ...] = data
        else:
            assert (
                data.shape == self.shape
            ), "The data and the shape of the shared memory must match"
            self.shm[:] = data

    def read(self):
        return self.shm.copy()


@dataclass
class xKeySwitch:
    R1: int
    L1: int
    start: int
    select: int
    R2: int
    L2: int
    F1: int
    F2: int
    A: int
    B: int
    X: bool
    Y: int
    up: int
    right: int
    down: int
    left: int


@dataclass
class xRockerBtn:
    head: list
    btn: xKeySwitch
    lx: float
    rx: float
    ry: float
    L2: float
    ly: float
