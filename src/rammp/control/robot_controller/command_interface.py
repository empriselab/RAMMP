from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray


class KinovaCommand:
    """Establish an interface for commands that can be sent to the robot."""

@dataclass(frozen=True)
class JointCommand(KinovaCommand):
    """Command to set the joint positions."""

    pos: NDArray

    def __init__(self, pos):
        object.__setattr__(self, "pos", np.array(pos))  # convert list to numpy array
        num_dof = 7
        assert self.pos.shape == (num_dof,)


@dataclass(frozen=True)
class CartesianCommand(KinovaCommand):
    """Command to set the cartesian pose."""

    pos: NDArray
    quat: NDArray

    def __init__(self, pos, quat):
        object.__setattr__(self, "pos", np.array(pos))
        object.__setattr__(self, "quat", np.array(quat))
        assert self.pos.shape == (3,)
        assert self.quat.shape == (4,)


class OpenGripperCommand(KinovaCommand):
    """Command to open the gripper."""


class CloseGripperCommand(KinovaCommand):
    """Command to close the gripper."""
