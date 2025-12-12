from typing import Protocol

import attrs
import panda3d.core as p3d


class ControlFunction(Protocol):
    """A callback protocol for generating end-effector velocity commands."""

    def __call__(
        self,
        x_des: p3d.LPoint3,
        xd_des: p3d.LVector3,
        x: p3d.LPoint3,
        xd: p3d.LVector3,
    ) -> p3d.LVector3: ...


@attrs.define
class SimplePD(ControlFunction):
    kp: float = 1.0
    kd: float = 0.0
    feedforward: bool = attrs.field(default=True, kw_only=True)

    def __call__(
        self,
        x_des: p3d.LPoint3,
        xd_des: p3d.LVector3,
        x: p3d.LPoint3,
        xd: p3d.LVector3,
    ) -> p3d.LVector3:
        """Calculate a velocity to command.

        Note that `xd_des` should be the desired velocity, not the time
        derivative of the desired position. These are often equal, but not
        when the desired position changes suddenly.
        Note also that setting `xd_des` to 0 is equivalent to the common
        practice of using `xd` in place of the time derivative of
        `x_des - x` in a single-input controller.
        """
        xd_cmd = (x_des - x) * self.kp + (xd_des - xd) * self.kd
        if self.feedforward:
            xd_cmd += xd_des
        return xd_cmd
