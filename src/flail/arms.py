from collections.abc import Iterable, Sequence
from typing import Protocol, Self

import attrs
import numpy as np
import panda3d.bullet as bt
import panda3d.core as p3d

from flail import ctrlfuncs, debug, ik, traj
from flail.arraytypes import Array1D, Array2D
from flail.bodies import BodyPath, attach_member


class Joint(Protocol):
    @property
    def prev_link(self) -> BodyPath: ...
    @property
    def this_link(self) -> BodyPath: ...
    @property
    def q(self) -> float: ...
    def drive(self, dq: float, max_impulse: float = ..., /) -> object: ...
    def jacobian(self, base: p3d.NodePath, poi: p3d.NodePath, /) -> Array1D: ...
    def attach_to_world(self, world: bt.BulletWorld, /) -> object: ...


@attrs.define
class HingeJoint(Joint):
    hinge: bt.BulletHingeConstraint
    prev_link: BodyPath
    this_link: BodyPath

    @property
    def q(self) -> float:
        return self.hinge.hinge_angle

    def drive(self, qd: float, max_impulse: float = 10.0) -> None:
        self.hinge.enable_angular_motor(True, qd, max_impulse)

    def jacobian(self, base: p3d.NodePath, poi: p3d.NodePath) -> Array1D:
        z = p3d.LVector3f.unit_z()
        r = poi.get_pos(self.prev_link)
        z0 = base.get_relative_vector(self.prev_link, z)
        r0 = base.get_relative_vector(self.prev_link, r)
        z0_x_r0 = z0.cross(r0)
        return np.array([*z0_x_r0, *z0])

    def attach_to_world(self, world: bt.BulletWorld) -> None:
        world.attach(self.this_link.node())
        world.attach(self.hinge)

    @classmethod
    def make(cls, prev: BodyPath, this: BodyPath) -> Self:
        hinge = bt.BulletHingeConstraint(
            node_a=prev.node(),
            node_b=this.node(),
            ts_a=p3d.TransformState.make_identity(),
            ts_b=prev.get_transform(this),
            use_frame_a=True,
        )
        hinge.set_limit(-180.0, 180.0)
        return cls(hinge, prev, this)


@attrs.define
class SliderJoint(Joint):
    slider: bt.BulletSliderConstraint
    prev_link: BodyPath
    this_link: BodyPath

    @property
    def q(self) -> float:
        return self.slider.linear_pos

    def drive(self, qd: float, max_impulse: float = 10.0) -> None:
        self.slider.target_linear_motor_velocity = qd
        self.slider.max_linear_motor_force = max_impulse
        self.slider.powered_linear_motor = True

    def jacobian(self, base: p3d.NodePath, poi: p3d.NodePath) -> Array1D:
        z0 = base.get_relative_vector(self.prev_link, p3d.LVector3.unit_z())
        return np.array([*z0, 0.0, 0.0, 0.0])

    def attach_to_world(self, world: bt.BulletWorld) -> None:
        world.attach(self.this_link.node())
        world.attach(self.slider)

    @classmethod
    def make(cls, prev: BodyPath, this: BodyPath) -> Self:
        # This aligns the slider's x-axis with the joint's z-axis.
        align_x_to_z = p3d.TransformState.make_hpr((0, 0, -90))
        slider = bt.BulletSliderConstraint(
            node_a=prev.node(),
            node_b=this.node(),
            frame_a=align_x_to_z,
            frame_b=prev.get_transform(this).compose(align_x_to_z),
            use_frame_a=True,
        )
        slider.lower_linear_limit = 0.0
        slider.upper_linear_limit = 1.0
        return cls(slider, prev, this)


@attrs.define
class Arm:
    joints: Sequence[Joint]

    @property
    def base(self) -> BodyPath:
        return self.joints[0].prev_link

    @property
    def tip(self) -> BodyPath:
        return self.joints[-1].this_link

    @property
    def current_position(self) -> p3d.LPoint3:
        return self.tip.get_pos(self.base)

    @property
    def current_velocity(self) -> p3d.LVector3:
        velocity = self.tip.node().linear_velocity
        root = self.base.ancestors[-1]
        return self.base.get_relative_vector(root, velocity)

    def jacobian(self) -> Array2D:
        """Calculate and return the current Jacobian of the arm.

        Please note that it is formatted as the transpose of the usual
        Jacobian format, since we use Panda3D's standard for matrices.
        """
        return np.array([joint.jacobian(self.base, self.tip) for joint in self.joints])

    def drive(self, qds: Array1D) -> int:
        for qd, joint in zip(qds, self.joints, strict=True):
            joint.drive(float(qd))
        return p3d.PythonTask.DS_cont

    def attach_to_world(self, world: bt.BulletWorld) -> None:
        for link in self.joints:
            link.attach_to_world(world)

    def draw_axes(self) -> None:
        axes = debug.axes()
        self.base.attach_new_node(axes)
        for joint in self.joints:
            joint.this_link.attach_new_node(axes)

    @classmethod
    def from_xforms(cls, base: BodyPath, xforms: Iterable[p3d.LMatrix4]) -> Self:
        first = base.attach_new_node(bt.BulletRigidBodyNode('joint-link-0'))
        paths = [base, first]
        for i, xform in enumerate(xforms, start=1):
            node = bt.BulletRigidBodyNode(f'joint-link-{i}')
            path = paths[-1].attach_new_node(node)
            path.set_mat(xform)
            paths.append(path)
        return cls.from_node_paths(paths)

    @classmethod
    def from_node_paths(cls, paths: Sequence[BodyPath]) -> Self:
        joints: list[Joint] = []
        for i in range(2, len(paths)):
            base, tip = paths[i - 1 : i + 1]
            attach_member(base, tip, mass=1.0)
            joint = HingeJoint.make(base, tip)
            joints.append(joint)
        return cls(joints)


@attrs.define
class PointArmController:
    arm: Arm
    ctrl_func: ctrlfuncs.ControlFunction = ctrlfuncs.SimplePD(kp=1.5, kd=0.75)
    ik_algorithm: ik.IKAlgorithm = ik.j_pinv

    async def update(self, target: p3d.LPoint3) -> None:
        clock = p3d.ClockObject.get_global_clock()
        marker = p3d.Loader.get_global_ptr().load_sync("jack.egg")
        marker_path = self.arm.base.attach_new_node(marker)
        marker_path.set_scale(0.2)
        t0 = clock.frame_time
        distance = (target - self.arm.current_position).length()
        path = traj.LinearPath(self.arm.current_position, target)
        traveler = traj.CubicTraveler()
        trajectory = traj.Trajectory(path, traveler, time_scale=1.0 / distance)
        while True:
            t = clock.frame_time - t0
            des = trajectory(t)
            marker_path.set_pos(des.pos)
            xd_cmd = self.ctrl_func(
                des.pos,
                des.vel,
                self.arm.current_position,
                self.arm.current_velocity,
            )
            if des.endpoint:
                self.arm.drive(np.zeros(len(self.arm.joints)))
                marker_path.remove_node()
                return
            qd = self.ik_algorithm(self.arm, xd_cmd)
            self.arm.drive(qd)
            await p3d.AsyncTaskPause(0.0)
