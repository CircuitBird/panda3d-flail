from typing import TypeAlias

import panda3d.bullet as bt
import panda3d.core as p3d

from flail import spatial

BodyPath: TypeAlias = 'p3d.NodePath[bt.BulletRigidBodyNode]'


def attach_member(
    base: BodyPath,
    tip: BodyPath,
    *,
    at_base: bool = False,
    mass: float = 0.0,
    collide_mask: p3d.CollideMask = p3d.CollideMask.all_off(),
) -> None:
    body = (base if at_base else tip).node()
    body.linear_sleep_threshold = 0.0
    body.mass = mass
    body.into_collide_mask = collide_mask
    span = tip.get_pos(base) if at_base else base.get_pos(tip)
    shape = bt.BulletCapsuleShape(0.1, span.length())
    q = spatial.required_rotation(p3d.LVector3.unit_z(), span)
    shape_xform = p3d.TransformState.make_pos_quat_scale(
        pos=span * 0.5, quat=q, scale=(1.0, 1.0, 1.0)
    )
    body.add_shape(shape, shape_xform)
