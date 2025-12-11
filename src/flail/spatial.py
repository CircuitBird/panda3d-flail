import math

import panda3d.core as p3d


def required_rotation(u: p3d.LVecBase3, v: p3d.LVecBase3, /) -> p3d.LQuaternion:
    """Return a quaternion representing the rotation
    required to align `u` with `v`.
    """
    u, v = u.normalized(), v.normalized()
    axis = u.cross(v)
    axis.normalize()
    dot_product = u.dot(v)
    cosine = math.sqrt((1 + dot_product) / 2)
    sine = math.sqrt((1 - dot_product) / 2)
    return p3d.LQuaternion(cosine, axis * sine)
