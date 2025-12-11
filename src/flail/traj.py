from typing import Protocol

import attrs
import panda3d.core as p3d


@attrs.define
class ArcPoint:
    s: float
    v: float = 0


@attrs.define
class PathPoint:
    pos: p3d.LPoint3
    vel: p3d.LVector3
    endpoint: bool = attrs.field(default=False, kw_only=True)


@attrs.define
class TrajPoint:
    pos: p3d.LPoint3
    vel: p3d.LVector3
    endpoint: bool = attrs.field(default=False, kw_only=True)


class ArcTravler(Protocol):
    def __call__(self, t: float, /) -> ArcPoint: ...


class Path(Protocol):
    def __call__(self, arc_point: ArcPoint, /) -> PathPoint: ...


@attrs.define
class CubicTraveler(ArcTravler):
    vi: float = 0
    vf: float = 0

    def __call__(self, t: float) -> ArcPoint:
        t = 0 if t < 0 else t if t < 1 else 1
        a = self.vi + self.vf - 2
        b = 3 - 2 * self.vi - self.vf
        s = a * t**3 + b * t**2 + self.vi * t
        v = 3 * a * t**2 + 2 * b * t + self.vi
        return ArcPoint(s, v)


@attrs.define
class LinearPath(Path):
    p0: p3d.LPoint3
    p1: p3d.LPoint3

    def __call__(self, arc_point: ArcPoint) -> PathPoint:
        tangent = self.p1 - self.p0
        pos = self.p0 + tangent * arc_point.s
        return PathPoint(pos, tangent * arc_point.v, endpoint=arc_point.s >= 1)


@attrs.define
class Trajectory:
    path: Path
    traveler: ArcTravler
    time_scale: float = attrs.field(default=1, kw_only=True)

    def __call__(self, t: float) -> PathPoint:
        return self.path(self.traveler(t * self.time_scale))
