import panda3d.core as p3d


def axes(scale: float = 0.5) -> p3d.GeomNode:
    segments = p3d.LineSegs()
    origin = p3d.LPoint3f.origin()
    for i in range(3):
        color = p3d.LColorf()
        color[i] = 1.0
        axis = p3d.LVector3f()
        axis[i] = 1.0
        segments.set_thickness(2)
        segments.set_color(color)
        segments.move_to(origin)
        segments.draw_to(axis * scale)
    return segments.create()
