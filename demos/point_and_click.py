import math

import numpy as np
import panda3d.bullet as bt
import panda3d.core as p3d
from direct.showbase.ShowBase import ShowBase

import flail.arms

base = ShowBase()
base.cam.setPos(-10, -10, 10)
base.cam.lookAt(0, 0, 0)

world = bt.BulletWorld()
world.set_gravity(0, 0, -0.1)

debug_node = bt.BulletDebugNode()
debug_path = base.render.attach_new_node(debug_node)
debug_path.show()
world.set_debug_node(debug_node)

ground_node = bt.BulletRigidBodyNode('ground')
ground_node.add_shape(bt.BulletPlaneShape(p3d.LVector3.unit_z(), 0))
ground_node.into_collide_mask = p3d.CollideMask.all_on()
ground_path = base.render.attach_new_node(ground_node)
world.attach(ground_node)


def dh_xform(theta: float, d: float, a: float, alpha: float) -> p3d.LMatrix4:
    st, ct = math.sin(theta), math.cos(theta)
    sa, ca = math.sin(alpha), math.cos(alpha)
    # fmt: off
    return p3d.LMatrix4(
            ct,     st, 0., 0.,
        -st*ca,  ct*ca, sa, 0.,
         st*sa, -ct*sa, ca, 0.,
          a*ct,   a*st,  d, 1.,
    )
    # fmt: on


arm = flail.arms.Arm.from_xforms(
    ground_path,
    [
        dh_xform(0, 1, 0, np.pi / 2),
        dh_xform(0, 0, 1, np.pi / 2),
        dh_xform(0, 0, 1, -np.pi / 2),
        dh_xform(0, 0, 1, -np.pi / 2),
    ],
)
arm.attach_to_world(world)
arm.draw_axes()
controller = flail.arms.PointArmController(arm)


async def update() -> None:
    clock = p3d.ClockObject.get_global_clock()
    prev_time = clock.frame_time
    while True:
        now = clock.frame_time
        world.do_physics(now - prev_time)
        prev_time = now
        await p3d.AsyncTaskPause(0.0)


async def click() -> int:
    await p3d.EventHandler.get_global_event_handler().get_future('mouse1')
    mouse_pos = base.mouseWatcherNode.get_mouse()
    lens = base.cam.node().get_lens()
    near_point = p3d.LPoint3()
    far_point = p3d.LPoint3()
    lens.extrude(mouse_pos, near_point, far_point)
    origin = ground_path.get_relative_point(base.cam, near_point)
    endpoint = ground_path.get_relative_point(base.cam, far_point)
    result = world.ray_test_closest(origin, endpoint)
    if result.node == ground_node:
        await controller.update(result.hit_pos)
    return p3d.PythonTask.DS_cont


task_manager = p3d.AsyncTaskManager.get_global_ptr()
update_task = p3d.PythonTask(update())
click_task = p3d.PythonTask(click)
click_task.set_args((), append_task=False)
task_manager.add(click_task)
task_manager.add(update_task)

base.run()
