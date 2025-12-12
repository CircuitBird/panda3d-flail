# panda3d-flail
A ragdoll character control library for Panda3D

# What?
`panda3d-flail`, or just flail for short, is a Python library that interfaces
with the Panda3D game engine and its bindings for the Bullet physics engine.

The purpose of flail to is to act as a customizable system for creating and
controlling ragdolls characters within a Panda3D game. Flail accomplishes this
by adapting conventions and algorithms from real-world robotics to the context
of Panda3D and Python.

## Modularity
The components within flail are designed to be modular, meaning each element of
a control algorithm can be individually selected and customized for the needs
of your game. If flail doesn't have what you need out-of-the-box, you can make
use of "duck typing" to write it yourself.

Internally, flail makes heavy use of Python's `typing.Protocol`, meaning a type
checker will recognize your types as compatible with flail or tell you what's
wrong if they are not. We also use the `attrs` library (the inspiration for
`dataclasses` in the standard library), so you can safely write your own
constructors for any of the classes defined within flail.

## Project status
Currently, flail is in the very early stages of development. At the moment, it
only contains functionality for building and controlling robotic arms (i.e.
open kinematic chains).

## Constructing an arm
An arm can be constructed using a series of transforms from one joint to the
next via `flail.arms.Arm.from_xforms`. (See `demos/` for an example of how to
convert DH parameters to a Panda3D transformation matrix.) They can also be
constructed between a series of existing `NodePath`s using
`flail.arms.Arm.from_node_paths`.

## Controlling an arm
Flail also contains a system for controlling these arms, such as the algorithm
implemented in `flail.arms.PointArmController`. This algorithm comprises four
distinct parts, each of which can be customized.

The first part of the control algorithm is the `flail.traj.ArcTraveler`
protocol. This protocol describes anything that can be called with a value
between 0 and 1, representing a portion of time elapsed, and returns an
`ArcPoint` instance. An `ArcPoint` contains another number between 0 and 1,
this time representing a portion of an arc length, and a velocity describing
the derivative of that number.

Next in the algorithm is the `flail.traj.Path` protocol, which describes
anything that can be called with an `ArcPoint` object and returns information
about a path at a particular position using an instance of the `PathPoint`
class. This `PathPoint` contains a position, which is the point in space along
the path corresponding to the arc length in the input. It also contains a
velocity, which is the rate of change of this position with respect to the time
input originally passed to the `ArcTraveler`.

The `flail.traj.Trajectory` class bridges implementations of the `ArcTraveler`
and `Path` protocols and allows setting a time scale. It can be called with a
time value (no longer bounded by 0 and 1), which it scales and passes to an
`ArcTraveler`. It passes the result of this call directly to a `Path` to create
a `PathPoint`. It then scales the velocity of the `PathPoint` (in accordance
with the time scale) and returns the object.

`PointArmController` calls a `Trajectory` with an appropriate time value, then
passes the output information to an instance of the
`flail.ctrlfuncs.ControlFunction` protocol. This protocol describes an object
that can be called with desired and current end-effector position and velocity
and returns a velocity to command. By default, this is a simple PD controller.

`PointArmController` then passes this commanded end-effector velocity to an
instance of the `flail.ik.IKAlgorithm` protocol, which handles inverse
kinematics. An `IKAlgorithm` is called with an arm and the commanded velocity
as arguments and returns a 1-D NumPy array containing joint velocities to
command.

These joint velocities are passed directly to `flail.arms.Arm.drive`, which
sends them through Panda3D to Bullet. Bullet then uses its own control
mechanisms to apply motor torques to reach these velocities.

# How?
If you want to experiment with the library for yourself, you can run
`python -m pip install https://github.com/CircuitBird/panda3d-flail.git`
in whatever environment you wish.

If you want to run the demos, you should first clone the repository and create
a virtual environment. Within the repository directory and with the environment
active, use `python -m pip install --editable .` to install the package. Once
it is installed, you can run, for instance, `python demos/point_and_click.py`.

If you want to contribute, you should first make a fork of the project on
GitHub. Then follow the instructions in the paragraph above, but append
`--group dev` to the `pip` command to install additional dependencies for
development. Finally, run `pre-commit install` to install our pre-commit
checks.
