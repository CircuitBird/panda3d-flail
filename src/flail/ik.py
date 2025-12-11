from typing import Protocol

import numpy as np
import panda3d.core as p3d

from flail.arraytypes import Array1D, Array2D


class HasJacobian(Protocol):
    def jacobian(self) -> Array2D: ...


class IKAlgorithm(Protocol):
    """A callback protocol for generating joint velocity commands."""

    def __call__(self, arm: HasJacobian, xd_cmd: p3d.LVector3, /) -> Array1D: ...


def j_transpose(arm: HasJacobian, xd_cmd: p3d.LVector3) -> Array1D:
    # Note that we're using Panda3D's row vector standard.
    jac_v = arm.jacobian()[:, 0:3]
    return np.array(xd_cmd) @ jac_v.T


def j_pinv(arm: HasJacobian, xd_cmd: p3d.LVector3) -> Array1D:
    # Note that we're using Panda3D's row vector standard.
    jac_v = arm.jacobian()[:, 0:3]
    jac_dag = np.linalg.pinv(jac_v)
    return np.array(xd_cmd) @ jac_dag


def j_pinv_damped(
    arm: HasJacobian, xd_cmd: p3d.LVector3, *, kp2: float = 1e-6
) -> Array1D:
    # Note that we're using Panda3D's row vector standard.
    jac_v = arm.jacobian()[:, 0:3]
    jac_star = np.linalg.inv(np.eye(3) * kp2 + jac_v.T @ jac_v) @ jac_v.T
    return np.array(xd_cmd) @ jac_star
