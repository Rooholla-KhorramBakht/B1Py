import numpy as np
from B1Py.interfaces import B1HighLevelReal


def test_velocity_clipping():
    robot = B1HighLevelReal(test=True)

    # checks if large velocities are clipped
    vxs = np.random.uniform(-2, 2, 1000)
    vys = np.random.uniform(-2, 2, 1000)

    clipped_vxs = []
    clipped_vys = []

    for vx, vy in zip(vxs, vys):
        _vx, _vy = robot.clip_velocity(vx, vy)
        clipped_vxs.append(_vx)
        clipped_vys.append(_vy)

    clipped_vxs = np.array(clipped_vxs)[:, np.newaxis]
    clipped_vys = np.array(clipped_vys)[:, np.newaxis]
    vs = np.hstack([clipped_vxs, clipped_vys])

    scales = np.sum((vs @ robot.P_v_max) * vs, axis=1)

    assert np.all(scales <= (1 + 1e-6))

    # checks if small velocities are not clipped
    vxs = np.random.uniform(-0.1, 0.1, 1000)
    vys = np.random.uniform(-0.1, 0.1, 1000)

    clipped_vxs = []
    clipped_vys = []

    for vx, vy in zip(vxs, vys):
        _vx, _vy = robot.clip_velocity(vx, vy)
        clipped_vxs.append(_vx)
        clipped_vys.append(_vy)

    clipped_vxs = np.array(clipped_vxs)
    clipped_vys = np.array(clipped_vys)

    assert np.allclose(clipped_vxs, vxs)
    assert np.allclose(clipped_vys, vys)
