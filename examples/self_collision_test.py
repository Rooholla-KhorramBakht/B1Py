import time

import numpy as np
import pinocchio as pin
import pybullet as p
import pybullet_data
import robot_interface as sdk

import hppfcl  # isort: skip


def q_map_b12pin(q):
    """
    B1 gives the joint angles in the order of
    FR, FL, RR, RL (hip, thigh, calf)

    Pinocchio expects the joint angles in the order of
    FL, FR, RL, RR (hip, thigh, calf)

    This function maps the joint angles from B1 to Pinocchio
    """
    q_fr = q[:3]
    q_fl = q[3:6]
    q_rr = q[6:9]
    q_rl = q[9:]

    q_pin = np.concatenate([q_fl, q_fr, q_rl, q_rr])

    return q_pin


if __name__ == "__main__":
    LOWLEVEL = 0xFF

    udp = sdk.UDP(LOWLEVEL, 8080, "192.168.123.10", 8007)
    safe = sdk.Safety(sdk.LeggedType.B1)

    client = p.connect(p.GUI)
    # Improves rendering performance on M1 Macs
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

    package_directory = "/home/bolun/Documents/B1Py/B1Py/assets/urdf"
    urdf_path = package_directory + "/b1.urdf"
    pin_robot = pin.RobotWrapper.BuildFromURDF(urdf_path, package_directory)

    # Load URDFs
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    p.setAdditionalSearchPath(package_directory)
    robotId = p.loadURDF("b1.urdf", useFixedBase=True)
    p.resetBasePositionAndOrientation(robotId, [0.0, 0.0, 1.0], [0.0, 0.0, 0.0, 1.0])

    # load leg cylinders
    fl_cylinder = p.loadURDF("leg_cylinder.urdf", useFixedBase=True)
    rl_cylinder = p.loadURDF("leg_cylinder.urdf", useFixedBase=True)
    fr_cylinder = p.loadURDF("leg_cylinder.urdf", useFixedBase=True)
    rr_cylinder = p.loadURDF("leg_cylinder.urdf", useFixedBase=True)

    base_offset = pin.SE3(np.eye(3), np.array([0.0, 0.0, 1.0]))

    num_joints = p.getNumJoints(robotId)
    active_joint_ids = []

    for i in range(num_joints):
        # get info of each joint
        _joint_infos = p.getJointInfo(robotId, i)

        if _joint_infos[2] != p.JOINT_FIXED:
            # Save the non-fixed joint IDs
            active_joint_ids.append(_joint_infos[0])

    # for calf collision detection
    calf_ids = {
        "FL": pin_robot.model.getFrameId("FL_calf"),
        "RL": pin_robot.model.getFrameId("RL_calf"),
        "FR": pin_robot.model.getFrameId("FR_calf"),
        "RR": pin_robot.model.getFrameId("RR_calf"),
    }

    # create leg cylinders hppfcl instances
    fl_cylinder_hppfcl = hppfcl.Cylinder(0.02, 0.5)
    rl_cylinder_hppfcl = hppfcl.Cylinder(0.02, 0.5)
    fr_cylinder_hppfcl = hppfcl.Cylinder(0.02, 0.5)
    rr_cylinder_hppfcl = hppfcl.Cylinder(0.02, 0.5)
    calf_offset = pin.SE3(np.eye(3), np.array([0.005237, 0.0, -0.15]))

    cmd = sdk.LowCmd()
    state = sdk.LowState()
    udp.InitCmdData(cmd)
    motiontime = 0

    while True:
        time.sleep(0.002)
        motiontime += 1

        udp.Recv()
        udp.GetRecv(state)

        if motiontime > 10:
            safe.PowerProtect(cmd, state, 1)

        motorStates = state.motorState
        _q, _dq = zip(
            *[(motorState.q, motorState.dq) for motorState in motorStates[:12]]
        )
        q, dq = np.array(_q), np.array(_dq)

        q_pin = q_map_b12pin(q)

        pin_robot.framesForwardKinematics(q_pin)

        fl_calf_pose = base_offset * pin_robot.data.oMf[calf_ids["FL"]] * calf_offset
        rl_calf_pose = base_offset * pin_robot.data.oMf[calf_ids["RL"]] * calf_offset
        fr_calf_pose = base_offset * pin_robot.data.oMf[calf_ids["FR"]] * calf_offset
        rr_calf_pose = base_offset * pin_robot.data.oMf[calf_ids["RR"]] * calf_offset

        # compute distance between calf cylinders
        req = hppfcl.CollisionRequest()
        res = hppfcl.CollisionResult()

        T_fl = hppfcl.Transform3f(fl_calf_pose.rotation, fl_calf_pose.translation)
        T_rl = hppfcl.Transform3f(rl_calf_pose.rotation, rl_calf_pose.translation)
        T_fr = hppfcl.Transform3f(fr_calf_pose.rotation, fr_calf_pose.translation)
        T_rr = hppfcl.Transform3f(rr_calf_pose.rotation, rr_calf_pose.translation)

        col_fl_rl = hppfcl.collide(
            fl_cylinder_hppfcl, T_fl, rl_cylinder_hppfcl, T_rl, req, res
        )
        col_fl_fr = hppfcl.collide(
            fl_cylinder_hppfcl, T_fl, fr_cylinder_hppfcl, T_fr, req, res
        )
        col_fl_rr = hppfcl.collide(
            fl_cylinder_hppfcl, T_fl, rr_cylinder_hppfcl, T_rr, req, res
        )
        col_rl_fr = hppfcl.collide(
            rl_cylinder_hppfcl, T_rl, fr_cylinder_hppfcl, T_fr, req, res
        )
        col_rl_rr = hppfcl.collide(
            rl_cylinder_hppfcl, T_rl, rr_cylinder_hppfcl, T_rr, req, res
        )
        col_fr_rr = hppfcl.collide(
            fr_cylinder_hppfcl, T_fr, rr_cylinder_hppfcl, T_rr, req, res
        )
        in_collision = np.any(
            [col_fl_rl, col_fl_fr, col_fl_rr, col_rl_fr, col_rl_rr, col_fr_rr]
        )

        print(in_collision)

        p.resetBasePositionAndOrientation(
            fl_cylinder,
            fl_calf_pose.translation.tolist(),
            pin.Quaternion(fl_calf_pose.rotation).coeffs().tolist(),
        )
        p.resetBasePositionAndOrientation(
            rl_cylinder,
            rl_calf_pose.translation.tolist(),
            pin.Quaternion(rl_calf_pose.rotation).coeffs().tolist(),
        )
        p.resetBasePositionAndOrientation(
            fr_cylinder,
            fr_calf_pose.translation.tolist(),
            pin.Quaternion(fr_calf_pose.rotation).coeffs().tolist(),
        )
        p.resetBasePositionAndOrientation(
            rr_cylinder,
            rr_calf_pose.translation.tolist(),
            pin.Quaternion(rr_calf_pose.rotation).coeffs().tolist(),
        )

        for i, joint_id in enumerate(active_joint_ids):
            p.resetJointState(robotId, joint_id, q[i], 0.0)

        udp.SetSend(cmd)
        udp.Send()
