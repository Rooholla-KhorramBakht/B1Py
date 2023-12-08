import numpy as np
import pinocchio as pin
import B1Py
import hppfcl  # isort: skip

class PinRobot:
    def __init__(self):
        self.package_directory = B1Py.URDF_DIR_PATH
        self.urdf_path = self.package_directory + "/b1.urdf"
        self.robot = pin.RobotWrapper.BuildFromURDF(
            self.urdf_path, self.package_directory
        )

        self._setup_self_collision_checker()

    def update(self, q):
        q_pin = self.map_joint_angles(q)
        self.robot.framesForwardKinematics(q_pin)

    def map_joint_angles(self, q):
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

    def _setup_self_collision_checker(self):
        # for calf collision detection
        self.calf_ids = {
            "FL": self.robot.model.getFrameId("FL_calf"),
            "RL": self.robot.model.getFrameId("RL_calf"),
            "FR": self.robot.model.getFrameId("FR_calf"),
            "RR": self.robot.model.getFrameId("RR_calf"),
        }

        # create leg cylinders hppfcl instances
        self.fl_cylinder_hppfcl = hppfcl.Cylinder(0.05, 0.5)
        self.rl_cylinder_hppfcl = hppfcl.Cylinder(0.05, 0.5)
        self.fr_cylinder_hppfcl = hppfcl.Cylinder(0.05, 0.5)
        self.rr_cylinder_hppfcl = hppfcl.Cylinder(0.05, 0.5)
        self.calf_offset = pin.SE3(np.eye(3), np.array([0.005237, 0.0, -0.15]))

    def check_calf_collision(self, q):
        """Note: run update(q) before calling this function"""
        fl_calf_pose = self.robot.data.oMf[self.calf_ids["FL"]] * self.calf_offset
        rl_calf_pose = self.robot.data.oMf[self.calf_ids["RL"]] * self.calf_offset
        fr_calf_pose = self.robot.data.oMf[self.calf_ids["FR"]] * self.calf_offset
        rr_calf_pose = self.robot.data.oMf[self.calf_ids["RR"]] * self.calf_offset

        # compute distance between calf cylinders
        req = hppfcl.CollisionRequest()
        res = hppfcl.CollisionResult()

        T_fl = hppfcl.Transform3f(fl_calf_pose.rotation, fl_calf_pose.translation)
        T_rl = hppfcl.Transform3f(rl_calf_pose.rotation, rl_calf_pose.translation)
        T_fr = hppfcl.Transform3f(fr_calf_pose.rotation, fr_calf_pose.translation)
        T_rr = hppfcl.Transform3f(rr_calf_pose.rotation, rr_calf_pose.translation)

        col_fl_rl = hppfcl.collide(
            self.fl_cylinder_hppfcl, T_fl, self.rl_cylinder_hppfcl, T_rl, req, res
        )
        col_fl_fr = hppfcl.collide(
            self.fl_cylinder_hppfcl, T_fl, self.fr_cylinder_hppfcl, T_fr, req, res
        )
        col_fl_rr = hppfcl.collide(
            self.fl_cylinder_hppfcl, T_fl, self.rr_cylinder_hppfcl, T_rr, req, res
        )
        col_rl_fr = hppfcl.collide(
            self.rl_cylinder_hppfcl, T_rl, self.fr_cylinder_hppfcl, T_fr, req, res
        )
        col_rl_rr = hppfcl.collide(
            self.rl_cylinder_hppfcl, T_rl, self.rr_cylinder_hppfcl, T_rr, req, res
        )
        col_fr_rr = hppfcl.collide(
            self.fr_cylinder_hppfcl, T_fr, self.rr_cylinder_hppfcl, T_rr, req, res
        )
        in_collision = np.any(
            [col_fl_rl, col_fl_fr, col_fl_rr, col_rl_fr, col_rl_rr, col_fr_rr]
        )

        return in_collision
