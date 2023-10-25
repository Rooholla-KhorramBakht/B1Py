import gtsam
import numpy as np
from gtsam.symbol_shorthand import X


class SE3ExtrinsicManager:
    """
    A class that manages the relative transformations (SE3) between a suite of sensors using GTSAM.
    """

    def __init__(self):
        """
        Initializes an SE3ExtrinsicManager object.
        """
        self.reset()

    def reset(self):
        """
        Resets the internal state of the manager.
        """
        self.graph = gtsam.NonlinearFactorGraph()  # Graph for optimization
        self.values = gtsam.Values()  # Values in the graph
        self.name_to_id = {}  # Map of sensor name to ID
        self.id_to_name = {}  # Map of ID to sensor name
        self.latest_assigned_id = -1  # Latest ID assigned to a sensor
        self.extrinsics = None  # Optimized extrinsics

    def add(self, parent, child, R, t, confidence=1):
        """
        Add a relative transformation (SE3) between parent and child sensors ${}^{parent} T_{child}$.

        @param parent: Name of the parent sensor.
        @param child: Name of the child sensor.
        @param R: Rotation matrix (3x3) or quaternion (4x1, 1x4, 4,) [qw, qx, qy, qz].
        @param t: Translation vector [x, y, z].
        @param confidence: Confidence level for the measurement. Defaults to 1.
        """
        # Handle the case where R is a quaternion
        if R.shape == (4, 1) or R.shape == (1, 4) or R.shape == (4,):
            qw = R[0]
            qx = R[1]
            qy = R[2]
            qz = R[3]
            R = gtsam.Rot3(qw, qx, qy, qz).matrix()

        # Construct a full transformation matrix
        T = np.vstack([np.hstack([R, t.reshape(3, 1)]), np.array([0, 0, 0, 1])])

        # Handle new sensors
        if parent not in self.name_to_id.keys():
            self.latest_assigned_id += 1
            self.id_to_name[self.latest_assigned_id] = parent
            self.name_to_id[parent] = self.latest_assigned_id
            self.values.insert(X(self.name_to_id[parent]), gtsam.Pose3())

        if child not in self.name_to_id.keys():
            self.latest_assigned_id += 1
            self.id_to_name[self.latest_assigned_id] = child
            self.name_to_id[child] = self.latest_assigned_id
            self.values.insert(X(self.name_to_id[child]), gtsam.Pose3())

        # Add measurement to the graph
        Sigma = gtsam.noiseModel.Diagonal.Sigmas([confidence for _ in range(6)])
        self.graph.add(
            gtsam.BetweenFactorPose3(
                X(self.name_to_id[parent]),
                X(self.name_to_id[child]),
                gtsam.Pose3(T),
                Sigma,
            )
        )
        # Optimize the graph
        params = gtsam.LevenbergMarquardtParams()
        optimizer = gtsam.LevenbergMarquardtOptimizer(self.graph, self.values, params)
        self.extrinsics = optimizer.optimize()

    def get(self, parent, child):
        """
        Get the relative transformation from the parent sensor to the child sensor.

        @param parent: Name of the parent sensor.
        @param child: Name of the child sensor.
        @return: A 4x4 matrix representing the relative transformation ${}^{reference} T_{child}$.
        """
        T_parent = self.extrinsics.atPose3(X(self.name_to_id[parent])).matrix()
        T_child = self.extrinsics.atPose3(X(self.name_to_id[child])).matrix()
        return np.linalg.inv(T_parent) @ T_child

    def get_all(self, reference):
        """
        Get transformations of all sensors relative to a given reference sensor.

        @param reference: Name of the reference sensor.
        @return: A dictionary with keys as "<sensor_name>_wrt_<reference>" and values as 4x4 transformation matrices.
        """
        assert (
            reference in self.name_to_id.keys()
        ), "Selected reference frame is not in the graph"
        return {
            f"{key}_wrt_{reference}": self.get(reference, key)
            for key in self.name_to_id.keys()
            if key != reference
        }

    def pose2qt(self, T):
        """
        Convert a 4x4 transformation matrix to quaternion and translation vector.

        @param T: A 4x4 transformation matrix.
        @return: A 4x1 quaternion.
        """
        return gtsam.Rot3(T[:3, :3]).quaternion(), T[0:3, 3]


def parseVicon2GtParams(params_file):
    """
    Read the transformation matrix representing the pose of the vicon marker frame with respect to
    the robot IMU frame (B1 body frame) and return as a 4x4 numpy array.
    """
    # Initialize variables to None
    R_BtoI = None
    p_BinI = None

    # Open the file and read line by line
    with open(params_file) as f:
        lines = f.readlines()

    reading_R = False
    reading_p = False
    R_rows = []
    p_values = []

    for line in lines:
        # Check if the line starts reading R_BtoI matrix or p_BinI vector
        if "R_BtoI:" in line:
            reading_R = True
            continue
        elif "p_BinI:" in line:
            reading_p = True
            continue

        # Read the values for R_BtoI
        if reading_R:
            if line.strip():  # not an empty line
                R_rows.append(list(map(float, line.split())))
            else:
                reading_R = False
                R_BtoI = np.array(R_rows)

        # Read the values for p_BinI
        if reading_p:
            if line.strip():  # not an empty line
                p_values.append(float(line.strip()))
            else:
                reading_p = False
                p_BinI = np.array(p_values)

    # Create the 4x4 transformation matrix
    T_BtoI = np.identity(4)
    T_BtoI[0:3, 0:3] = R_BtoI
    T_BtoI[0:3, 3] = p_BinI

    return T_BtoI
