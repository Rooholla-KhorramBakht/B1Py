from rclpy.clock import ROSClock
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStatePublisher(Node):
    def __init__(
        self,
        joint_names=[
            "FR_hip_joint",
            "FR_thigh_joint",
            "FR_calf_joint",
            "FL_hip_joint",
            "FL_thigh_joint",
            "FL_calf_joint",
            "RR_hip_joint",
            "RR_thigh_joint",
            "RR_calf_joint",
            "RL_hip_joint",
            "RL_thigh_joint",
            "RL_calf_joint",
        ],
        node_name="joint_state_publisher",
        topic_name="/B1/joint_states",
        queue_size=10,
        timer_period=0.1,
    ):
        """
        Args:
            joint_names (list): list of joint names
            node_name (str): name of the node
            topic_name (str): name of the topic to publish joint statesco
            queue_size (int): size of the message queue
            timer_period (float): period of callback execution
        """
        super().__init__(node_name)
        self.publisher_ = self.create_publisher(JointState, topic_name, queue_size)
        timer_period = timer_period  # seconds
        self.timer = self.create_timer(timer_period, self.callback)
        self.joint_names = joint_names
        self.n_joints = len(joint_names)
        self.q = [0.0] * self.n_joints
        self.dq = [0.0] * self.n_joints
        self.tau = [0.0] * self.n_joints

    def callback(self):
        joint_state = JointState()
        joint_state.header.stamp = ROSClock().now().to_msg()
        joint_state.name = self.joint_names
        joint_state.position = self.q
        joint_state.velocity = self.dq
        joint_state.effort = self.tau
        self.publisher_.publish(joint_state)

    def set_joint_states(self, q, dq=None, tau=None):
        """
        Set the joint states of the publisher.

        Args:
            q (list): joint angles
            dq (list): joint velocities
            tau (list): joint torques
        """
        self.q = q

        if dq is None:
            self.dq = [0.0] * self.n_joints
        else:
            self.dq = dq
        
        if tau is None:
            self.tau = [0.0] * self.n_joints
        else:
            self.tau = tau
