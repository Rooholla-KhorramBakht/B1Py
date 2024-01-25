#include "rclcpp/rclcpp.hpp"
#include "unitree_msgs/msg/high_cmd.hpp"
#include "unitree_msgs/msg/high_state.hpp"
#include "unitree_msgs/msg/low_cmd.hpp"
#include "unitree_msgs/msg/low_state.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"

#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
using namespace UNITREE_LEGGED_SDK;

class Custom: public rclcpp::Node
{
public:
    Custom() : 
                            Node("b1_highlevel_node"),
                            safe(LeggedType::B1),
                            udp(HIGHLEVEL, 8090, "192.168.123.220", 8082)
    {
        udp.InitCmdData(cmd);
        // udp.print = true;
        pub_high = this->create_publisher<unitree_msgs::msg::HighState>("/b1/high_state", 1);
        pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("/b1/odom", 1);
        pub_imu  = this->create_publisher<sensor_msgs::msg::Imu>("/b1/imu", 1);
        pub_joint  = this->create_publisher<sensor_msgs::msg::JointState>("/b1/joint_states", 1);
        sub_high = this->create_subscription<unitree_msgs::msg::HighCmd>("/b1/high_cmd", 1, std::bind(&Custom::highCmdCallback, this, std::placeholders::_1));
        sub_twist = this->create_subscription<geometry_msgs::msg::TwistStamped>("/b1/twist_cmd", 1, std::bind(&Custom::twistCmdCallback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(2ms, std::bind(&Custom::RobotControl, this));
        udp_send_timer_ = this->create_wall_timer(3ms, std::bind(&Custom::UDPSend, this));
        udp_receive_timer_ = this->create_wall_timer(3ms, std::bind(&Custom::UDPRecv, this));
        
    }
    void UDPRecv();
    void UDPSend();
    void RobotControl();
    void highCmdCallback(const unitree_msgs::msg::HighCmd::SharedPtr msg);
    void twistCmdCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    Safety safe;
    UDP udp;
    HighCmd cmd = {0};
    HighState state = {0};
    int motiontime = 0;
    int reset = 0; 
    float dt = 0.002; // 0.001~0.01
    rclcpp::Subscription<unitree_msgs::msg::HighCmd>::SharedPtr sub_high;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist;
    rclcpp::Publisher<unitree_msgs::msg::HighState>::SharedPtr pub_high;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
    geometry_msgs::msg::TwistStamped twist_cmd;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr udp_send_timer_;
    rclcpp::TimerBase::SharedPtr udp_receive_timer_;
    uint64_t last_command_stamp = 0;
    unitree_msgs::msg::HighState high_state_ros;
};

void Custom::UDPRecv()
{
    // udp.Recv();
}

void Custom::UDPSend()
{
    // udp.Send();
}

void Custom::RobotControl()
{
    udp.Recv();
    udp.GetRecv(state);
    motiontime += 2;

    high_state_ros = state2rosMsg(state);
    sensor_msgs::msg::Imu imu;
    sensor_msgs::msg::JointState joint_state;
    nav_msgs::msg::Odometry odom_state;
    // printf("%d   %f\n", motiontime, state.imu.rpy[2]);

    // Load the IMU message
    imu.header.stamp=rclcpp::Clock().now();
    imu.header.frame_id = "b1_imu_link";
    imu.orientation.w = state.imu.quaternion[0];
    imu.orientation.x = state.imu.quaternion[1];
    imu.orientation.y = state.imu.quaternion[2];
    imu.orientation.z = state.imu.quaternion[3];
    imu.linear_acceleration.x = state.imu.accelerometer[0];
    imu.linear_acceleration.y = state.imu.accelerometer[1];
    imu.linear_acceleration.z = state.imu.accelerometer[2];
    imu.angular_velocity.x = state.imu.gyroscope[0];
    imu.angular_velocity.y = state.imu.gyroscope[1];
    imu.angular_velocity.z = state.imu.gyroscope[2];
    // Load the joint state messages
    joint_state.header.stamp = imu.header.stamp;
    joint_state.header.frame_id = "b1_imu_link";
    joint_state.name.push_back("FR_hip_joint");
    joint_state.name.push_back("FR_thigh_joint");
    joint_state.name.push_back("FR_calf_joint");

    joint_state.name.push_back("FL_hip_joint");
    joint_state.name.push_back("FL_thigh_joint");
    joint_state.name.push_back("FL_calf_joint");

    joint_state.name.push_back("RR_hip_joint");
    joint_state.name.push_back("RR_thigh_joint");
    joint_state.name.push_back("RR_calf_joint");

    joint_state.name.push_back("RL_hip_joint");
    joint_state.name.push_back("RL_thigh_joint");
    joint_state.name.push_back("RL_calf_joint");

    odom_state.header.stamp = imu.header.stamp;
    odom_state.header.frame_id = "odom";
    odom_state.child_frame_id = "base_link";

    // odometry states published by the onboard high-level controller
    odom_state.pose.pose.position.x = state.position[0];
    odom_state.pose.pose.position.y = state.position[1];
    odom_state.pose.pose.position.z = state.position[2];


    odom_state.twist.twist.linear.x = state.velocity[0];
    odom_state.twist.twist.linear.y = state.velocity[1];
    odom_state.twist.twist.linear.z = state.velocity[2];
    odom_state.twist.twist.angular.z= state.yawSpeed;
    double position_R = 0.05;
    double orientation_R = 0.005;
    odom_state.pose.covariance = {
        position_R, 0, 0, 0, 0, 0,  // Covariance for position x
        0, position_R, 0, 0, 0, 0,  // Covariance for position y
        0, 0, position_R, 0, 0, 0,  // Covariance for position z
        0, 0, 0, orientation_R, 0, 0,  // Covariance for orientation roll
        0, 0, 0, 0, orientation_R, 0,  // Covariance for orientation pitch
        0, 0, 0, 0, 0, orientation_R   // Covariance for orientation yaw    
    };

    odom_state.twist.covariance = {
        position_R, 0, 0, 0, 0, 0,  // Covariance for position x
        0, position_R, 0, 0, 0, 0,  // Covariance for position y
        0, 0, position_R, 0, 0, 0,  // Covariance for position z
        0, 0, 0, orientation_R, 0, 0,  // Covariance for orientation roll
        0, 0, 0, 0, orientation_R, 0,  // Covariance for orientation pitch
        0, 0, 0, 0, 0, orientation_R   // Covariance for orientation yaw    
    };
    
    for(int i=0; i<12; i++)
    {
        joint_state.position.push_back(state.motorState[i].q);
        joint_state.velocity.push_back(state.motorState[i].dq);
        joint_state.effort.push_back(state.motorState[i].tauEst);
    }
    pub_joint->publish(joint_state);
    pub_imu->publish(imu);
    pub_high->publish(high_state_ros); 
    pub_odom->publish(odom_state);

    auto stamp_now = std::chrono::high_resolution_clock::now();
    uint64_t current_time = std::chrono::duration_cast<std::chrono::microseconds>(stamp_now.time_since_epoch()).count();
    uint64_t latency = current_time - last_command_stamp;
    if(latency > 0.2*1e7 && reset==0)
    {   
        reset = 1;
        cmd = {0};
        udp.InitCmdData(cmd);
    }
    udp.SetSend(cmd);
    udp.Send();
}

void Custom::highCmdCallback(const unitree_msgs::msg::HighCmd::SharedPtr msg)
{
    auto stamp_now = std::chrono::high_resolution_clock::now();
    last_command_stamp = std::chrono::duration_cast<std::chrono::microseconds>(stamp_now.time_since_epoch()).count();
    for (int i(0); i < 2; i++)
    {
        cmd.velocity[i] = msg->velocity[i];
    }
    cmd.yawSpeed = msg->yaw_speed;
    for (int i(0); i < 3; i++)
    {
        cmd.euler[i] = msg->euler[i];
    }
    cmd.mode = msg->mode;
    cmd.footRaiseHeight = msg->foot_raise_height;
    cmd.bodyHeight = msg->body_height;
    cmd.yawSpeed = msg->yaw_speed;
    reset = 0; 
}

void Custom::twistCmdCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    auto stamp_now = std::chrono::high_resolution_clock::now();
    last_command_stamp = std::chrono::duration_cast<std::chrono::microseconds>(stamp_now.time_since_epoch()).count();
    cmd.velocity[0] = msg->twist.linear.x;
    cmd.velocity[1] = msg->twist.linear.y;
    cmd.yawSpeed = msg->twist.angular.z;
    reset = 0; 
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    printf("high level runing!\n");
    rclcpp::spin(std::make_shared<Custom>());
    rclcpp::shutdown();
    return 0;
}
