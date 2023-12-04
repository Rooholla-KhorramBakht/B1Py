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

using namespace UNITREE_LEGGED_SDK;
UDP *high_udp;
HighCmd high_cmd = {0};
HighState high_state = {0};

rclcpp::Subscription<unitree_msgs::msg::HighCmd>::SharedPtr sub_high;

rclcpp::Publisher<unitree_msgs::msg::HighState>::SharedPtr pub_high;
rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint;

rclcpp::TimerBase::SharedPtr timer;
long high_count = 0;
long low_count = 0;

volatile uint64_t last_command_stamp = 0;
void timerCallback()
{   
    auto stamp_now = std::chrono::high_resolution_clock::now();
    uint64_t current_time = std::chrono::duration_cast<std::chrono::microseconds>(stamp_now.time_since_epoch()).count();
    uint64_t latency = current_time - last_command_stamp;
    if(latency > 0.2*1e7)
    {
        high_cmd.velocity[0] = 0.;
        high_cmd.velocity[1] = 0.;
        high_cmd.yawSpeed = 0.;
    }
    high_udp->SetSend(high_cmd);
    high_udp->Send();
    unitree_msgs::msg::HighState high_state_ros;
    high_udp->Recv();
    high_udp->GetRecv(high_state);
    high_state_ros = state2rosMsg(high_state);
    sensor_msgs::msg::Imu imu;
    sensor_msgs::msg::JointState joint_state;

    // Load the IMU message
    imu.header.stamp=rclcpp::Clock().now();
    imu.header.frame_id = "b1_imu";
    imu.orientation.w = high_state.imu.quaternion[0];
    imu.orientation.x = high_state.imu.quaternion[1];
    imu.orientation.y = high_state.imu.quaternion[2];
    imu.orientation.z = high_state.imu.quaternion[3];
    imu.linear_acceleration.x = high_state.imu.accelerometer[0];
    imu.linear_acceleration.y = high_state.imu.accelerometer[1];
    imu.linear_acceleration.z = high_state.imu.accelerometer[2];
    imu.angular_velocity.x = high_state.imu.gyroscope[0];
    imu.angular_velocity.y = high_state.imu.gyroscope[1];
    imu.angular_velocity.z = high_state.imu.gyroscope[2];
    // Load the jointstate messages
    joint_state.header.stamp = rclcpp::Clock().now();
    joint_state.header.frame_id = "b1_imu";
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
    
    for(int i=0; i<12; i++)
    {
        joint_state.position.push_back(high_state.motorState[i].q);
        joint_state.velocity.push_back(high_state.motorState[i].dq);
        joint_state.effort.push_back(high_state.motorState[i].tauEst);
    }

    pub_joint->publish(joint_state);
    pub_imu->publish(imu);
    pub_high->publish(high_state_ros);
}

void highCmdCallback(const unitree_msgs::msg::HighCmd::SharedPtr msg)
{
    // printf("highCmdCallback is running !\t%ld\n", ::high_count);
    auto stamp_now = std::chrono::high_resolution_clock::now();
    last_command_stamp = std::chrono::duration_cast<std::chrono::microseconds>(stamp_now.time_since_epoch()).count();
    high_cmd = rosMsg2Cmd(msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("b1_highlevel_node");

    timer = node->create_wall_timer(std::chrono::milliseconds(2), timerCallback);
    printf("high level runing!\n");

    high_udp = new UDP(HIGHLEVEL, 8090, "192.168.123.220", 8082);
    high_udp->InitCmdData(high_cmd);
    pub_high = node->create_publisher<unitree_msgs::msg::HighState>("/B1/high_state", 1);
    pub_imu  = node->create_publisher<sensor_msgs::msg::Imu>("/B1/imu", 1);
    pub_joint  = node->create_publisher<sensor_msgs::msg::JointState>("/B1/joint_states", 1);
    
    sub_high = node->create_subscription<unitree_msgs::msg::HighCmd>("/B1/high_cmd", 1, highCmdCallback);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}