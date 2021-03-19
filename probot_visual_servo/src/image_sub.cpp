#include <ros/ros.h>
#include <iostream>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"

void poseCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    // 将接收到的消息打印出来
    for(int i = 0; i < 9; ++i)
    	std::cout << msg->data.at(i) << " ";
    std::cout << std::endl;
}

int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "image_sub");

    // 创建节点句柄
    ros::NodeHandle n;

    // 创建一个Subscriber
    ros::Subscriber pose_sub = n.subscribe("/local_camera/region", 10, poseCallback);

    /*
    ros::Publisher joint_velocity_pub  = n.advertise<std_msgs::Float64MultiArray>("/probot_anno/arm_vel_controller/command",1);  // 发布关节速度控制信息
    ros::Publisher speed_pub = n.advertise<std_msgs::Float32MultiArray>("speed_chatter", 10);     // 向机器人真机发送速度控制
    std_msgs::Float64MultiArray speed_msg;        // 关节角控制速度（用于gazebo仿真）
    std_msgs::Float32MultiArray robot_speed_msg;          // 速度（用于真机运行）
    speed_msg.data = { 0.08, 0.0, 0.0, 0.0, 0.0, 0.0};
    robot_speed_msg.data = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    robot_speed_msg.data.at(0) = speed_msg.data.at(4) * 125 * 180 / (2 * 3.1416);
    while (ros::ok())
    {
        ros::spinOnce();
        joint_velocity_pub.publish(speed_msg);
        speed_pub.publish(robot_speed_msg);
    }
    */
    // 循环等待回调函数
    ros::spin();

    return 0;
}
