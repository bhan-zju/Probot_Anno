/***********************************************************************
Copyright 2019 Wuhan PS-Micro Technology Co., Itd.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>   //C++接口

#define PI 3.14159265

int main(int argc, char **argv)
{
    //初始化节点，节点名为moveit_fk_demo
    ros::init(argc, argv, "moveit_fk_demo");

    //多线程
    ros::AsyncSpinner spinner(1);

    //开启新的线程
    spinner.start();

    //初始化需要使用move group控制的arm group
    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    //设置机械臂的允许误差值（单位：弧度）(这一步也可以不设置，不影响运行)
    arm.setGoalJointTolerance(0.001);

    //设置最大加速度和最大速度(这一步也可以不设置，不影响运行)
    arm.setMaxAccelerationScalingFactor(0.5);
    arm.setMaxVelocityScalingFactor(0.5);

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

    //定义一个数组，存放6个关节信息
    double targetPose[6] = {0.391410, -0.676384, -0.376217, 0.2, 1.052834, 0.454125};
    //double targetPose[6] = {0, 0, PI/2, 0, 0, 0};

    //关节向量赋值
    std::vector<double> joint_group_positions(6);
    joint_group_positions[0] = targetPose[0];
    joint_group_positions[1] = targetPose[1];
    joint_group_positions[2] = targetPose[2];
    joint_group_positions[3] = targetPose[3];
    joint_group_positions[4] = targetPose[4];
    joint_group_positions[5] = targetPose[5];


    //将关节向量写入
    arm.setJointValueTarget(joint_group_positions);
    arm.move();   //规划、控制　　　
    sleep(1);

    //显示机械臂末端位姿
    ROS_INFO_STREAM(arm.getCurrentPose().pose.position.x);
    ROS_INFO_STREAM(arm.getCurrentPose().pose.position.y);
    ROS_INFO_STREAM(arm.getCurrentPose().pose.position.z);
    ROS_INFO_STREAM(arm.getCurrentPose().pose.orientation.x);
    ROS_INFO_STREAM(arm.getCurrentPose().pose.orientation.y);
    ROS_INFO_STREAM(arm.getCurrentPose().pose.orientation.z);
    ROS_INFO_STREAM(arm.getCurrentPose().pose.orientation.w);


/*   //也可以这样写
    arm.setJointValueTarget(joint_group_positions);
     // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    //进行运动规划，成功则返回true
    moveit::planning_interface::MoveItErrorCode success = arm.plan(plan);

    //让机械臂按照规划的轨迹开始运动。
    if(success)
      arm.execute(plan);
    sleep(1);
*/

    // 控制机械臂重新回到初始化位置
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

    //关闭并退出
    ros::shutdown(); 

    return 0;
}
