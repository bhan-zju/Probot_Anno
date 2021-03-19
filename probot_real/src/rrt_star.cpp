//
//  rrt_star.cpp
//  采用 RRT* 路径规划算法 为机械臂规划一条无碰路径
//  Created by 韩奔 on 2020/3/1.
//

#include <ros/ros.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include "Manipulator.cpp"
#include "Obstacle.h"
#include "rrtTree.cpp"

#define PI 3.1416

int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "class_test");
    ros::NodeHandle n;

    rrtTree pathPlan(n);
    // 添加障碍物
    Obstacle ball1;
    ball1.radius = 0.12;
    ball1.position << 0.22, 0.20, 0.30;
    pathPlan.addObstacle(&ball1);
    Obstacle ball2;
    ball2.radius = 0.15;
    ball2.position << -0.30, 0.15, 0.50;
    pathPlan.addObstacle(&ball2);
    /*Obstacle ball3;
    ball3.radius = 0.13;
    ball3.position << 0.225, -0.075, 0.185;
    pathPlan.addObstacle(&ball3);
    Obstacle ball4;
    ball4.radius = 0.13;
    ball4.position << 0.375, -0.075, 0.185;
    pathPlan.addObstacle(&ball4);
    */

    // 设置一个目标位姿
    geometry_msgs::Pose target_pose;
    target_pose.position.x = 0.00;
    target_pose.position.y = 0.32;
    target_pose.position.z = 0.20;
    target_pose.orientation.x = 1;
    target_pose.orientation.y = sqrt(1-pow(0.707,2));
    pathPlan.setTargetPose(target_pose);

    /*
    Eigen::Matrix<double, 6, 1> root_angle;
    root_angle << 1.4327, -0.77266, 0.48143, 0, 0.29123, -0.1384;
    pathPlan.setRootNode(root_angle);
    */

    // 开始规划
    pathPlan.pathPlanning();
    // 开始控制
    pathPlan.robotControl();

    return 0;
}