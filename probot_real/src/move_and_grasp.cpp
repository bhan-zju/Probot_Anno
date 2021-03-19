//
//  rrt_star.cpp
//  路径规划与抓取
//  Created by 韩奔 on 2021/3/1.
//

#include <ros/ros.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include "std_msgs/Bool.h"
#include "Manipulator.cpp"
#include "Obstacle.h"
#include "rrtTree.cpp"
#include "visual_servo_control.h"
#include "visual_servo_control.cpp"

#define PI 3.1416

int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "class_test");
    ros::NodeHandle nh;

    ros::Publisher gripper_pub = nh.advertise<std_msgs::Bool>("gripper_chatter",10);
    std_msgs::Bool gripper_signal;     // 夹爪
    gripper_signal.data = 1;
    gripper_pub.publish(gripper_signal);

    rrtTree pathPlan1(nh);
    // 添加障碍物
    Obstacle ball3;
    ball3.radius = 0.15;
    ball3.position << 0.225, 0.075, 0.185;
    pathPlan1.addObstacle(&ball3);
    Obstacle ball4;
    ball4.radius = 0.15;
    ball4.position << 0.225, -0.075, 0.185;
    pathPlan1.addObstacle(&ball4);
    // 设置第一个目标位姿
    geometry_msgs::Pose target_pose;
    target_pose.position.x = 0.052;
    target_pose.position.y = 0.373;
    target_pose.position.z = 0.242;
    target_pose.orientation.x = 0.707;
    target_pose.orientation.y = sqrt(1-pow(0.707,2));
    pathPlan1.setTargetPose(target_pose);
    // 开始规划
    pathPlan1.pathPlanning();
    // 开始控制
    pathPlan1.robotControl();
    std::cout << "Successfully First Path Planning and Control\n" << std::endl;

    sleep(3);


    // 视觉伺服控制
    std::cout << "Visual Servo Control Start\n" << std::endl;
    VisualServoControl vsc(nh);
    ros::spinOnce();
    //ros::AsyncSpinner spinner(1);
    //spinner.start();
    while (ros::ok() && (!vsc.stopControl))
    {
        vsc.processingStep();
    }
    std::cout << "Successfully Visual Servo Control\n" << std::endl;
    sleep(1);

    // 控制抓取
    ros::AsyncSpinner spinner2(1);
    spinner2.start();
    geometry_msgs::Pose pose1 = vsc.getCurPose();
    pose1.position.z -= 0.13;
    pose1.position.y -= 0.02;
    vsc.goTarget(pose1);
    // 抓
    std::cout << "Grasp Start!\n" << std::endl;
    gripper_signal.data = 0;
    gripper_pub.publish(gripper_signal);
    sleep(5);

    geometry_msgs::Pose pose2 = vsc.getCurPose();
    pose2.position.z += 0.1;
    vsc.goTarget(pose2);
    spinner2.stop();


    // 第二段路径规划
    rrtTree pathPlan2(nh);
    // 添加障碍物
    pathPlan2.addObstacle(&ball3);
    pathPlan2.addObstacle(&ball4);
    // 设置初始位姿
    Eigen::Matrix<double, 6, 1> new_root;
    new_root << 1.4327, -0.8868, 0.3995, 0, 0.4873, -0.1388;
    pathPlan2.setRootNode(new_root);
    // 设置目标位姿
    geometry_msgs::Pose target_pose2;
    target_pose2.position.x = 0.10;
    target_pose2.position.y = -0.35;
    target_pose2.position.z = 0.20;
    target_pose2.orientation.x = 0.707;
    target_pose2.orientation.y = -sqrt(1-pow(0.707,2));
    pathPlan2.setTargetPose(target_pose2);
    // 进行路径规划与控制
    pathPlan2.pathPlanning();
    pathPlan2.robotControl();
    std::cout << "Successfully Second Path Planning and Control\n" << std::endl;


    // 放置物体
    gripper_signal.data = 1;
    gripper_pub.publish(gripper_signal);

    return 0;

}