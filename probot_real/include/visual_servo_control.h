//
// Created by bhan on 2020/5/28.
//

#ifndef VISUAL_SERVO_CONTROL_H
#define VISUAL_SERVO_CONTROL_H

#include <fstream>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <vector>
#include <eigen3/Eigen/Geometry>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"

// 定义一个类，用于处理图像信息，计算雅克比矩阵
class VisualServoControl
{
private:
    ros::NodeHandle nh_;
    moveit::planning_interface::MoveGroupInterface arm;      //需要用move group控制的机械臂中的arm group

    Eigen::Matrix<double, 8, 1> goal_feature;         // 目标图像特征
    Eigen::Matrix<double, 8, 1> curr_feature;         // 当前图像特征
    Eigen::Matrix<double , 8, 1> imageError;       // 图像特征偏差
    Eigen::Matrix<double, 8, 6> imageJacob;        // 图像雅可比矩阵
    Eigen::Matrix<double, 6, 6> trans;             // 相机坐标系和基坐标系转换矩阵
    double errorNorm;                              // 特征偏差的范数，用于决定什么时候停止控制
    double goalArea;                               // 期望面积
    double areaRatio;                              // 面积比例
    double joint6_err;                             // 第六关节控制偏差

    ros::Subscriber image_info_sub;                // 订阅当前图像特征
    ros::Publisher joint_velocity_pub;             // 发布关节速度控制信息
    ros::Publisher speed_pub;                      // 向机器人真机发送速度控制
    //ros::Publisher image_error_pub;                // 发布图像特征误差，用于绘图

    //std::ofstream outp;         // 输出流

    double depth = 0.1;                                 // 深度信息
    double lamda = 0.0007;                              // 控制比例增益

    Eigen::Matrix<double, 6, 1> cameraVelocity;      // 相机速度控制量
    Eigen::MatrixXd robotJacob;                   // 机械臂速度雅可比矩阵
    Eigen::Matrix<double, 6, 1> jointVelocity;    // 机械臂关节角控制速度
    std_msgs::Float64MultiArray speed_msg;        // 关节角控制速度（用于gazebo仿真）
    std_msgs::Float32MultiArray robot_speed_msg;          // 速度（用于真机运行）
    std_msgs::Float32MultiArray error_msg;

private:
    // 添加机械臂模型
    std::vector<double> joint_values;
    robot_model_loader::RobotModelLoader robot_model_loader;
    robot_model::RobotModelPtr kinematic_model;

public:
    bool stopControl;                 // 当偏差范数足够小时，可停止控制

public:
    explicit VisualServoControl(ros::NodeHandle n_);    // 构造函数
    void getImageJacob(const std_msgs::Float32MultiArray::ConstPtr&);       // 计算当前图像雅克比矩阵
    void getImageError();
    void getCameraVelocity();
    void getRobotJacob();
    void getJointVelocity();
    void processingStep();
    geometry_msgs::Pose getCurPose();
    void goTarget(geometry_msgs::Pose& forPose);
};

#endif
