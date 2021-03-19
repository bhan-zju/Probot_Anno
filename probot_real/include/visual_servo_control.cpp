//
// Created by bhan on 2020/5/28.
//

#ifndef VISUAL_SERVO_CONTROL_CPP
#define VISUAL_SERVO_CONTROL_CPP

#include <fstream>
#include <ros/ros.h>
#include <cmath>
#include <eigen3/Eigen/Geometry>
#include <moveit/robot_model/robot_model.h>
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "visual_servo_control.h"

#define PI 3.1416

// 构造函数
VisualServoControl::VisualServoControl(ros::NodeHandle n_):
nh_(n_),
arm("manipulator"),
stopControl(false),
depth(0.1),
errorNorm(2000)
{
    // 真机：640×480(320,240)   仿真：1280×640(640,320)
    goal_feature << 555, 30, 355, 30, 355, 230, 555, 230;       //  真机抓取用
    // 390, 170, 250, 170, 250, 310, 390, 310;      // 真机，小偏差
    // 440, 120, 200, 120, 200, 360, 440, 360;       // 真机用，大偏差
    // 515, 50, 315, 50, 315, 250, 515, 250;       //  真机抓取用
    // 724, 276, 556, 276, 556, 444, 724, 444;       // 仿真用
    // 800, 200, 480, 200, 480, 520, 800, 520;       // 仿真用
    // 840, 160, 440, 160, 440, 560, 840, 560;       // 38.72 仿真用

    goalArea = pow((goal_feature(4,0)-goal_feature(0,0)) / 100, 2) + pow((goal_feature(5,0)-goal_feature(1,0)) / 100, 2);

    jointVelocity << 0, 0, 0, 0, 0, 0;
    speed_msg.data = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    robot_speed_msg.data = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    error_msg.data = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    trans << 0, -1, 0, 0, 0, 0,
            -1, 0, 0, 0, 0, 0,
            0, 0, -1, 0, 0, 0,
            0, 0,  0, 0, -1, 0,
            0, 0, 0, -1, 0, 0,
            0, 0, 0, 0, 0, -1;

    // 初始化机械臂模型
    arm.setPoseReferenceFrame("base_link");
    kinematic_model = robot_model_loader.getModel();

    //outp.open("imageFeature.txt");    // 在 /devel/lib/probot_visual_servo 下

    //开启新的线程
    //ros::AsyncSpinner spinner(1);
    //spinner.start();

    //image_error_pub = nh_.advertise<std_msgs::Float32MultiArray>("image_error",1);     // 绘图用
    joint_velocity_pub = nh_.advertise<std_msgs::Float64MultiArray>("/probot_anno/arm_vel_controller/command",1);
    speed_pub = nh_.advertise<std_msgs::Float32MultiArray>("speed_chatter", 10);
    // 订阅图像
    image_info_sub = nh_.subscribe("/local_camera/region", 5, &VisualServoControl::getImageJacob, this);
}

void VisualServoControl::getImageJacob(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    // 将当前图像特征存入 curr_feature
    for(unsigned i = 0; i < 8; ++i){
        this->curr_feature(i, 0) = msg->data.at(i);
    }

    // 记录当前特征信息，存入imageFeature.txt文本文件，用于绘图
/*
    for(unsigned i = 0; i < 8; ++i){
        outp << curr_feature(i,0) << " ";
    }
    outp << std::endl;
*/

    // 计算面积比例
    double cur_area = pow((curr_feature(4,0)-curr_feature(0,0)) / 100, 2) + pow((curr_feature(5,0)-curr_feature(1,0)) / 100, 2);
    areaRatio = cur_area / goalArea;

    joint6_err = msg->data.at(8);

    // 计算当前雅克比矩阵
    for(unsigned i = 0; i < 8; i += 2){
        Eigen::Matrix<double, 2, 6> i_feature;
        //auto x = curr_feature(i, 0) - 640, y = curr_feature(i + 1, 0) - 320;    // 仿真
        auto x = curr_feature(i, 0) - 320, y = curr_feature(i + 1, 0) - 240;    // 真机
        i_feature << -1/depth, 0, x/depth, x * y, -(1+pow(x,2)), y,
                0, -1/depth, y/depth, 1+pow(y,2), - x * y, -x;
        imageJacob.block(i, 0, 2, 6) = i_feature.block(0, 0, 2, 6);
    }

}


void VisualServoControl::getImageError()
{
    imageError = goal_feature - curr_feature;
    for(unsigned i = 0; i < 8; ++i)
        error_msg.data[i] = imageError(i,0);
    errorNorm = imageError.norm();
    //std::cout << errorNorm << std::endl;
}

void VisualServoControl::getCameraVelocity()
{
    Eigen::Matrix<double, 6, 8> imageJacob_trans = imageJacob.transpose();
    Eigen::Matrix<double, 6, 8> imageJacob_pinv = (imageJacob_trans * imageJacob).inverse() * imageJacob_trans;
    cameraVelocity = lamda * imageJacob_pinv * imageError;
    // 单独计算深度方向的线速度，加入到相机速度中
    cameraVelocity(2,0) += 0.008 * (1-areaRatio);
}

void VisualServoControl::getRobotJacob()
{
    //建立运动学对象
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("manipulator");
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
    //读取当前的位置信息，刷新关节角
    kinematic_state = arm.getCurrentState();
    //读出起点位置关节角
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    robotJacob = kinematic_state->getJacobian(joint_model_group);
}

void VisualServoControl::getJointVelocity()
{
    if(errorNorm < 25){               // 当偏差足够小时，停止控制
        speed_msg.data = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        stopControl = true;
        return;
    }
    Eigen::Matrix<double, 6, 6> robotJacob_inv = robotJacob.inverse();
    jointVelocity = robotJacob_inv * (trans * cameraVelocity);

    for(size_t i = 0; i < 5; ++i){
        if(jointVelocity(i, 0) < 20 && jointVelocity(i, 0) > -20)
            speed_msg.data.at(i) = jointVelocity(i, 0);
        else
            speed_msg.data.at(i) = 0;
    }
    speed_msg.data.at(5) = 0.01 * joint6_err;

    // 如仅在gazebo中仿真，可注释掉下边这几句

    robot_speed_msg.data.at(0) = speed_msg.data.at(0) * 30 * 180 / PI;
    robot_speed_msg.data.at(1) = speed_msg.data.at(1) * 205 * 180 / (3 * PI);
    robot_speed_msg.data.at(2) = speed_msg.data.at(2) * 50 * 180 / PI;
    robot_speed_msg.data.at(3) = speed_msg.data.at(3) * 125 * 180 / (2 * PI);
    robot_speed_msg.data.at(4) = speed_msg.data.at(4) * 125 * 180 / (2 * PI);
    robot_speed_msg.data.at(5) = speed_msg.data.at(5) * 200 * 180 / (9 * PI);

}

void VisualServoControl::processingStep()
{
    // 开启一个新的线程
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 显示当前特征
    //ROS_INFO_STREAM(curr_feature);

    // 显示雅克比矩阵
    //ROS_INFO_STREAM(imageJacob);

    // 计算图像偏差
    getImageError();
    //image_error_pub.publish(error_msg);     // 用于画图
    //ROS_INFO_STREAM(imageError);

    // 计算相机控制速度
    getCameraVelocity();
    //ROS_INFO_STREAM(cameraVelocity);

    // 计算机械臂速度雅可比矩阵
    getRobotJacob();
    //ROS_INFO_STREAM(robotJacob);

    // 计算关节角速度
    getJointVelocity();
    //ROS_INFO_STREAM(jointVelocity);
    //ROS_INFO_STREAM(speed_msg);

    // 发布速度信息
    joint_velocity_pub.publish(speed_msg);    // gazebo
    speed_pub.publish(robot_speed_msg);     // 真机
}

geometry_msgs::Pose VisualServoControl::getCurPose() {
    return arm.getCurrentPose().pose;
}

void VisualServoControl::goTarget(geometry_msgs::Pose& forPose) {
    //当运动规划失败后，允许重新规划
    arm.allowReplanning(true);
    // 设置机器臂当前的状态作为运动初始状态
    arm.setStartStateToCurrentState();

    arm.setPoseTarget(forPose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    //进行运动规划，成功则返回true
    moveit::planning_interface::MoveItErrorCode success = arm.plan(plan);
    ROS_INFO("Plan (pose goal) %s",success?"":"FAILED");

    //在rviz中让机械臂按照规划的轨迹开始运动
    if(success)
        arm.execute(plan);

    double time_start;
    double time_end;
    double time_duration;

    //开始计时
    for(int num = 0; num < plan.trajectory_.joint_trajectory.points.size(); num++){
        // 向gazebo发送控制信号
        speed_msg.data = plan.trajectory_.joint_trajectory.points[num].velocities;
        joint_velocity_pub.publish(speed_msg);
        // 真机
        robot_speed_msg.data.at(0) = speed_msg.data.at(0) * 30 * 180 / PI;
        robot_speed_msg.data.at(1) = speed_msg.data.at(1) * 205 * 180 / (3 * PI);
        robot_speed_msg.data.at(2) = speed_msg.data.at(2) * 50 * 180 / PI;
        robot_speed_msg.data.at(3) = speed_msg.data.at(3) * 125 * 180 / (2 * PI);
        robot_speed_msg.data.at(4) = speed_msg.data.at(4) * 125 * 180 / (2 * PI);
        robot_speed_msg.data.at(5) = speed_msg.data.at(5) * 200 * 180 / (9 * PI);
        speed_pub.publish(robot_speed_msg);

        time_start = plan.trajectory_.joint_trajectory.points[num].time_from_start.toSec();
        if(num < plan.trajectory_.joint_trajectory.points.size()-1){
            time_end = plan.trajectory_.joint_trajectory.points[num+1].time_from_start.toSec();
            time_duration = (time_end - time_start) * 1000000;     //假定每个循环的时间花费250微秒
        }else{
            time_duration = 10000;
        }

        usleep(time_duration);      //微秒
    }

}

#endif
