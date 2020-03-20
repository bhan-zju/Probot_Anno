//
// Created by bhan on 2019/12/26.
//
#include <string>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "ikfast.h"
#include "probot_anno_manipulator_ikfast_moveit_plugin.cpp"
#include <urdf/model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <time.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include <sstream>

#define pi 3.1416

bool Feedback = 1;
using namespace ikfast_kinematics_plugin;
void Feed_back(const std_msgs::String::ConstPtr& Feedback_msg)
{
    Feedback = 0;
}
int main(int argc, char **argv) {

    bool ret;
    ros::init(argc, argv, "trajectory");
    ros::NodeHandle node_handle;

    //创建一个发布者Float_chatter_pub，发布消息类型为std_msgs::Float32MultiArray，话题名为Float_chatter，用于发布速度
    ros::Publisher speed_pub = node_handle.advertise<std_msgs::Float32MultiArray>("speed_chatter", 1000);
    //创建一个发布者Float64_chatter_pub，发布消息类型为std_msgs::Float64MultiArray，话题名为Float64_chatter，用于发布位置
    ros::Publisher position_pub = node_handle.advertise<std_msgs::Float32MultiArray>("position_chatter", 1000);
    //创建一个订阅者Feedback_sub，接收话题"Feedback_chatter"，回调函数为Feed_back
    ros::Subscriber Feedback_sub = node_handle.subscribe("Feedback_chatter",100,Feed_back);
    //ros::Publisher reset_chatter_pub = node_handle.advertise<std_msgs::String>("reset_chatter", 1000);
    ros::AsyncSpinner spinner(1);


    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    //设置机械臂的允许误差值（单位：弧度）(这一步也可以不设置，不影响运行)
    arm.setGoalJointTolerance(0.001);

    //设置最大加速度和最大速度(这一步也可以不设置，不影响运行)
    arm.setMaxAccelerationScalingFactor(0.5);
    arm.setMaxVelocityScalingFactor(0.5);

    //设置目标位置所使用的参考坐标系
    std::string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);

    //当运动规划失败后，允许重新规划
    arm.allowReplanning(true);

    // rviz中控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

    //读出初始位置末端的位姿
    geometry_msgs::PoseStamped pose_solved;
    pose_solved = arm.getCurrentPose("tool0");       //arm.getCurrentPose()返回geometry_msgs::PoseStamped类型的变量
    ROS_INFO_STREAM("start: " << pose_solved);          //输出pose_solved的所有信息

    std_msgs::Float32MultiArray speed_msg;    //速度
    speed_msg.data.push_back(0.0);
    speed_msg.data.push_back(0.0);
    speed_msg.data.push_back(0.0);
    speed_msg.data.push_back(0.0);
    speed_msg.data.push_back(0.0);
    speed_msg.data.push_back(0.0);

    std_msgs::Float32MultiArray position_msg;   //位置
    position_msg.data.push_back(0.0);
    position_msg.data.push_back(0.0);
    position_msg.data.push_back(0.0);
    position_msg.data.push_back(0.0);
    position_msg.data.push_back(0.0);
    position_msg.data.push_back(0.0);
    position_msg.data.push_back(0.0);


    position_pub.publish(position_msg);     //将初始化位置发送出去，用于控制器订阅
    sleep(5);

    // 设置机器人终端的目标位置
    geometry_msgs::Pose target_pose;
    target_pose.position.x = 0.2593;
    target_pose.position.y = -0.1636;
    target_pose.position.z = 0.1787;
    target_pose.orientation.x = 1.0;

    // 设置机器臂当前的状态作为运动初始状态
    arm.setStartStateToCurrentState();

    arm.setPoseTarget(target_pose);


    moveit::planning_interface::MoveGroupInterface::Plan plan;
    //进行运动规划，成功则返回true
    moveit::planning_interface::MoveItErrorCode success = arm.plan(plan);
    ROS_INFO("Plan (pose goal) %s",success?"":"FAILED");

    //在rviz中让机械臂按照规划的轨迹开始运动
    if(success)
        arm.execute(plan);
    sleep(1);
    

    //std::vector<double> velocity(6);
    //std::vector<double> joint_group_positions(6);

//    int point_num = plan.trajectory_.joint_trajectory.points.size();
    double time_start;
    double time_end;
    double time_duration;

    //开始计时
    clock_t start = clock();

    for(int n = 0; n < plan.trajectory_.joint_trajectory.points.size(); n++){

        //clock_t startn = clock();
        speed_msg.data.at(0) = plan.trajectory_.joint_trajectory.points[n].velocities[0] * 30 * 180 / pi;
        speed_msg.data.at(1) = plan.trajectory_.joint_trajectory.points[n].velocities[1] * 205 * 180 / (3 * pi);
        speed_msg.data.at(2) = plan.trajectory_.joint_trajectory.points[n].velocities[2] * 50 * 180 / pi;
        speed_msg.data.at(3) = plan.trajectory_.joint_trajectory.points[n].velocities[3] * 125 * 180 / (2 * pi);
        speed_msg.data.at(4) = plan.trajectory_.joint_trajectory.points[n].velocities[4] * 125 * 180 / (2 * pi);
        speed_msg.data.at(5) = plan.trajectory_.joint_trajectory.points[n].velocities[5] * 200 * 180 / (9 * pi);
        speed_pub.publish(speed_msg);

        time_start = plan.trajectory_.joint_trajectory.points[n].time_from_start.toSec();
        if(n < plan.trajectory_.joint_trajectory.points.size()-1){
            time_end = plan.trajectory_.joint_trajectory.points[n+1].time_from_start.toSec();
            time_duration = (time_end - time_start) * 1000000 - 250;     //假定每个循环的时间花费250微秒
        }else{
            time_duration = 10000;
        }

        usleep(time_duration);      //微秒

        //clock_t endsn = clock();
        //std::cout << "Running Time n : " << (double) 1000 * (endsn - startn) / CLOCKS_PER_SEC << std::endl;

    }


    clock_t ends = clock();
    std::cout << "Running Time : " << (double) 1000 * (ends - start) / CLOCKS_PER_SEC << std::endl;

    sleep(5);

    pose_solved = arm.getCurrentPose("tool0");//get end position
    ROS_INFO_STREAM("END: " << pose_solved);

}

