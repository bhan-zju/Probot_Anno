// 实现每个关节角的速度控制
// Created by kjwang on 2019/11/18.
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
#include "std_msgs/Float64MultiArray.h"
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
    ros::init(argc, argv, "ik");
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
    
    std::vector<double> joint_values;
    moveit::planning_interface::MoveGroupInterface arm("manipulator");
    
    //设置目标位置所使用的参考坐标系
    std::string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);

    //载入模型        不解。为什么是"robot_description"
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();   //运动学模型kinematic_model
    ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

    //建立运动学对象    robot_state::RobotStatePtr类？？？
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const robot_state::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("manipulator");
    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

    //读取当前的位置信息，刷新关节角
    kinematic_state = arm.getCurrentState();
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);   //Vector3d 实质上是 Eigen::Matrix<double, 3, 1>
    Eigen::MatrixXd jacobian;//雅克比矩阵
    Eigen::MatrixXd jacobian_inv;//雅克比矩阵的逆
    Eigen::Matrix<double, 6, 1> speedl;//6维线速度    xyz线速度和zyx欧拉角速度
    Eigen::Matrix<double, 6, 1> speedj;//6关节角速度，要发到控制器的就是这6个浮点数
    Eigen::Matrix<double, 6, 1> accel1;//加速度
    Eigen::Matrix<double, 6, 1> accel2;//减速时的加速度

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

    //std_msgs::String reset;
    //reset.data = "ok";
    //reset_chatter_pub.publish(reset);
    position_pub.publish(position_msg);     //将初始化位置发送出去，用于控制器订阅
    sleep(10);

    //while(Feedback);
    Feedback = 1;

    double del_t = 0.01;//加速的时间间隔delta a，单位：s
    int n = 0;//第几个
    speedl << 0, 0, 0, 0, 0, 0;        //末端线速度    zyx
    speedj << 0, 0, 0, 0, 0, 0;        //6关节角速度
    accel1 << 0.003536, 0.003536, 0, 0, 0, 0;       // 加速速度矢量 标量为0.005m/s^2
    accel2 << -0.002829, -0.002829, 0, 0, 0, 0;     // 减速度矢量 标量为0.004m/s^2   这两个加速度都是末端的笛卡尔加速度

    double targetPose[6] = {45, -28.8467, -34.2441, 0.0, 63.0908, 45};  //运动的起始位置
    //这里是.data.at()而不是.data.push_back，是修改容器中该位置的数据，而不是往容器中堆放数据
    position_msg.data.at(0) = targetPose[0] * 30;
    position_msg.data.at(1) = targetPose[1] * 205 / 3;
    position_msg.data.at(2) = targetPose[2] * 50;
    position_msg.data.at(3) = targetPose[3] * 125 / 2;
    position_msg.data.at(4) = targetPose[4] * 125 / 2;
    position_msg.data.at(5) = targetPose[5] * 200 / 9;
    position_msg.data.at(6) = 800.0;       //设定速度
    position_pub.publish(position_msg);      //将起始位置发送出去，用于控制器订阅
    sleep(5);

    //while(Feedback);
    Feedback = 1;

    std::vector<double> joint_group_positions(6);
    joint_group_positions[0] = targetPose[0] * pi / 180;
    joint_group_positions[1] = targetPose[1] * pi / 180;
    joint_group_positions[2] = targetPose[2] * pi / 180;
    joint_group_positions[3] = targetPose[3] * pi / 180;
    joint_group_positions[4] = targetPose[4] * pi / 180;
    joint_group_positions[5] = targetPose[5] * pi / 180;
    arm.setJointValueTarget(joint_group_positions);
    arm.move();//在rviz中运动到起始位置
    sleep(1);

    //读出起点位置末端的位姿
    geometry_msgs::PoseStamped pose_solved;
    pose_solved = arm.getCurrentPose("tool0");       //arm.getCurrentPose()返回geometry_msgs::PoseStamped类型的变量
    ROS_INFO_STREAM("start: " << pose_solved);          //输出pose_solved的所有信息

    //读出起点位置关节角
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
                                 //将joint_model_group中的关节角赋值给joint_values
    for (std::size_t i = 0; i < joint_names.size(); ++i) {               //std::size_t无符号整数
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }        //输出每个关节角的当前大小

    //开始计时
    clock_t start = clock();

    //加速阶段，4s，加速度为accel1
    for (n = 0; n < 400; n++)//speed up:4s
    {
        //求雅克比矩阵，关节角速度与末端位姿速度之间的雅可比矩阵，位姿速度=雅可比矩阵×角速度
        //雅克比矩阵与机械臂当前的位置状态有关，所以每个步长都要计算一次
        kinematic_state->getJacobian(joint_model_group,     //该变量包含了当前机械臂的位置信息
                                     kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                     reference_point_position,     //这是什么？
                                     jacobian);
        //ROS_INFO_STREAM("Jacobian: " << jacobian);
        //求逆
        jacobian_inv = jacobian.inverse();
        //更新速度
        speedl = speedl + accel1 * del_t;
        speedj = jacobian_inv * speedl;        //得到弧度制
        speed_msg.data.at(0) = speedj[0] * 30 * 180 / pi;
        speed_msg.data.at(1) = speedj[1] * 205 * 180 / (3 * pi);
        speed_msg.data.at(2) = speedj[2] * 50 * 180 / pi;
        speed_msg.data.at(3) = speedj[3] * 125 * 180 / (2 * pi);
        speed_msg.data.at(4) = speedj[4] * 125 * 180 / (2 * pi);
        speed_msg.data.at(5) = speedj[5] * 200 * 180 / (9 * pi);
        speed_pub.publish(speed_msg);
        usleep(9500);      //9500微秒，9.5毫秒
        //然后修改关节角度
        joint_group_positions[0] = joint_group_positions[0] + speedj[0] * 0.01;
        joint_group_positions[1] = joint_group_positions[1] + speedj[1] * 0.01;
        joint_group_positions[2] = joint_group_positions[2] + speedj[2] * 0.01;
        joint_group_positions[3] = joint_group_positions[3] + speedj[3] * 0.01;
        joint_group_positions[4] = joint_group_positions[4] + speedj[4] * 0.01;
        joint_group_positions[5] = joint_group_positions[5] + speedj[5] * 0.01;
        kinematic_state->setJointGroupPositions(joint_model_group, joint_group_positions);   //注意这里是set不是copy
                             //更新joint_model_group中的机械臂位置信息
        /*std::vector<double> joint_values;
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        for(std::size_t i = 0; i < joint_names.size(); ++i)
        {
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }*/

        //sleep(2);
    }
    ROS_INFO_STREAM("speedl: " << speedl);
    //匀速阶段，保持0.02m/s的速度运行3s
    for (n = 0; n < 300; n++) {
        //求雅克比矩阵
        kinematic_state->getJacobian(joint_model_group,
                                     kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                                     reference_point_position,
                                     jacobian);
        //求逆
        jacobian_inv = jacobian.inverse();
        //更新速度
        speedj = jacobian_inv * speedl;
        speed_msg.data.at(0) = speedj[0] * 30 * 180 / pi;
        speed_msg.data.at(1) = speedj[1] * 205 * 180 / (3 * pi);
        speed_msg.data.at(2) = speedj[2] * 50 * 180 / pi;
        speed_msg.data.at(3) = speedj[3] * 125 * 180 / (2 * pi);
        speed_msg.data.at(4) = speedj[4] * 125 * 180 / (2 * pi);
        speed_msg.data.at(5) = speedj[5] * 200 * 180 / (9 * pi);
        speed_pub.publish(speed_msg);
        usleep(9500);
        //然后修改关节角度
        joint_group_positions[0] = joint_group_positions[0] + speedj[0] * 0.01;
        joint_group_positions[1] = joint_group_positions[1] + speedj[1] * 0.01;
        joint_group_positions[2] = joint_group_positions[2] + speedj[2] * 0.01;
        joint_group_positions[3] = joint_group_positions[3] + speedj[3] * 0.01;
        joint_group_positions[4] = joint_group_positions[4] + speedj[4] * 0.01;
        joint_group_positions[5] = joint_group_positions[5] + speedj[5] * 0.01;
        kinematic_state->setJointGroupPositions(joint_model_group, joint_group_positions);
        /*std::vector<double> joint_values;
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        for(std::size_t i = 0; i < joint_names.size(); ++i)
        {
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }*/

        //sleep(2);
    }
    ROS_INFO_STREAM("speedl: " << speedl);

    //5s的减速阶段，减速速度为accel2
    for (n = 0; n < 500; n++) {
        //求雅克比矩阵
        kinematic_state->getJacobian(joint_model_group,
                                     kinematic_state->getLinkModel(
                                             joint_model_group->getLinkModelNames().back()),
                                     reference_point_position,
                                     jacobian);
        //求逆
        jacobian_inv = jacobian.inverse();
        //更新速度
        speedl = speedl + accel2 * del_t;
        speedj = jacobian_inv * speedl;
        speed_msg.data.at(0) = speedj[0] * 30 * 180 / pi;
        speed_msg.data.at(1) = speedj[1] * 205 * 180 / (3 * pi);
        speed_msg.data.at(2) = speedj[2] * 50 * 180 / pi;
        speed_msg.data.at(3) = speedj[3] * 125 * 180 / (2 * pi);
        speed_msg.data.at(4) = speedj[4] * 125 * 180 / (2 * pi);
        speed_msg.data.at(5) = speedj[5] * 200 * 180 / (9 * pi);
        speed_pub.publish(speed_msg);
        usleep(9500);
        //然后修改关节角度，更新关节角度
        joint_group_positions[0] = joint_group_positions[0] + speedj[0] * 0.01;
        joint_group_positions[1] = joint_group_positions[1] + speedj[1] * 0.01;
        joint_group_positions[2] = joint_group_positions[2] + speedj[2] * 0.01;
        joint_group_positions[3] = joint_group_positions[3] + speedj[3] * 0.01;
        joint_group_positions[4] = joint_group_positions[4] + speedj[4] * 0.01;
        joint_group_positions[5] = joint_group_positions[5] + speedj[5] * 0.01;
        kinematic_state->setJointGroupPositions(joint_model_group, joint_group_positions);
        /*std::vector<double> joint_values;
        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        for(std::size_t i = 0; i < joint_names.size(); ++i)
        {
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }*/

        //sleep(2);
    }
    ROS_INFO_STREAM("speedl: " << speedl);


    clock_t ends = clock();
    cout << "Running Time : " << (double) 1000 * (ends - start) / CLOCKS_PER_SEC << endl;

    arm.setJointValueTarget(joint_group_positions);//move
    arm.move();    //在rviz中展示
    pose_solved = arm.getCurrentPose("tool0");//get end position
    ROS_INFO_STREAM("END: " << pose_solved);
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);//get joints value
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }


}
