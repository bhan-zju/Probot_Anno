//
//  Manipulator.hpp
//  机械臂 类型，包括一个机械臂的 DH 参数、关节角 以及 正逆运动学 等等
//  Created by 韩奔 on 2017/2/27.
//  

#ifndef MANIPULATOR_HPP
#define MANIPULATOR_HPP

#include <eigen3/Eigen/Geometry>

class Manipulator {
private:
	Eigen::Matrix<double, 6, 1> joint_angle;           //关节角
	Eigen::Matrix<double, 6, 4> dh_param;              //DH参数

	// moveit 机械臂模型
    moveit::planning_interface::MoveGroupInterface arm;
    ikfast_kinematics_plugin::IKFastKinematicsPlugin ik;     // 运动学实例

public:
    Eigen::Matrix<double, 6, 1> max_ang;            //最大关节角
    Eigen::Matrix<double, 6, 1> min_ang;            //最小关节角
    Eigen::Matrix<double, 6, 1> ang_gap;            // 最大关节角与最小关节角的差，即关节角跨度

    double arm_radius;                          //机械臂连杆的半径(粗细)，用于碰撞检测
    double link_length[3] = { 0.225, 0.22886, 0.0549 };             //连杆长度(不需要算第一个连杆)

    Eigen::Matrix<double,6,1> ang_plus;     // 这两项用于正运动学时关节角补偿
    Eigen::Matrix<double,6,1> ang_prod;     // 因为DH参数定义的关节角与程序不同

public:
	Manipulator();          //构造函数
	void setJointAngle(Eigen::Matrix<double,6,1>& q);            //设定关节角

	Eigen::MatrixXd fkine(Eigen::Matrix<double,6,1>& cur_angle);            //正运动学
	Eigen::Matrix<double,6,1> ikine(geometry_msgs::Pose& target_pose);        //逆运动学

};

#endif /* MANIPULATOR_HPP */
