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
	Eigen::MatrixXd joint_angle;           //关节角
	Eigen::MatrixXd dh_param;              //DH参数

public:
	int joint_num;                          //关节数
	double arm_radius;                          //机械臂连杆的半径(粗细)，用于碰撞检测

    double link_length[3] = { 0.225, 0.22886, 0.0549 };             //连杆长度(不算第一个连杆)

	Eigen::MatrixXd max_ang;                          //最大关节角
	Eigen::MatrixXd min_ang;                          //最小关节角

public:
	Manipulator();          //构造函数
	void setJointAngle(Eigen::MatrixXd q);            //设定关节角
	void setDhParam(Eigen::MatrixXd dh_param);        //设定DH参数

	Eigen::MatrixXd getDhParam();                     //获取DH参数

	Eigen::MatrixXd fkine(Eigen::MatrixXd joint_angle);            //正运动学
	Eigen::MatrixXd ikine(geometry_msgs::Pose target_pose);        //逆运动学

};

#endif /* MANIPULATOR_HPP */
