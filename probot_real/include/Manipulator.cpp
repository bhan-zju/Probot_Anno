//
//  Manipulator.cpp
//  对 Manipulator.hpp 中函数的定义
//  Created by 韩奔 on 2017/2/27.
//  

#include <iostream>
#include <vector>
#include <eigen3/Eigen/Geometry>
#include <moveit/move_group_interface/move_group_interface.h>
#include "probot_anno_manipulator_ikfast_moveit_plugin.cpp"
#include "Manipulator.hpp"

#define PI 3.1416

Manipulator::Manipulator():
arm("manipulator"),
arm_radius(0.08)   //机械臂连杆的半径(粗细)，暂时都按最粗处80mm计算
{
    // 初始化关节角
    joint_angle << 0, 0, 0, 0, 0, 0;

	// 初始化机械臂的DH参数(不一定正确)
	dh_param << 0, 0.2619,   0,      PI / 2,
		        0, 0,       0.225,  0,
		        0, 0,       0,      PI / 2,
		        0, 0.22886, 0,      PI / 2,
		        0, 0,       0,      PI / 2,
		        0, 0.0549,   0,      0;

	max_ang = 3 * PI / 4 * Eigen::Matrix<double,6,1>::Ones(6,1);       //最大关节角135°
	min_ang = -3 * PI / 4 * Eigen::Matrix<double,6,1>::Ones(6, 1);     //最小关节角-135°
	ang_gap = max_ang - min_ang;

    // 关节角补偿
    ang_plus << 0, PI/2, 0, 0, -PI/2, PI;
    ang_prod << 1, 1, 1, 1, -1, 1;
}


// 设定关节角
void Manipulator::setJointAngle(Eigen::Matrix<double,6,1>& q) {
	joint_angle = q;
}

// 正运动学求解
// 输入六个关节角的大小，输入的是第2、3、5、6个关节处在笛卡尔空间的三维坐标，3×4矩阵。用于碰撞检测
Eigen::MatrixXd Manipulator::fkine(Eigen::Matrix<double,6,1>& cur_angle) {      //正运动学
	Eigen::MatrixXd joint_position(3, 4);
	joint_position.col(0) << 0, 0, 0.2818;
	Eigen::Matrix4Xd T = Eigen::Matrix4Xd::Identity(4, 4);

	for (int i = 0; i < 6; i++) {
		double q = cur_angle(i) * ang_prod(i) + ang_plus(i);                    //关节角
        double& d = dh_param(i, 1);                          //连杆偏距
        double& a = dh_param(i, 2);                          //连杆长度
        double& alpha = dh_param(i, 3);                      //扭转角
		Eigen::Matrix4Xd T_temp(4, 4);

		T_temp << cos(q), -sin(q)*cos(alpha), sin(q)*sin(alpha), a*cos(q),
			sin(q), cos(q)*cos(alpha), -cos(q)*sin(alpha), a*sin(q),
			0, sin(alpha), cos(alpha), d,
			0, 0, 0, 1;

		T *= T_temp;
		if (i == 1) {
			joint_position.col(1) = T.block(0, 3, 3, 1);           // 从元素(0,3)开始，提取3×1的矩阵块
		}
		if (i == 3) {
			joint_position.col(2) = T.block(0, 3, 3, 1);
		}
	}
	joint_position.col(3) = T.block(0, 3, 3, 1);
	return joint_position;
}


// 逆运动学求解
// 输入geometry_msgs::Pose类型的目标位姿的四元数(笛卡尔空间)，输出为一个6×1矩阵Eigen::MatrixXd类型
Eigen::Matrix<double,6,1> Manipulator::ikine(geometry_msgs::Pose& target_pose) {

    std::vector<geometry_msgs::Pose> pose1{target_pose};   // 将目标位姿放置一个vector中，方便后续计算

    //设定一个参考点，求解时要用到，作用类似于一个解的试验值
    std::vector<double> seed1(6, 0.0);     // 容器初始化为 6 个 0.0

    std::vector<std::vector<double>> sol_rad;      //二维vector，用于存放逆解

    //设置目标位置所使用的参考坐标系
    arm.setPoseReferenceFrame("base_link");

    //初始化对象(必须初始化)：机器人模型、规划组、参考坐标、终端link、0.01不知道是啥
    bool ret = ik.IKFastKinematicsPlugin::initialize("robot_description","manipulator","base_link",arm.getEndEffectorLink(),0.01);

    kinematics::KinematicsResult kinematic_result;

    //计算逆解，存入sol_rad，计算成功则ret=true，结果存入到 sol_rad 中(弧度制)
    ret = ik.getPositionIK(pose1, seed1, sol_rad, kinematic_result, kinematics::KinematicsQueryOptions());

    double rad_result[sol_rad.size()][6];     //sol_rad.size()是计算出逆解的数量，通常有2个，一般第一个解最好

    Eigen::Matrix<double,6,1> sol_angle = Eigen::Matrix<double,6,1>::Zero();

    if (ret) {       // 如果求解成功
        std::cout << "Ikine solved successfully!\n" << std::endl;
        for(int k = 0; k < sol_rad.size(); k++)
        {
            if (!sol_rad.empty())
            {
                memcpy(rad_result[k], &sol_rad[k][0], sol_rad[0].size() * sizeof(double));     // 将所有解存入 double 型二维数组rad_result中
            }
        }
        for (int i = 0; i < 6; ++i)
            sol_angle(i) = rad_result[0][i];       // 将第一个解拷贝给sol_angle

    }else{
        std::cerr << "No Sulutions for Ikine" << std::endl;    // 如果求解不成功，则输出错误信息
    }

	return sol_angle;
}

