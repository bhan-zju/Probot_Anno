//
//  rrtTree.h
//
//  Created by 韩奔 on 2020/3/1.
//

#ifndef RRTTREE_HPP
#define RRTTREE_HPP

#include <vector>
#include <cstdio>
#include <flann/flann.hpp>
#include <stack>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/CollisionObject.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "Manipulator.hpp"
#include "treeNode.h"
#include "Obstacle.h"

class rrtTree {
private:
    treeNode root_node;       // 根节点
    treeNode goal_node;       // 目标节点
    std::vector<double> goal_angle;
    geometry_msgs::Pose target_pose;      // 笛卡尔空间目标位姿

    float node_step;          // 节点步长
    double goal_bias;         // 随机采样时在目标节点附近采样的概率
    int max_iter;             // 迭代次数

    std::vector<Obstacle*> obstacles;        // 障碍物的指针（目前仅考虑球形障碍物）
    int step_dive;             // 碰撞检测时的分步

    flann::Index<flann::L2<double>>* kd_tree;     // KD-树
    std::vector<treeNode*> tree;      // 搜索树，存放节点的指针
    std::stack<treeNode*> path;       // 所得到的路径，存放节点指针

    Manipulator* anno;

    bool isSuccess;

private:
    ros::NodeHandle nh_;
    moveit::planning_interface::MoveGroupInterface arm;
    std_msgs::Float64MultiArray speed_msg_sim;        // gazebo关节角控制速度
    std_msgs::Float32MultiArray speed_msg;        // 真机关节角控制速度
    ros::Publisher sim_speed_pub;             // 向gazebo发布关节速度控制信息
    ros::Publisher speed_pub;      // 机械臂真机速度发布者

private:
    Eigen::Matrix<double, 6, 1> samplePoint();       // 随机采样节点
    treeNode* getNewPoint(Eigen::Matrix<double, 6, 1>& rand_point);         // 根据随机采样点截取新节点
    bool collisionDetect(Eigen::Matrix<double, 6, 1>& new_point, Eigen::Matrix<double, 6, 1>& neighbor_point);      // 碰撞检测
    double sumCost(Eigen::Matrix<double, 6, 1>& new_point, int& neighbor_ind);
    bool chooseParent(treeNode* newNode, std::vector<std::vector<int> >& neighbors);        // 为新节点选取父节点，如果成功，则返回true
    void insertNode(treeNode* newNode);     // 将新节点加入搜索树和kd树
    void findPath();                        // 寻找路径

public:
    rrtTree(ros::NodeHandle nh_);                // 构造函数

    void setRootNode(const treeNode &root);
    void setRootNode(const Eigen::Matrix<double, 6, 1>& root_joint);
    void setTargetPose(geometry_msgs::Pose& pose);                  // 设定目标位姿

    void addObstacle(Obstacle* obs);                // 添加障碍物（的指针）

    void pathPlanning();                    // 规划路径总过程
    void robotControl();                    // 控制机械臂运动（仿真和真机）

};


#endif /* RRTTREE_HPP */
