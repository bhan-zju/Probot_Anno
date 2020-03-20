//
//  rrt_star.cpp
//  采用 RRT* 路径规划算法 为机械臂规划一条无碰路径
//  Created by 韩奔 on 2020/3/1.
//

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <flann/flann.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include "Manipulator.cpp"
#include "treeNode.h"
#include "rrtTree.cpp"

// 根据起始关节位置设置根节点
treeNode set_root(Eigen::MatrixXd start_angle){
    treeNode root_node;
    root_node.joint_angle = start_angle;           // 根节点的位置
    root_node.ind = 0;                        // 索引号
    root_node.parent_ind = 0;
    root_node.node_cost = 0;
    return root_node;
}

// 根据目标关节位置设置根节点
treeNode set_goal(Eigen::MatrixXd target_angle){
    treeNode goal_node;
    goal_node.joint_angle = target_angle;
    goal_node.parent_ind = -1;
    return goal_node;
}

int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "class_test");
    ros::NodeHandle n;



    // 设定随机数种子
    srand((int)time(0));          // 必须要加这一行，且必须写在循环体外，否则随机采样时生成的是伪随机数


    // 初始化一个 机械臂实例
    Manipulator Anno;


    // 设定 起始点 和 目标点
    // 起始点
    Eigen::MatrixXd start_angle = Eigen::MatrixXd::Zero(6,1);     // 起始点设为 (0, 0, 0, 0, 0, 0)
    // 目标点（示例）
    geometry_msgs::Pose target_pose;
    target_pose.position.x = 0.00;
    target_pose.position.y = 0.28;
    target_pose.position.z = 0.20;
    target_pose.orientation.x = 1;
    Eigen::MatrixXd target_angle = Anno.ikine(target_pose);       // 该过程需要打开rviz
    std::cout << "target_angle: \n" << target_angle << std::endl << std::endl;
    // 将 起始点 和 目标点 转换成树节点的格式
    treeNode root_node = set_root(start_angle);
    treeNode goal_node = set_goal(target_angle);


    // 初始化一个 搜索树实例
    rrtTree Tree(root_node, goal_node);
    if(!Tree.tree.empty()){
        std::cout << "Root node has been added into the tree successfully!\n" << std::endl;
    }else{
        std::cout << "No data in tree!" << std::endl;
    }
    // 创建KD树，方便后续寻找临近点的计算
    flann::Matrix<double> point(root_node.joint_angle.data(), 1, 6);
    flann::Index<flann::L2<double>> kd_tree(point, flann::KDTreeIndexParams(4));       //将初始节点加入kd树
    kd_tree.buildIndex();


    // 添加障碍物实例（暂时只考虑球形障碍物）
    // 简易版：直接设定障碍物的位置和尺寸
    Tree.obs_radius = {0.12};//, 0.13};                // 球形障碍物半径
    Eigen::MatrixXd obs_position1(3,1);
    //Eigen::MatrixXd obs_position2(3,1);
    //Eigen::MatrixXd obs_position3(3,1);
    obs_position1 << 0.22, 0.20, 0.30;
    //obs_position2 << 0.25, 0.13, 0.13;
    //obs_position3 << 0.20, 0,20, 0.39;
    Tree.obs_positions.push_back(obs_position1);
    //Tree.obs_positions.push_back(obs_position2);
    //Tree.obs_positions.push_back(obs_position3);


    //开始计时
    clock_t start = clock();


    // 开始迭代
    for(int i = 1; i < Tree.max_iter; ++i){
        //std::cout << "iteration" << i <<std::endl;

        // 随机采样，得到rand_point
        Eigen::MatrixXd rand_point = Tree.samplePoint(Anno.max_ang, Anno.min_ang);
        // std::cout << "rand_point: \n" << rand_point <<std::endl;


        // 截取得到新节点new_point
        std::vector<std::vector<int> > index;      // 用于存储最近点的索引号
        std::vector<std::vector<double> > dist0;   // 用于存储最近距离
        flann::Matrix<double> query_node(rand_point.data(), 1, 6);
        // kd_tree.knnSearch()输入的
        // 第一项是要查询的点
        // 第二项保存所有被找到的K最近邻的索引号
        // 第三项保存所有被找到的K最近邻的距离
        // 第四项是要找的临近点个数
        // 第五项是搜索时要使用的参数的结构体
        kd_tree.knnSearch(query_node, index, dist0, 1, flann::SearchParams(32, 0, false));
        int closest_node_ind = index[0][0];
        float closest_dist = (rand_point - Tree.tree[closest_node_ind].joint_angle).norm();
        //std::cout << "closest_node_ind: " << closest_node_ind << "\tclosest_dist: " << closest_dist << std::endl;
        Eigen::MatrixXd new_point;
        if(closest_dist <= Tree.node_step){
            new_point = rand_point;
        }else{
            new_point = Tree.getNewPoint(rand_point, closest_node_ind);
        }
        //std::cout << "new_point: \n" << new_point <<std::endl;


        // 找到新节点邻域内的节点
        std::vector<std::vector<int> > neighbors;     // 用于存储临近点的索引号
        std::vector<std::vector<double> > dists;      // 用于存储距离临近点的距离
        flann::Matrix<double> new_kd_point(new_point.data(), 1, 6);
        kd_tree.radiusSearch(new_kd_point, neighbors, dists, Tree.neighbor_radius, flann::SearchParams(32, 0, false));


        // 为新节点选择父节点
        int parent_ind = Tree.chooseParent(new_point, closest_node_ind, neighbors, dists);
        //std::cout << "parent_ind: " << parent_ind << std::endl;

        // 将新节点加入搜索树和kd树
        if(parent_ind >= 0){                  // parent_ind < 0 说明有碰撞
            Tree.insertNode(new_point, parent_ind);
            flann::Matrix<double> node(new_point.data(), 1, 6);
            kd_tree.addPoints(node);
            // 邻域内重新布线
            Tree.rewire(Tree.added_node, neighbors, dists);
        }

    }
    std::cout << Tree.tree.size() << " nodes has added into the tree\n";
    std::cout << "The newest node ind is " << Tree.added_node << std::endl << std::endl;


    // 将目标节点加入搜索树
    // 找到目标节点邻域内的节点
    std::vector<std::vector<int> > neighbors;     // 用于存储临近点的索引号
    std::vector<std::vector<double> > dists;      // 用于存储距离临近点的距离
    flann::Matrix<double> goal_kd_point(target_angle.data(), 1, 6);
    kd_tree.radiusSearch(goal_kd_point, neighbors, dists, Tree.node_step, flann::SearchParams(32, 0, false));
    int closest_node_ind = neighbors[0][0];
    // 为目标节点匹配父节点
    int goal_parent_ind = Tree.chooseParent(target_angle, closest_node_ind, neighbors, dists);
    // 将目标节点加入搜索树
    if(goal_parent_ind >= 0){                  // parent_ind < 0 说明有碰撞
        Tree.insertGoalNode(goal_parent_ind);
        flann::Matrix<double> goal_kd_node(target_angle.data(), 1, 6);
        kd_tree.addPoints(goal_kd_node);
        std::cout << "goal_parent_ind: " << goal_parent_ind << std::endl;
        std::cout << "The goal node has added on the tree!\n" << std::endl;
        std::cout << "The summary of the cost is " << Tree.tree[Tree.added_node].node_cost << std::endl;
    }else{
        std::cerr << "The goal node does ont have a parent!\n" << std::endl;
        return -1;
    }


    // 回溯路径
    Tree.findPath();

    // 计时结束
    clock_t end = clock();
    std::cout << "RRT* Running Time : " << (double) (end - start) / CLOCKS_PER_SEC << "seconds." << std::endl;
    //sleep(3);

    // 显示路径上的节点




    // 尝试在rviz运行
    //多线程
    ros::AsyncSpinner spinner(1);
    //开启新的线程
    spinner.start();
    //初始化需要使用move group控制的arm group
    moveit::planning_interface::MoveGroupInterface arm("manipulator");
    //设置机械臂的允许误差值（单位：弧度）(这一步也可以不设置，不影响运行)
    arm.setGoalJointTolerance(0.001);
    //设置最大加速度和最大速度(这一步也可以不设置，不影响运行)
    arm.setMaxAccelerationScalingFactor(0.5);
    arm.setMaxVelocityScalingFactor(0.5);

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);


    // 获取机械臂的起始位置，存入 start_state，将当前状态存入 joint_model_group
    moveit::core::RobotStatePtr start_state(arm.getCurrentState());
    const robot_state::JointModelGroup *joint_model_group = start_state->getJointModelGroup(arm.getName());

    std::vector<double> joint_group_positions;
    // 将当前状态拷贝给 joint_group_positions
    start_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // 初始化路径，每个节点对应一小段路径
    std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans(Tree.path_node);
    int i = 0;

    //
    while(!Tree.path.empty()){
        // 将路径中的每个点逐个设为目标点
        treeNode current_node = Tree.path.top();
        joint_group_positions[0] = current_node.joint_angle(0);
        joint_group_positions[1] = current_node.joint_angle(1);
        joint_group_positions[2] = current_node.joint_angle(2);
        joint_group_positions[3] = current_node.joint_angle(3);
        joint_group_positions[4] = current_node.joint_angle(4);
        joint_group_positions[5] = current_node.joint_angle(5);
        arm.setJointValueTarget(joint_group_positions);
        std::cout << "node_angle[" << i << "]:\n" << current_node.joint_angle << "\n" << std::endl;

        moveit::planning_interface::MoveItErrorCode success = arm.plan(plans[i]);
        ++i;

        // 更新初始状态
        joint_model_group = start_state->getJointModelGroup(arm.getName());
        start_state->setJointGroupPositions(joint_model_group, joint_group_positions);
        arm.setStartState(*start_state);

        Tree.path.pop();
    }

    // 将路径连接
    // 初始化一条轨迹
    moveit_msgs::RobotTrajectory trajectory;
    // 先将第一条路径加入轨迹
    trajectory.joint_trajectory.joint_names = plans[0].trajectory_.joint_trajectory.joint_names;
    trajectory.joint_trajectory.points = plans[0].trajectory_.joint_trajectory.points;

    // 将其余路径加入轨迹
    for(int j = 1; j < plans.size(); ++j){
        for(size_t k = 1; k < plans[j].trajectory_.joint_trajectory.points.size(); ++k){
            trajectory.joint_trajectory.points.push_back(plans[j].trajectory_.joint_trajectory.points[k]);
        }
    }

    //
    moveit::planning_interface::MoveGroupInterface::Plan joinedPlan;    //定义一个新的规划轨迹joinedPlan
    robot_trajectory::RobotTrajectory rt(arm.getCurrentState()->getRobotModel(), "manipulator");
    rt.setRobotTrajectoryMsg(*arm.getCurrentState(), trajectory);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    iptp.computeTimeStamps(rt, 0.5, 0.5);   // z最大速度和最大加速度
    rt.getRobotTrajectoryMsg(trajectory);
    joinedPlan.trajectory_ = trajectory;

    // 控制机械臂运动
    if(!arm.execute(joinedPlan)){
        ROS_ERROR("Failed to execute plan");
        return -1;
    }

    ROS_INFO("Finished");

/*
 *  // 直接这样运动，中间会停顿
    while(!Tree.path.empty()){
        treeNode current_node = Tree.path.top();
        std::vector<double> joint_group_positions(6);
        joint_group_positions[0] = current_node.joint_angle(0);
        joint_group_positions[1] = current_node.joint_angle(1);
        joint_group_positions[2] = current_node.joint_angle(2);
        joint_group_positions[3] = current_node.joint_angle(3);
        joint_group_positions[4] = current_node.joint_angle(4);
        joint_group_positions[5] = current_node.joint_angle(5);
        std::cout << "angle planned:\n" << current_node.joint_angle << std::endl;
        std::cout << "costs: \t" << current_node.node_cost << std::endl;
        //将关节向量写入
        arm.setJointValueTarget(joint_group_positions);
        arm.move();   //规划、控制
        //sleep(1);
        Tree.path.pop();
    }

    sleep(3);
*/




    return 0;
}