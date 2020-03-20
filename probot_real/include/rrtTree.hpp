//
//  rrtTree.h
//
//  Created by 韩奔 on 2020/3/1.
//

#ifndef RRTTREE_HPP
#define RRTTREE_HPP

#include <cstdio>
#include <flann/flann.hpp>
#include <stack>
#include <moveit_msgs/CollisionObject.h>
#include "Manipulator.hpp"
#include "treeNode.h"


class rrtTree {
private:
    treeNode root_node;       // 根节点
    treeNode goal_node;       // 目标节点

public:
    Manipulator robot;        // 定义一个Manipulator实例，用于碰撞检测时的运动学计算

    int added_node;           // 用于记录当前树上所加入节点的数量，数值等于最新加入节点的索引数
    int path_node;            // 记录路径上的节点数

	int max_iter;             // 最大迭代次数
    float node_step;         // 节点步长
	double neighbor_radius;            // 邻域半径
	double goal_bias;          // 随机采样时在目标节点附近采样的概率

	std::vector<moveit_msgs::CollisionObject> obstacles;     // 障碍物
	std::vector<double> obs_radius;               // 障碍物的半径
	std::vector<Eigen::MatrixXd> obs_positions;         // 障碍物的中心位置

//    flann::Index<flann::L2<double> >* kd_tree_ptr;     // KD-树（失败）
    std::vector<treeNode> tree;         // 随机树
	std::stack<treeNode> path;          // 所得到的路径


public:
	rrtTree(treeNode &root_node, treeNode &goal_node);

	void setRootNode(treeNode &root_node);
	void setGoalNode(treeNode &goal_node);

	void add_obstacle(moveit_msgs::CollisionObject &obstacle);
	void get_obstacle();

	Eigen::MatrixXd samplePoint(Eigen::MatrixXd max_ang, Eigen::MatrixXd min_ang);      // 采样随机点
//  void getNeighbors(Eigen::MatrixXd &new_point, std::vector<std::vector<int> > *neighbors, std::vector<std::vector<double> > *dist);
    Eigen::MatrixXd getNewPoint(Eigen::MatrixXd &rand_point, int &closest_node_ind);         // 截取新节点

	int chooseParent(Eigen::MatrixXd &new_point, int &closest_node_ind, std::vector<std::vector<int> > &neighbors, std::vector<std::vector<double> > &dist);

	int collision_detect(Eigen::MatrixXd &new_point, int &neighbor_ind);
	double sumCost(Eigen::MatrixXd &new_point, int &neighbor_ind);

	void insertNode(Eigen::MatrixXd &new_point, int parent_ind);     // 将新节点加入搜索树和kd树
    void insertGoalNode(int goal_parent_ind);

	void rewire(int &new_node_ind, std::vector<std::vector<int> > &neighbors, std::vector<std::vector<double> > &dist);  // 新节点邻域范围内重新布线

	void findPath();

};

#endif /* RRTTREE_HPP */
