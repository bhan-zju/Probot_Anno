//
//  treeNode.h
//  搜索树中的 节点 类型，包含一个节点的 位置、索引、父节点的索引 和 从根节点到该节点的代价
//  Created by 韩奔 on 2017/2/27.
//

#ifndef TREENODE_H
#define TREENODE_H

#include <eigen3/Eigen/Geometry>

typedef struct treeNode{
    Eigen::MatrixXd joint_angle = Eigen::MatrixXd::Zero(6,1);      // 该节点的位置，初始化维度为6×1
    int ind;                 // 该节点的索引
    int parent_ind;          // 该节点的父节点的索引
    double node_cost;         // 从根节点到该节点的代价
}treeNode;

#endif /* TREENODE_H */
