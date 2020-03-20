// 本程序用来测试include中所建的几个类是否有效

#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <flann/flann.hpp>
#include "Manipulator.cpp"
#include "treeNode.h"
#include "rrtTree.cpp"

int main(int argc, char **argv)
{
    // 测试 treeNode 类
    std::vector<treeNode> tree;
    treeNode node1;
    node1.ind = 0;
    node1.parent_ind = 2;
    node1.node_cost = 15;
    tree.push_back(node1);


    // 测试 Manipulator 类
    Manipulator anno;                  // 必须初始化ros节点，才能进行后续的逆运动学，并要将moveit打开
    std::cout << anno.joint_num << std::endl;
/*
    // 测试逆运动学
    ros::init(argc, argv, "class_test");
    ros::NodeHandle n;

    geometry_msgs::Pose target_pose;

    target_pose.position.x = 0.233385;
    target_pose.position.y = 0.102423;
    target_pose.position.z = 0.204072;
    target_pose.orientation.x = -0.991783;
    target_pose.orientation.y = 0.117883;
    target_pose.orientation.z = 0.0201464;
    target_pose.orientation.w = 0.0454412;

    Eigen::MatrixXd joint_angle;

    joint_angle = anno.ikine(target_pose);
    std::cout << joint_angle << std::endl;
*/

    // 测试正运动学
    Eigen::MatrixXd joint_angle1 = Eigen::MatrixXd::Zero(6,1);
    joint_angle1 << 0.392215, -0.594776, -0.381265, 0.177883, 0.977981, 0.481268;
    Eigen::MatrixXd joint_positions;

    joint_positions = anno.fkine(joint_angle1);
    std::cout << joint_positions << std::endl << std::endl;

    // 测试rrtTree类
    // 初始化一个rrtTree实例
    treeNode root_node, goal_node;
    Eigen::MatrixXd root_angle = Eigen::MatrixXd::Zero(6,1);
    root_node.ind = 0;
    root_node.parent_ind = 0;
    root_node.joint_angle = root_angle;           // 根节点的位置
    //std::cout << root_node.joint_angle << std::endl;
    root_node.node_cost = 0;
    goal_node.joint_angle << 0.392215, -0.594776, -0.381265, 0.177883, 0.977981, 0.481268;     // 目标节点的位置，两种赋值方法
    //std::cout << goal_node.joint_angle << std::endl;
    rrtTree rrttree(root_node, goal_node);
    if(!rrttree.tree.empty()){
        std::cout << rrttree.tree[0].joint_angle << std::endl << std::endl;
    }else{
        std::cout << "no data in tree" << std::endl;
    }

    // 添加障碍物
    // 简易版：直接设定障碍物的位置和尺寸
    rrttree.obs_radius = {0.15, 0.13};
    Eigen::MatrixXd obs_position1(3,1), obs_position2(3,1);
    obs_position1 << 0.4, -0.15, 0.15;
    obs_position2 << 0.4, 0.13, 0.13;
    rrttree.obs_positions.push_back(obs_position1);
    rrttree.obs_positions.push_back(obs_position2);
    std::cout << "obs\n" <<rrttree.obs_positions[1].replicate(1,4) << std::endl << std::endl;


    // kd树，只能自己创建，不能用在类里
    flann::Matrix<double> point(root_node.joint_angle.data(), 1, 6);
    flann::Index<flann::L2<double>> kd_tree(point, flann::KDTreeIndexParams(4));           //创建一个kd树kd_tree，并将初始节点加入kd树
    kd_tree.buildIndex();

/*

    // 随机采样，得到随机点

    Eigen::MatrixXd rand_point(6,1);
    rand_point = rrttree.samplePoint(anno.max_ang, anno.min_ang);
    std::cout << rand_point <<std::endl << std::endl;







    // 截取得到新节点
    //Eigen::MatrixXd new_point = rrttree.getNewPoint(rand_point);
    //std::cout << new_point <<std::endl;
    std::vector<std::vector<int> > index;
    std::vector<std::vector<double> > dist;
    flann::Matrix<double> query_node(rand_point.data(), 1, 6);
    // kd_tree -> knnSearch()输入的
    // 第一项是要查询的点
    // 第二项保存所有被找到的K最近邻的索引号
    // 第三项保存所有被找到的K最近邻的距离
    // 第四项是要找的临近点个数
    // 第五项是搜索时要使用的参数的结构体
    kd_tree.knnSearch(query_node, index, dist, 1, flann::SearchParams(32, 0, false));
    int closest_node_ind = index[0][0];
    float closest_dist = dist[0][0];
    std::cout << closest_node_ind << "\t" << closest_dist << std::endl;

    Eigen::MatrixXd new_point;

    if(closest_dist <= rrttree.node_step){
        new_point = rand_point;
    }else{
        new_point = rrttree.getNewPoint(rand_point, closest_node_ind);
    }
    std::cout << new_point << std::endl << std::endl;





    // 找到新节点邻域内的节点
    std::vector<std::vector<int> > neighbors;
    std::vector<std::vector<double> > dist2;
    flann::Matrix<double> new_kd_point(new_point.data(), 1, 6);
    kd_tree.radiusSearch(new_kd_point, neighbors, dist, rrttree.node_step, flann::SearchParams(32, 0, false));


    // 为新节点选择父节点
    int parent_ind = rrttree.chooseParent(new_point, closest_node_ind, neighbors, dist2);
    std::cout << parent_ind << std::endl;



    // 将新节点加入搜索树和kd树
    if(parent_ind >= 0){
        rrttree.insertNode(new_point, parent_ind);
        flann::Matrix<double> node(new_point.data(), 1, 6);
        kd_tree.addPoints(node);
    }
    std::cout << rrttree.tree.size() << " ";
    std::cout << rrttree.added_node << " ";
    std::cout << rrttree.tree[1].joint_angle << std::endl << std::endl;




    // 邻域内重新布线
    rrttree.rewire(rrttree.added_node, neighbors, dist2);
    std::cout << rrttree.tree[1].joint_angle << std::endl << std::endl;

*/


    srand((int)time(0));            // 必须要加这一行，且必须写在循环体外，否则生成的是伪随机数

    // 开始迭代, 先采样30个点试验
    for(int i = 1; i < 30; ++i){
        std::cout << "iteration" << i <<std::endl;

        // 随机采样
        std::cout << "Test rand number: " <<(double)rand()/RAND_MAX << std::endl;
        Eigen::MatrixXd rand_point = rrttree.samplePoint(anno.max_ang, anno.min_ang);
        //std::cout << "rand_point: \n" << rand_point <<std::endl;



        // 截取得到新节点
        std::vector<std::vector<int> > index;
        std::vector<std::vector<double> > dist;
        flann::Matrix<double> query_node(rand_point.data(), 1, 6);
        // kd_tree.knnSearch()输入的
        // 第一项是要查询的点
        // 第二项保存所有被找到的K最近邻的索引号
        // 第三项保存所有被找到的K最近邻的距离
        // 第四项是要找的临近点个数
        // 第五项是搜索时要使用的参数的结构体
        kd_tree.knnSearch(query_node, index, dist, 1, flann::SearchParams(32, 0, false));
        int closest_node_ind = index[0][0];
        float closest_dist = dist[0][0];
        std::cout << "closest_node_ind: " << closest_node_ind << "\tclosest_dist: " << closest_dist << std::endl;

        Eigen::MatrixXd new_point;

        if(closest_dist <= rrttree.node_step){
            new_point = rand_point;
        }else{
            new_point = rrttree.getNewPoint(rand_point, closest_node_ind);
        }
        std::cout << "new_point: \n" << new_point <<std::endl;


        // 找到新节点邻域内的节点
        std::vector<std::vector<int> > neighbors;
        std::vector<std::vector<double> > dist2;
        flann::Matrix<double> new_kd_point(new_point.data(), 1, 6);
        kd_tree.radiusSearch(new_kd_point, neighbors, dist2, rrttree.node_step, flann::SearchParams(32, 0, false));
        std::cout << "neighbor's size: " << neighbors[0].size() <<std::endl;


        // 碰撞检测
        //int collision = rrttree.collision_detect(new_point, closest_node_ind);



        // 为新节点选择父节点
        int parent_ind = rrttree.chooseParent(new_point, closest_node_ind, neighbors, dist2);
        std::cout << "parent_ind: " << parent_ind << std::endl;
        if((parent_ind != closest_node_ind) && (parent_ind >= 0))
            std::cout << "************************************************************" <<std::endl;

        // 将新节点加入搜索树和kd树
        if(parent_ind >= 0){                  // parent_ind < 0 说明有碰撞
            rrttree.insertNode(new_point, parent_ind);
            flann::Matrix<double> node(new_point.data(), 1, 6);
            kd_tree.addPoints(node);

            // 邻域内重新布线
            rrttree.rewire(rrttree.added_node, neighbors, dist2);
        }
        std::cout << rrttree.tree.size() << " nodes has added into the tree\n";
        std::cout << "The new node ind is " << rrttree.added_node << std::endl << std::endl;


    }



    return 0;

}