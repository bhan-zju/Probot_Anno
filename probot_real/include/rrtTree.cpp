//
//  rrtTree.cpp
//
//  Created by 韩奔 on 2020/3/1.
//

#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Geometry>
#include "rrtTree.hpp"
#include "Manipulator.hpp"

// 构造函数
rrtTree::rrtTree(treeNode &root_node, treeNode &goal_node) {

    setRootNode(root_node);     // 设定根节点
    setGoalNode(goal_node);     // 设定目标节点

    added_node = 0;             // 已加入第0个节点，即根节点
    path_node = 0;
    max_iter = 8000;            // 最大迭代次数
    node_step = 0.4;            // 节点步长
    neighbor_radius = 0.4;      // 邻域半径
    goal_bias = 0.5;            // 在目标点附近采样的概率

}


// 设定根节点
void rrtTree::setRootNode(treeNode &root_node) {
    rrtTree::root_node = root_node;
    rrtTree::tree.push_back(root_node);
}


// 设定目标节点
void rrtTree::setGoalNode(treeNode &goal_node) {
    rrtTree::goal_node = goal_node;
}


// 添加障碍物
void rrtTree::add_obstacle(moveit_msgs::CollisionObject &obstacle) {
    rrtTree::obstacles.push_back(obstacle);
}


// 得到障碍物的位置和大小信息
void rrtTree::get_obstacle() {

}


// 随机采样节点，并以一定的概率在目标节点附近进行采样
Eigen::MatrixXd rrtTree::samplePoint(Eigen::MatrixXd max_ang, Eigen::MatrixXd min_ang){

    Eigen::MatrixXd rand_point;
    if((double)rand()/RAND_MAX < rrtTree::goal_bias){
        // Eigen::MatrixXd::Random()返回随机数范围为(-1,1)
        rand_point = rrtTree::goal_node.joint_angle + Eigen::MatrixXd::Random(6,1) * rrtTree::node_step;
    }else{
        Eigen::MatrixXd step = max_ang - min_ang;
        Eigen::MatrixXd random = (Eigen::MatrixXd::Random(6,1) + Eigen::MatrixXd::Ones(6,1)) / 2;    // 生成(0,1)之间随机数
        // Eigen::MatrixXd::cwiseProduct()函数，逐元素相乘，类似matlab的 .*
        rand_point = step.cwiseProduct(random) + min_ang;
    }
    return rand_point;
}

/*
// 得到新节点的邻域
void rrtTree::getNeighbors(Eigen::MatrixXd &new_point, std::vector<std::vector<int> > *neighbors, std::vector<std::vector<double> > *dist) {

    // 找到新节点邻域内的所有节点
    flann::Matrix<double> new_kd_point(new_point.data(), 1, 6);
    rrtTree::kd_tree_ptr -> radiusSearch(new_kd_point, *neighbors, *dist, rrtTree::node_step, flann::SearchParams(32, 0, false));
    // 这些临近点的序号存入neighbors，这些点与new_node之间的距离存入dist

}
*/

// 根据步长截取得到新节点
Eigen::MatrixXd rrtTree::getNewPoint(Eigen::MatrixXd &rand_point, int &closest_node_ind) {

    Eigen::MatrixXd new_point;

    Eigen::MatrixXd closest_point = rrtTree::tree[closest_node_ind].joint_angle;
    Eigen::MatrixXd angle_diff = rand_point - closest_point;
    new_point = closest_point + rrtTree::node_step * angle_diff / angle_diff.norm();

    return new_point;
}


// 为新节点选择父节点
int rrtTree::chooseParent(Eigen::MatrixXd &new_point, int &closest_node_ind, std::vector<std::vector<int> > &neighbors, std::vector<std::vector<double> > &dist) {

    double min_cost = rrtTree::sumCost(new_point, closest_node_ind);
    double temp_cost = min_cost;
    int parent_ind = -1;
    if(!rrtTree::collision_detect(new_point, closest_node_ind))
        parent_ind = closest_node_ind;
    // 在邻域内选择父节点
    for (int i = 0; i < neighbors[0].size(); ++i){
        double dist = (new_point - rrtTree::tree[neighbors[0][i]].joint_angle).norm();
        if((!rrtTree::collision_detect(new_point, neighbors[0][i])) && (dist <= rrtTree::neighbor_radius)){
            temp_cost = rrtTree::sumCost(new_point, neighbors[0][i]);
            if (temp_cost < min_cost){
                parent_ind = neighbors[0][i];
                min_cost = temp_cost;
            }
        }
    }

    return parent_ind;
}


// 碰撞检测
int rrtTree::collision_detect(Eigen::MatrixXd &new_point, int &neighbor_ind) {

    int step_dive = 4;
    int obs_num = rrtTree::obs_radius.size();    // 障碍物的数量
    //std::cout << "The obstacle's num is " << obs_num << std::endl;

    Eigen::MatrixXd neighbor_point = rrtTree::tree[neighbor_ind].joint_angle;
    Eigen::MatrixXd dist_temp = new_point - neighbor_point;
    double dist = dist_temp.norm();
    Eigen::MatrixXd vector = dist_temp / dist;     // 从临近点 指向 新节点 的 单位向量
    double step = dist / step_dive;

    Eigen::MatrixXd state_angle;
    Eigen::MatrixXd joint_position;           // 每个关节末端在笛卡尔空间的位置
    Eigen::MatrixXd obs_dist;
/*
 *  // 不知道下面的步骤怎么错了
    for(int i = 1; i <= step_dive; ++i){
        state_angle = neighbor_point + i * step * vector;
        joint_position = rrtTree::robot.fkine(state_angle);    // 计算每个关节末端在笛卡尔空间的位置
        double min_dist = 65535;
        // 计算机械臂各连杆与所有障碍物之间的最短距离
        for(int j = 0; j < obs_num; ++j){
            obs_dist = (joint_position - rrtTree::obs_positions[j].replicate(1,4)).colwise().norm();
            // 求出机械臂各连杆与某个障碍物之间的最短距离
            for(int k = 0; k < 3; ++k){
                if (obs_dist(0, k) + obs_dist(0, k+1) == rrtTree::robot.link_length[k]){
                    return 1;
                }else if (pow(obs_dist(0, k+1), 2) >= pow(obs_dist(0, k),2) + pow(rrtTree::robot.link_length[k], 2)){   // 在k一侧
                    if (obs_dist(0, k) < min_dist)
                        min_dist = obs_dist(0, k);
                }else if (pow(obs_dist(0, k), 2) >= pow(obs_dist(0, k+1),2) + pow(rrtTree::robot.link_length[k], 2)){   // 在k+1一侧
                    if (obs_dist(0, k+1) < min_dist)
                        min_dist = obs_dist(0, k+1);
                }else{                                            // 在连杆中间时
                    double p = (obs_dist(0, k) + obs_dist(0, k+1) + rrtTree::robot.link_length[k]) / 2;
                    double s = sqrt(p * (p - obs_dist(0,k)) * (p - obs_dist(0, k+1)) * (p - rrtTree::robot.link_length[k]));
                    double h = 2 * s / rrtTree::robot.link_length[k];
                    if (h < min_dist)
                        min_dist = h;
                }
            }
        }
        min_dist = min_dist - rrtTree::robot.arm_radius - obs_radius[i];
        std::cout << "min_dist: " << min_dist << std::endl;
        if(min_dist < 0 || min_dist > 100){     // min_dist > 100说明计算错误
            return 1;
        }
    }
*/

    for(int i = 0; i < obs_num; ++i){
        for(int j = 1; j <= step_dive; ++j){
            state_angle = neighbor_point + j * step * vector;
            joint_position = rrtTree::robot.fkine(state_angle);     // 计算每个关节末端在笛卡尔空间的位置
            //std::cout << "joint_position: \n" << joint_position << std::endl;
            // 计算 障碍物 与 每个关节末端位置 之间的距离
            obs_dist = (joint_position - rrtTree::obs_positions[i].replicate(1,4)).colwise().norm();
            double min_dist = 65535;
            for(int k = 0; k < 3; ++k){
                if (obs_dist(0, k) + obs_dist(0, k+1) == rrtTree::robot.link_length[k]){
                    return 1;
                }else if (pow(obs_dist(0, k+1), 2) >= pow(obs_dist(0, k),2) + pow(rrtTree::robot.link_length[k], 2)){
                    if (obs_dist(0, k) < min_dist)
                        min_dist = obs_dist(0, k);
                }else if (pow(obs_dist(0, k), 2) >= pow(obs_dist(0, k+1),2) + pow(rrtTree::robot.link_length[k], 2)){
                    if (obs_dist(0, k+1) < min_dist)
                        min_dist = obs_dist(0, k+1);
                }else{
                    double p = (obs_dist(0, k) + obs_dist(0, k+1) + rrtTree::robot.link_length[k]) / 2;
                    double s = sqrt(p * (p - obs_dist(0,k)) * (p - obs_dist(0, k+1)) * (p - rrtTree::robot.link_length[k]));
                    double h = 2 * s / rrtTree::robot.link_length[k];
                    if (h < min_dist)
                        min_dist = h;
                }
            }
            min_dist = min_dist - rrtTree::robot.arm_radius - obs_radius[i];
            //std::cout << "min_dist: " << min_dist << std::endl;
            if(min_dist < 0 || min_dist > 100){     // min_dist > 100说明计算错误
                return 1;
            }
        }
    }

    return 0;
}


// 从根节点到新节点的代价，笛卡尔空间的距离 和 关节空间的距离 加权求和
double rrtTree::sumCost(Eigen::MatrixXd &new_point, int &neighbor_ind) {
    // 设定 笛卡尔空间的距离 和 关节空间的距离 的权重
    int w1 = 50, w2 = 50;
    Eigen::MatrixXd neighbor_point = rrtTree::tree[neighbor_ind].joint_angle;

    // 正运动学求解末端在笛卡尔空间的位置
    Eigen::MatrixXd new_position = robot.fkine(new_point).col(3);
    Eigen::MatrixXd neighbor_position = robot.fkine(neighbor_point).col(3);

    double cartesian_cost = (new_position - neighbor_position).norm();         // 笛卡尔空间的距离
    double joint_cost = (new_point - neighbor_point).norm();                   // 关节空间的距离

    double sum_cost = w1 * cartesian_cost + w2 * joint_cost + rrtTree::tree[neighbor_ind].node_cost;
    return sum_cost;
}


// 将节点加入搜索树和
void rrtTree::insertNode(Eigen::MatrixXd &new_point, int parent_ind) {
    if(parent_ind >= 0){
        ++rrtTree::added_node;
        treeNode new_node;
        new_node.joint_angle = new_point;
        new_node.ind = rrtTree::added_node;
        new_node.parent_ind = parent_ind;
        new_node.node_cost = sumCost(new_point, parent_ind);

        rrtTree::tree.push_back(new_node);
        // flann::Matrix<double> node(new_node.joint_angle.data(), 1, 6);
        // kd_tree_ptr -> addPoints(node);
    }
}


// 将目标节点加入搜索树
void rrtTree::insertGoalNode(int goal_parent_ind) {
    if(goal_parent_ind >= 0){
        ++rrtTree::added_node;
        rrtTree::goal_node.ind = rrtTree::added_node;
        rrtTree::goal_node.parent_ind = goal_parent_ind;
        rrtTree::goal_node.node_cost = sumCost(rrtTree::goal_node.joint_angle, goal_parent_ind);
        rrtTree::tree.push_back(goal_node);
    }
}


// 重新布线：新节点邻域内的节点选择新的父节点
void rrtTree::rewire(int &new_node_ind, std::vector<std::vector<int> > &neighbors, std::vector<std::vector<double> > &dist) {
    double min_cost;
    double temp_cost;
    Eigen::MatrixXd new_point = rrtTree::tree[new_node_ind].joint_angle;

    // 在邻域内选择父节点
    for (int i = 0; i < neighbors[0].size(); ++i){
        Eigen::MatrixXd neighbor_point = rrtTree::tree[neighbors[0][i]].joint_angle;
        double dist = (new_point - neighbor_point).norm();
        if(!rrtTree::collision_detect(new_point, neighbors[0][i]) && dist <= rrtTree::node_step) {
            temp_cost = rrtTree::sumCost(neighbor_point, new_node_ind);     // 注意该函数输入参数的顺序
            if (temp_cost < rrtTree::tree[neighbors[0][i]].node_cost){
                rrtTree::tree[neighbors[0][i]].parent_ind = rrtTree::tree[new_node_ind].ind;
                rrtTree::tree[neighbors[0][i]].node_cost = temp_cost;
                //std::cout<<"rewire successfully!"<<std::endl;
            }
        }
    }
}


// 回溯路径
void rrtTree::findPath() {
    int ind = rrtTree::goal_node.ind;
    while(ind > 0){
        rrtTree::path.push(rrtTree::tree[ind]);
        ind = rrtTree::tree[ind].parent_ind;
        ++rrtTree::path_node;
    }
    rrtTree::path.push(rrtTree::root_node);
    ++rrtTree::path_node;
    std::cout << "There are " << rrtTree::path_node << " nodes on the path.\n" << std::endl;
}