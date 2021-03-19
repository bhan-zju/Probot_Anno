//
//  rrtTree.cpp
//
//  Created by 韩奔 on 2020/3/1.
//

#ifndef RRTTREE_CPP
#define RRTTREE_CPP

#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Geometry>

#include "rrtTree.hpp"
#include "Manipulator.hpp"

#define PI 3.1416

// 构造函数1
rrtTree::rrtTree(ros::NodeHandle nh):
root_node(),
goal_node(),
goal_angle(6,0),
goal_bias(0.05),
node_step(0.4),
max_iter(6000),
step_dive(3),
anno(new Manipulator()),
isSuccess(false),
nh_(nh),
arm("manipulator")
{
    // 初始化搜索树
    tree.clear();
    tree.push_back(&root_node);
    // 初始化kd树
    flann::Matrix<double> point(root_node.joint_angle.data(), 1, 6);
    kd_tree = new flann::Index<flann::L2<double>>(point, flann::KDTreeIndexParams(4));
    kd_tree->buildIndex();

    goal_node.node_cost = 10000;

    speed_msg.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    speed_msg_sim.data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    speed_pub = nh_.advertise<std_msgs::Float32MultiArray>("speed_chatter", 1000);
    sim_speed_pub = nh_.advertise<std_msgs::Float64MultiArray>("/probot_anno/arm_vel_controller/command",1);
}

// 设定根节点
void rrtTree::setRootNode(const Eigen::Matrix<double, 6, 1>& root_angle) {
    root_node.joint_angle =  root_angle;
    tree.clear();
    tree.push_back(&root_node);
    // 初始化kd树
    flann::Matrix<double> point(root_node.joint_angle.data(), 1, 6);
    kd_tree = new flann::Index<flann::L2<double>>(point, flann::KDTreeIndexParams(4));
    kd_tree->buildIndex();
}
void rrtTree::setRootNode(const treeNode &root) {
    setRootNode(root.joint_angle);
}

// 设定目标位姿和目标节点
void rrtTree::setTargetPose(geometry_msgs::Pose& pose) {
    goal_node.joint_angle = anno->ikine(pose);       // 该过程需要打开rviz
    for(int i = 0; i < 6; ++i)
        goal_angle[i] = goal_node.joint_angle(i,0);
    std::cout << goal_node.joint_angle << std::endl;
}

// 添加障碍物（的指针）
void rrtTree::addObstacle(Obstacle* obsPtr){
    obstacles.push_back(obsPtr);
}

// 随机采样
Eigen::Matrix<double, 6, 1> rrtTree::samplePoint(){
    Eigen::Matrix<double, 6, 1> rand_point;
    if((double)rand()/RAND_MAX < goal_bias){
        // Eigen::MatrixXd::Random()返回随机数范围为(-1,1)
        rand_point = goal_node.joint_angle + Eigen::Matrix<double, 6, 1>::Random() * node_step;
    }else{
        Eigen::Matrix<double, 6, 1> random = (Eigen::Matrix<double, 6, 1>::Random() + Eigen::Matrix<double, 6, 1>::Ones()) / 2;    // 生成(0,1)之间随机数
        // Eigen::MatrixXd::cwiseProduct()函数，逐元素相乘，类似matlab的 .*
        rand_point = anno->ang_gap.cwiseProduct(random) + anno-> min_ang;
    }
    return rand_point;
}

// 根据步长截取得到新节点（的指针）
treeNode* rrtTree::getNewPoint(Eigen::Matrix<double, 6, 1> &rand_point) {
    // 先使用kd树找到搜索树上距离采样点最近的节点
    std::vector<std::vector<int> > index;        // 用于存储最近点的索引号
    std::vector<std::vector<double> > dist0;     // 用于存储最近距离
    flann::Matrix<double> query_node(rand_point.data(), 1, 6);
    // kd_tree.knnSearch()输入的
    // 第一项是要查询的点
    // 第二项保存所有被找到的K最近邻的索引号
    // 第三项保存所有被找到的K最近邻的距离
    // 第四项是要找的临近点个数
    // 第五项是搜索时要使用的参数的结构体
    kd_tree->knnSearch(query_node, index, dist0, 1, flann::SearchParams(32, 0, false));
    int closest_node_ind = index[0][0];     // 最近点的索引号
    float closest_dist = (rand_point - tree[closest_node_ind]->joint_angle).norm();    // 最近距离(直接读取dist0不准确，未知原因)
    //std::cout << "closest_node_ind: " << closest_node_ind << "\tclosest_dist: " << closest_dist << std::endl;

    Eigen::Matrix<double, 6, 1> new_point = rand_point;
    double cost = sumCost(new_point, closest_node_ind);
    // 判断距离是否大于步长，如果大于，则需截取
    if(closest_dist > node_step){
        Eigen::Matrix<double, 6, 1>& closest_point = rrtTree::tree[closest_node_ind]->joint_angle;
        Eigen::Matrix<double, 6, 1> angle_diff = rand_point - closest_point;
        new_point = closest_point + rrtTree::node_step * angle_diff / angle_diff.norm();
        cost = sumCost(new_point, closest_node_ind);
    }

    // 构成节点
    treeNode* newNode = new treeNode(new_point, closest_node_ind, cost);
    return newNode;
}

// 碰撞检测，无碰则返回true，有碰撞则返回false
bool rrtTree::collisionDetect(Eigen::Matrix<double, 6, 1>& new_point, Eigen::Matrix<double, 6, 1>& neighbor_point) {

    Eigen::Matrix<double, 6, 1> dist_temp = new_point - neighbor_point;
    double dist = dist_temp.norm();
    Eigen::Matrix<double, 6, 1> unitVector = dist_temp / dist;     // 从临近点 指向 新节点 的 单位向量
    double step = dist / step_dive;

    Eigen::Matrix<double, 6, 1> state_angle;
    Eigen::MatrixXd joint_position;           // 每个关节末端在笛卡尔空间的位置
    Eigen::MatrixXd obs_dist;

    for(int i = 0; i < obstacles.size(); ++i){
        for(int j = 1; j <= step_dive; ++j){
            state_angle = neighbor_point + j * step * unitVector;
            joint_position = anno->fkine(state_angle);     // 计算每个关节末端在笛卡尔空间的位置
            //std::cout << "joint_position: \n" << joint_position << std::endl;
            // 计算 障碍物 与 每个关节末端位置 之间的距离
            obs_dist = (joint_position - obstacles[i]->position.replicate(1,4)).colwise().norm();
            double min_dist = 65535;
            for(int k = 0; k < 3; ++k){
                if (obs_dist(0, k) + obs_dist(0, k+1) == anno->link_length[k]){
                    return false;
                }else if (pow(obs_dist(0, k+1), 2) >= pow(obs_dist(0, k),2) + pow(anno->link_length[k], 2)){
                    if (obs_dist(0, k) < min_dist)
                        min_dist = obs_dist(0, k);
                }else if (pow(obs_dist(0, k), 2) >= pow(obs_dist(0, k+1),2) + pow(anno->link_length[k], 2)){
                    if (obs_dist(0, k+1) < min_dist)
                        min_dist = obs_dist(0, k+1);
                }else{
                    double p = (obs_dist(0, k) + obs_dist(0, k+1) + anno->link_length[k]) / 2;
                    double s = sqrt(p * (p - obs_dist(0,k)) * (p - obs_dist(0, k+1)) * (p - anno->link_length[k]));
                    double h = 2 * s / anno->link_length[k];
                    if (h < min_dist)
                        min_dist = h;
                }
            }
            min_dist = min_dist - anno->arm_radius - obstacles[i]->radius;
            //std::cout << "min_dist: " << min_dist << std::endl;
            if(min_dist < 0 || min_dist > 100){     // min_dist > 100说明计算错误
                return false;
            }
        }
    }

    return true;
}

// 从根节点到新节点的代价，笛卡尔空间的距离 和 关节空间的距离 加权求和
double rrtTree::sumCost(Eigen::Matrix<double, 6, 1>& new_point, int& neighbor_ind) {
    // 设定 笛卡尔空间的距离 和 关节空间的距离 的权重
    int w1 = 20, w2 = 80;
    Eigen::Matrix<double, 6, 1>& neighbor_point = tree[neighbor_ind]->joint_angle;

    // 正运动学求解末端在笛卡尔空间的位置
    Eigen::MatrixXd new_position = anno->fkine(new_point).col(3);
    Eigen::MatrixXd neighbor_position = anno->fkine(neighbor_point).col(3);

    double cartesian_cost = (new_position - neighbor_position).norm();         // 笛卡尔空间的距离
    double joint_cost = (new_point - neighbor_point).norm();                   // 关节空间的距离

    double sum_cost = w1 * cartesian_cost + w2 * joint_cost + tree[neighbor_ind]->node_cost;
    return sum_cost;
}

// 为新节点选择父节点，并加入搜索树
bool rrtTree::chooseParent(treeNode* newNode, std::vector<std::vector<int> >& neighbors) {
    Eigen::Matrix<double, 6, 1>& new_point = newNode->joint_angle;
    // 如果与初始父节点间有障碍物，则将父节点序号置为-1
    if(!collisionDetect(new_point, tree[newNode->parent_ind]->joint_angle))
        newNode->parent_ind = -1;

    // 在邻域内选择父节点
    for (int i = 0; i < neighbors[0].size(); ++i){
        Eigen::Matrix<double, 6, 1>& neighbor_point = tree[neighbors[0][i]]->joint_angle;
        double dist = (new_point - neighbor_point).norm();
        if( collisionDetect(new_point, neighbor_point) && (dist <= node_step) ){
            double temp_cost = rrtTree::sumCost(new_point, neighbors[0][i]);
            if (temp_cost < newNode->node_cost){
                newNode->parent_ind = neighbors[0][i];
                newNode->node_cost = temp_cost;
            }
        }
    }

    // 父节点序号大于0表明在搜索树上为该节点找到了合适的父节点
    return (newNode->parent_ind >= 0);
}

// 将节点加入搜索树和kd树
void rrtTree::insertNode(treeNode* newNode) {
    // 先找到合适的父节点
    Eigen::Matrix<double, 6, 1>& new_point = newNode->joint_angle;
    // 找到新节点邻域内的节点
    std::vector<std::vector<int> > neighbors;     // 用于存储临近点的索引号
    std::vector<std::vector<double> > dists;      // 用于存储距离临近点的距离
    flann::Matrix<double> new_kd_point(new_point.data(), 1, 6);
    kd_tree->radiusSearch(new_kd_point, neighbors, dists, node_step, flann::SearchParams(32, 0, false));

    // 如果返回false，说明没有合适的父节点，则舍弃该节点即可
    if(!chooseParent(newNode, neighbors)) return;

    // 加入搜索树和kd树
    tree.push_back(newNode);
    flann::Matrix<double> node(newNode->joint_angle.data(), 1, 6);
    kd_tree -> addPoints(node);
    int new_node_ind = tree.size() - 1;

    // 如果该节点为目标节点，则可以停止搜索
    if(newNode->joint_angle == goal_node.joint_angle) return;

    // 重新布线：新节点邻域内的节点选择新的父节点
    for (int i = 0; i < neighbors[0].size(); ++i){
        Eigen::Matrix<double, 6, 1>& neighbor_point = tree[neighbors[0][i]]->joint_angle;
        double dist = (new_point - neighbor_point).norm();
        if(collisionDetect(new_point, neighbor_point) && dist <= node_step) {
            double temp_cost = rrtTree::sumCost(neighbor_point, new_node_ind);     // 注意该函数输入参数的顺序
            if (temp_cost < rrtTree::tree[neighbors[0][i]]->node_cost){
                tree[neighbors[0][i]]->parent_ind = new_node_ind;
                tree[neighbors[0][i]]->node_cost = temp_cost;
            }
        }
    }
}

// 回溯路径
void rrtTree::findPath() {
    // path 清空
    while(!path.empty())
        path.pop();

    int ind = tree.size() - 1;
    while(ind > 0 && tree[ind]->joint_angle != root_node.joint_angle){
        path.push(rrtTree::tree[ind]);
        ind = tree[ind]->parent_ind;
    }
    path.push(&root_node);
    std::cout << "There are " << path.size() << " nodes on the path.\n" << std::endl;
}

// 路径规划过程
void rrtTree::pathPlanning() {
    // 如果初始位置与目标位置相同，则不需要规划
    if(root_node.joint_angle == goal_node.joint_angle) return;
    // 如果无障碍物，则直接连线即可
    if(obstacles.empty()){
        path.push(&goal_node);
        path.push(&root_node);
        std::cout << "Path planning successfully! There are no obstacles.\n" << std::endl;
        isSuccess = true;
        return;
    }

    // 设定随机数种子
    srand((int)time(0));          // 必须要加这一行，且必须写在循环体外，否则随机采样时生成的是伪随机数

    //开始计时
    clock_t start = clock();

    for(int i = 0; i < max_iter; ++i){
        // 随机采样，得到rand_point
        Eigen::Matrix<double, 6, 1> rand_point = samplePoint();
        // 根据步长截取得到newNode（的指针）
        treeNode* newNode = getNewPoint(rand_point);
        // 将该节点加入搜索树
        insertNode(newNode);
    }

    // 最后将根节点加入搜索树
    insertNode(&goal_node);
    if(goal_node.parent_ind < 0){
        // 说明没有为根节点匹配好父节点
        std::cout << "No Parent Node Found for the Goal Node!\n" << std::endl;
        return;
    }
    std::cout << tree.size() << " nodes has added into the tree\n";
    // 回溯路径
    findPath();

    // 计时结束
    clock_t end = clock();
    std::cout << "RRT* Running Time : " << (double) (end - start) / CLOCKS_PER_SEC << "seconds." << std::endl;
    std::cout << "The cost of the path is " << goal_node.node_cost << std::endl;

    isSuccess = true;
}

// 控制机械臂运动
void rrtTree::robotControl() {
    if(!isSuccess) return;
    // 尝试在rviz运行
    //多线程
    ros::AsyncSpinner spinner(1);
    //开启新的线程
    spinner.start();

    // 获取机械臂的起始位置，存入 start_state，将当前状态存入 joint_model_group
    moveit::core::RobotStatePtr start_state(arm.getCurrentState());
    const robot_state::JointModelGroup *joint_model_group = start_state->getJointModelGroup(arm.getName());
    std::vector<double> joint_group_positions;
    // 将当前状态拷贝给 joint_group_positions
    start_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    // 初始化路径，每个节点对应一小段路径
    std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans(path.size());
    int i = 0;

    while(!path.empty()){
        // 将路径中的每个点逐个设为目标点
        treeNode* current_node = path.top();
        joint_group_positions[0] = current_node->joint_angle(0);
        joint_group_positions[1] = current_node->joint_angle(1);
        joint_group_positions[2] = current_node->joint_angle(2);
        joint_group_positions[3] = current_node->joint_angle(3);
        joint_group_positions[4] = current_node->joint_angle(4);
        joint_group_positions[5] = current_node->joint_angle(5);
        arm.setJointValueTarget(joint_group_positions);
        //std::cout << "node_angle[" << i << "]:\n" << current_node->joint_angle << "\n" << std::endl;

        moveit::planning_interface::MoveItErrorCode success = arm.plan(plans[i]);
        ++i;

        // 更新初始状态
        joint_model_group = start_state->getJointModelGroup(arm.getName());
        start_state->setJointGroupPositions(joint_model_group, joint_group_positions);
        arm.setStartState(*start_state);

        path.pop();
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

    // moveit控制机械臂运动
    if(!arm.execute(joinedPlan)){
        ROS_ERROR("Failed to execute plan");
        return;
    }
    ROS_INFO("Simulink Finished");

    // 控制机械臂真机运动
    std::cout << "Control Real Robot" << std::endl;
    double time_start;
    double time_end;
    double time_duration;

    for(int num = 0; num < joinedPlan.trajectory_.joint_trajectory.points.size(); num++){
        // 向gazebo发送控制信号
        //speed_msg_sim.data = joinedPlan.trajectory_.joint_trajectory.points[num].velocities;
        //sim_speed_pub.publish(speed_msg_sim);

        // 向机械臂真机发送控制信号
        speed_msg.data[0] = joinedPlan.trajectory_.joint_trajectory.points[num].velocities[0] * 30 * 180 / PI;
        speed_msg.data[1] = joinedPlan.trajectory_.joint_trajectory.points[num].velocities[1] * 205 * 180 / (3 * PI);
        speed_msg.data[2] = joinedPlan.trajectory_.joint_trajectory.points[num].velocities[2] * 50 * 180 / PI;
        speed_msg.data[3] = joinedPlan.trajectory_.joint_trajectory.points[num].velocities[3] * 125 * 180 / (2 * PI);
        speed_msg.data[4] = joinedPlan.trajectory_.joint_trajectory.points[num].velocities[4] * 125 * 180 / (2 * PI);
        speed_msg.data[5] = joinedPlan.trajectory_.joint_trajectory.points[num].velocities[5] * 200 * 180 / (9 * PI);
        speed_pub.publish(speed_msg);

        time_start = joinedPlan.trajectory_.joint_trajectory.points[num].time_from_start.toSec();
        if(num < joinedPlan.trajectory_.joint_trajectory.points.size()-1){
            time_end = joinedPlan.trajectory_.joint_trajectory.points[num+1].time_from_start.toSec();
            time_duration = (time_end - time_start) * 1000000;// - 250;     //假定每个循环的时间花费250微秒
        }else{
            time_duration = 10000;
        }

        usleep(time_duration);      //微秒
    }
}


#endif