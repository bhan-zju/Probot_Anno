//
// Created by bhan on 2019/12/27.
//
#include <string>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "ikfast.h"
#include "probot_anno_manipulator_ikfast_moveit_plugin.cpp"
#include <time.h>
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include <sstream>

#define pi 3.1416

bool Feedback = 1;
using namespace ikfast_kinematics_plugin;
void Feed_back(const std_msgs::String::ConstPtr& Feedback_msg)
{
    Feedback = 0;
}

//添加障碍物描述
void add_broad(ros::NodeHandle nh)
{
//创建一个发布场景变化信息的发布者planning_scene_diff_publisher
    //发布名为planning_scene的话题，消息类型为moveit_msgs::PlanningScene，队列长度为１
    ros::Publisher planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    //ros::WallDuration是时间间隔类型变量，设置一个0.5s的时间间隔变量sleep_t
    ros::WallDuration sleep_t(0.5);

    //planning_scene_diff_publisher.getNumSubscribers()是订阅者的数量，如果没有订阅者，则等待0.5s
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        sleep_t.sleep();
    }


    // 创建运动规划的场景planning_scene
    moveit_msgs::PlanningScene planning_scene;

    // 声明一个障碍物体
    moveit_msgs::CollisionObject broad1;
    broad1.id = "broad1_model";
    broad1.header.frame_id = "base_link";

    // 设置障碍物的外形、尺寸等属性
    // shape_msgs::SolidPrimitive类型详见http://docs.ros.org/api/shape_msgs/html/msg/SolidPrimitive.html
    shape_msgs::SolidPrimitive broad1_primitive;
    broad1_primitive.type = broad1_primitive.BOX;
    //add_object_primitive.dimensions是一个栈区
    broad1_primitive.dimensions.resize(3);
    //障碍物尺寸大小
    broad1_primitive.dimensions[0] = 0.40;
    broad1_primitive.dimensions[1] = 0.05;
    broad1_primitive.dimensions[2] = 0.40;

    // 设置障碍物的位置姿态
    geometry_msgs::Pose broad1_pose;
    broad1_pose.position.x =  0.35;
    broad1_pose.position.y =  0.175;
    broad1_pose.position.z =  0.20;

    // 将障碍物的属性、位置加入到障碍物的实例中
    broad1.primitives.push_back(broad1_primitive);
    broad1.primitive_poses.push_back(broad1_pose);
    broad1.operation = broad1.ADD;

//
    moveit_msgs::CollisionObject broad2;
    broad2.id = "broad2_model";
    broad2.header.frame_id = "base_link";

    shape_msgs::SolidPrimitive broad2_primitive;
    broad2_primitive.type = broad2_primitive.BOX;

    broad2_primitive.dimensions.resize(3);
    broad2_primitive.dimensions[0] = 0.40;
    broad2_primitive.dimensions[1] = 0.05;
    broad2_primitive.dimensions[2] = 0.40;

    geometry_msgs::Pose broad2_pose;
    broad2_pose.position.x =  0.35;
    broad2_pose.position.y =  -0.175;
    broad2_pose.position.z =  0.20;

    broad2.primitives.push_back(broad2_primitive);
    broad2.primitive_poses.push_back(broad2_pose);
    broad2.operation = broad2.ADD;

    // 所有障碍物加入列表后，再把障碍物加入到当前的情景中
    planning_scene.world.collision_objects.push_back(broad1);
    planning_scene.world.collision_objects.push_back(broad2);

    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);
}

int main(int argc, char **argv) {

    bool ret;
    ros::init(argc, argv, "collision_avoid");
    ros::NodeHandle node_handle;

    //添加障碍物描述
    add_broad(node_handle);
    sleep(5);
    std::cout << "Scene has updated!" << std::endl;

    //创建一个发布者Float_chatter_pub，发布消息类型为std_msgs::Float32MultiArray，话题名为Float_chatter，用于发布速度
    ros::Publisher speed_pub = node_handle.advertise<std_msgs::Float32MultiArray>("speed_chatter", 1000);
    //创建一个发布者Float64_chatter_pub，发布消息类型为std_msgs::Float64MultiArray，话题名为Float64_chatter，用于发布位置
    ros::Publisher position_pub = node_handle.advertise<std_msgs::Float32MultiArray>("position_chatter", 1000);
    //创建一个订阅者Feedback_sub，接收话题"Feedback_chatter"，回调函数为Feed_back
    ros::Subscriber Feedback_sub = node_handle.subscribe("Feedback_chatter",100,Feed_back);

    ros::AsyncSpinner spinner(1);


    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    //设置机械臂的允许误差值（单位：弧度）(这一步也可以不设置，不影响运行)
    arm.setGoalJointTolerance(0.001);

    //设置最大加速度和最大速度(这一步也可以不设置，不影响运行)
    arm.setMaxAccelerationScalingFactor(0.5);
    arm.setMaxVelocityScalingFactor(0.5);

    //设置目标位置所使用的参考坐标系
    std::string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);

    //当运动规划失败后，允许重新规划
    arm.allowReplanning(true);

    // rviz中控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

    //读出初始位置末端的位姿
    geometry_msgs::PoseStamped pose_solved;
    pose_solved = arm.getCurrentPose("tool0");       //arm.getCurrentPose()返回geometry_msgs::PoseStamped类型的变量
    ROS_INFO_STREAM("start: " << pose_solved);          //输出pose_solved的所有信息

    std_msgs::Float32MultiArray speed_msg;    //速度
    speed_msg.data.push_back(0.0);
    speed_msg.data.push_back(0.0);
    speed_msg.data.push_back(0.0);
    speed_msg.data.push_back(0.0);
    speed_msg.data.push_back(0.0);
    speed_msg.data.push_back(0.0);

    std_msgs::Float32MultiArray position_msg;   //位置
    position_msg.data.push_back(0.0);
    position_msg.data.push_back(0.0);
    position_msg.data.push_back(0.0);
    position_msg.data.push_back(0.0);
    position_msg.data.push_back(0.0);
    position_msg.data.push_back(0.0);
    position_msg.data.push_back(0.0);


    position_pub.publish(position_msg);     //将初始化位置发送出去，用于控制器订阅
    sleep(5);


    // 设置机器人终端的目标位置
    geometry_msgs::Pose target_pose1;
    target_pose1.position.x = 0.00;
    target_pose1.position.y = 0.28;
    target_pose1.position.z = 0.20;
    target_pose1.orientation.x = 1;

    // 设置机器臂当前的状态作为运动初始状态
    arm.setStartStateToCurrentState();

    arm.setPoseTarget(target_pose1);


    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    //进行运动规划，成功则返回true
    moveit::planning_interface::MoveItErrorCode success1 = arm.plan(plan1);
    ROS_INFO("Plan (pose goal) %s",success1?"":"FAILED");

    //在rviz中让机械臂按照规划的轨迹开始运动
    if(success1)
        arm.execute(plan1);
    sleep(1);


//    int point_num = plan.trajectory_.joint_trajectory.points.size();
    double time_start;
    double time_end;
    double time_duration;

    //开始计时
    clock_t start = clock();

    for(int n = 0; n < plan1.trajectory_.joint_trajectory.points.size(); n++){

        //clock_t startn = clock();
        speed_msg.data.at(0) = plan1.trajectory_.joint_trajectory.points[n].velocities[0] * 30 * 180 / pi;
        speed_msg.data.at(1) = plan1.trajectory_.joint_trajectory.points[n].velocities[1] * 205 * 180 / (3 * pi);
        speed_msg.data.at(2) = plan1.trajectory_.joint_trajectory.points[n].velocities[2] * 50 * 180 / pi;
        speed_msg.data.at(3) = plan1.trajectory_.joint_trajectory.points[n].velocities[3] * 125 * 180 / (2 * pi);
        speed_msg.data.at(4) = plan1.trajectory_.joint_trajectory.points[n].velocities[4] * 125 * 180 / (2 * pi);
        speed_msg.data.at(5) = plan1.trajectory_.joint_trajectory.points[n].velocities[5] * 200 * 180 / (9 * pi);
        speed_pub.publish(speed_msg);

        time_start = plan1.trajectory_.joint_trajectory.points[n].time_from_start.toSec();
        if(n < plan1.trajectory_.joint_trajectory.points.size()-1){
            time_end = plan1.trajectory_.joint_trajectory.points[n+1].time_from_start.toSec();
            time_duration = (time_end - time_start) * 1000000 - 250;
        }else{
            time_duration = 10000;
        }

        usleep(time_duration);      //微秒

        //clock_t endsn = clock();
        //cout << "Running Time n : " << (double) 1000 * (endsn - startn) / CLOCKS_PER_SEC << endl;
    }

    sleep(2);

    pose_solved = arm.getCurrentPose("tool0");//get end position
    ROS_INFO_STREAM("END: " << pose_solved);

    // 设置机器人终端的目标位置
    geometry_msgs::Pose target_pose2;
    target_pose2.position.x = 0.30;
    target_pose2.position.y = 0.0;
    target_pose2.position.z = 0.20;
    target_pose2.orientation.x = 1;

    // 设置机器臂当前的状态作为运动初始状态
    arm.setStartStateToCurrentState();

    arm.setPoseTarget(target_pose2);


    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    //进行运动规划，成功则返回true
    moveit::planning_interface::MoveItErrorCode success2 = arm.plan(plan2);
    ROS_INFO("Plan (pose goal) %s",success2?"":"FAILED");

    //在rviz中让机械臂按照规划的轨迹开始运动
    if(success2)
        arm.execute(plan2);
    sleep(1);

    for(int n = 0; n < plan2.trajectory_.joint_trajectory.points.size(); n++){

        //clock_t startn = clock();
        speed_msg.data.at(0) = plan2.trajectory_.joint_trajectory.points[n].velocities[0] * 30 * 180 / pi;
        speed_msg.data.at(1) = plan2.trajectory_.joint_trajectory.points[n].velocities[1] * 205 * 180 / (3 * pi);
        speed_msg.data.at(2) = plan2.trajectory_.joint_trajectory.points[n].velocities[2] * 50 * 180 / pi;
        speed_msg.data.at(3) = plan2.trajectory_.joint_trajectory.points[n].velocities[3] * 125 * 180 / (2 * pi);
        speed_msg.data.at(4) = plan2.trajectory_.joint_trajectory.points[n].velocities[4] * 125 * 180 / (2 * pi);
        speed_msg.data.at(5) = plan2.trajectory_.joint_trajectory.points[n].velocities[5] * 200 * 180 / (9 * pi);
        speed_pub.publish(speed_msg);

        time_start = plan2.trajectory_.joint_trajectory.points[n].time_from_start.toSec();
        if(n < plan2.trajectory_.joint_trajectory.points.size()-1){
            time_end = plan2.trajectory_.joint_trajectory.points[n+1].time_from_start.toSec();
            time_duration = (time_end - time_start) * 1000000 - 250;
        }else{
            time_duration = 10000;
        }

        usleep(time_duration);      //微秒

        //clock_t endsn = clock();
        //cout << "Running Time n : " << (double) 1000 * (endsn - startn) / CLOCKS_PER_SEC << endl;
    }

    sleep(2);

    pose_solved = arm.getCurrentPose("tool0");//get end position
    ROS_INFO_STREAM("END: " << pose_solved);

    // 设置机器人终端的目标位置
    geometry_msgs::Pose target_pose3;
    target_pose3.position.x = 0.00;
    target_pose3.position.y = -0.30;
    target_pose3.position.z = 0.20;
    target_pose3.orientation.x = 1;

    // 设置机器臂当前的状态作为运动初始状态
    arm.setStartStateToCurrentState();

    arm.setPoseTarget(target_pose3);


    moveit::planning_interface::MoveGroupInterface::Plan plan3;
    //进行运动规划，成功则返回true
    moveit::planning_interface::MoveItErrorCode success3 = arm.plan(plan3);
    ROS_INFO("Plan (pose goal) %s",success2?"":"FAILED");

    //在rviz中让机械臂按照规划的轨迹开始运动
    if(success3)
        arm.execute(plan3);
    sleep(1);

    for(int n = 0; n < plan3.trajectory_.joint_trajectory.points.size(); n++){

        //clock_t startn = clock();
        speed_msg.data.at(0) = plan3.trajectory_.joint_trajectory.points[n].velocities[0] * 30 * 180 / pi;
        speed_msg.data.at(1) = plan3.trajectory_.joint_trajectory.points[n].velocities[1] * 205 * 180 / (3 * pi);
        speed_msg.data.at(2) = plan3.trajectory_.joint_trajectory.points[n].velocities[2] * 50 * 180 / pi;
        speed_msg.data.at(3) = plan3.trajectory_.joint_trajectory.points[n].velocities[3] * 125 * 180 / (2 * pi);
        speed_msg.data.at(4) = plan3.trajectory_.joint_trajectory.points[n].velocities[4] * 125 * 180 / (2 * pi);
        speed_msg.data.at(5) = plan3.trajectory_.joint_trajectory.points[n].velocities[5] * 200 * 180 / (9 * pi);
        speed_pub.publish(speed_msg);

        time_start = plan3.trajectory_.joint_trajectory.points[n].time_from_start.toSec();
        if(n < plan3.trajectory_.joint_trajectory.points.size()-1){
            time_end = plan3.trajectory_.joint_trajectory.points[n+1].time_from_start.toSec();
            time_duration = (time_end - time_start) * 1000000 - 250;
        }else{
            time_duration = 10000;
        }

        usleep(time_duration);      //微秒

        //clock_t endsn = clock();
        //cout << "Running Time n : " << (double) 1000 * (endsn - startn) / CLOCKS_PER_SEC << endl;
    }


    clock_t ends = clock();
    cout << "Running Time : " << (double) 1000 * (ends - start) / CLOCKS_PER_SEC << endl;

    sleep(5);

    pose_solved = arm.getCurrentPose("tool0");//get end position
    ROS_INFO_STREAM("END: " << pose_solved);


}

