/***********************************************************************
Copyright 2019 Wuhan PS-Micro Technology Co., Itd.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/OrientationConstraint.h>

int main(int argc, char **argv)
{
    //初始化节点
    ros::init(argc, argv, "moveit_revise_trajectory_demo");

    //创建节点句柄(后面没用到呀)
    ros::NodeHandle node_handle; 

    //多线程
    ros::AsyncSpinner spinner(1);
    //开启新的线程
    spinner.start();

    //初始化需要使用move group控制的机械臂中的arm group
    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    //设置机械臂关节角的允许误差值
    arm.setGoalJointTolerance(0.001);

    //设置最大速度和最大加速度
    double accScale = 0.5;
    double velScale = 0.5;
    arm.setMaxAccelerationScalingFactor(accScale);
    arm.setMaxVelocityScalingFactor(velScale);

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

    // 获取机器人的起始位置
    //初始化存放机械臂状态的变量的指针start_state，指向当前状态
    moveit::core::RobotStatePtr start_state(arm.getCurrentState());
    //初始化指针变量joint_model_group，指向start_state->getJointModelGroup(arm.getName())
    //arm.getName就是"manipulator"，joint_model_group指向机械臂的模型状态
    const robot_state::JointModelGroup *joint_model_group = start_state->getJointModelGroup(arm.getName());

    //声明栈区joint_group_positions
    std::vector<double> joint_group_positions;

    //将机械臂的初始状态[0 0 0 0 0 0]赋值给joint_group_positions
    start_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    //设置第一个目标点[-0.6 0 0 0 0 0]
    joint_group_positions[0] = -0.6;  // radians弧度
    arm.setJointValueTarget(joint_group_positions);

    // 计算第一条轨迹
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    moveit::planning_interface::MoveItErrorCode success = arm.plan(plan1);
    //将运行完plan1后的状态设为新的初始状态
    joint_model_group = start_state->getJointModelGroup(arm.getName());    
    start_state->setJointGroupPositions(joint_model_group, joint_group_positions);
    arm.setStartState(*start_state);

    //设置第二个目标点[-1.2 -0.5 0 0 0 0]
    joint_group_positions[0] = -1.2;  // radians
    joint_group_positions[1] = -0.5;  // radians
    arm.setJointValueTarget(joint_group_positions);

    // 计算第二条轨迹
    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    success = arm.plan(plan2);
    //将运行完plan2后的状态设为新的初始状态
    joint_model_group = start_state->getJointModelGroup(arm.getName());    
    start_state->setJointGroupPositions(joint_model_group, joint_group_positions);
    arm.setStartState(*start_state);

    //连接两条轨迹
    //初始化轨迹变量trajectory
    moveit_msgs::RobotTrajectory trajectory;

    //将plan1规划的轨迹存入trajectory
    trajectory.joint_trajectory.joint_names = plan1.trajectory_.joint_trajectory.joint_names;
    trajectory.joint_trajectory.points = plan1.trajectory_.joint_trajectory.points;

    //将plan2规划的轨迹存入trajectory
    for (size_t j = 1; j < plan2.trajectory_.joint_trajectory.points.size(); j++)
    {
        trajectory.joint_trajectory.points.push_back(plan2.trajectory_.joint_trajectory.points[j]);
    }

    //重新规划的过程，不懂，该过程可以将两条轨迹连接成一条连续的轨迹
    moveit::planning_interface::MoveGroupInterface::Plan joinedPlan;    //定义一个新的规划轨迹joinedPlan
    robot_trajectory::RobotTrajectory rt(arm.getCurrentState()->getRobotModel(), "manipulator");
    rt.setRobotTrajectoryMsg(*arm.getCurrentState(), trajectory);
    trajectory_processing::IterativeParabolicTimeParameterization iptp;
    iptp.computeTimeStamps(rt, velScale, accScale);

    //将rt中的轨迹信息存入trjectory
    rt.getRobotTrajectoryMsg(trajectory);
    joinedPlan.trajectory_ = trajectory;
    //ROS_INFO_STREAM(joinedPlan.trajectory_.joint_trajectory);

    //运行轨迹
    if (!arm.execute(joinedPlan))
    {
        ROS_ERROR("Failed to execute plan");
        return false;
    }

    sleep(1);

    ROS_INFO("Finished");

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

    ros::shutdown(); 

    return 0;
}
