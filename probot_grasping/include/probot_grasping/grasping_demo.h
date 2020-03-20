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

#ifndef PROBOT_GRASPING_DEMO
#define PROBOT_GRASPING_DEMO

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>

#include "probot_grasping/vision_manager.h"

class GraspingDemo
{
  private:
	//@brief NodeHandle of the current node节点句柄
	ros::NodeHandle nh_;
	
        //@brief target_pose_1 is the pose to moveit!
	geometry_msgs::Pose target_pose1;

	//@brief armgroup moveit interface for arm
	//需要用move group控制的机械臂中的arm group
	moveit::planning_interface::MoveGroupInterface armgroup;

	//@brief grippegroup moveit interface for gripper
	//需要用move group控制的机械臂中的gripper group
	moveit::planning_interface::MoveGroupInterface grippergroup;

	//@brief it_ takes care of message to image conversion
	//????????????????????????????????????????????????????负责从信息到图像的转换？
	image_transport::ImageTransport it_;

	//@brief image_sub_ subscribes to image/raw topic
	//???????????????????????????????????????????????
	image_transport::Subscriber image_sub_;

	//@brief boolean to control the grasping movements判断抓取动作是否在运行
	bool grasp_running;

	//@brief cv_ptr is the pointer to image as received by it_
	//cv_ptr是it_所接收的图像的指针
        cv_bridge::CvImagePtr cv_ptr;

	//@brief vMng_ is the instance of the library for object detection
	//????????????????????????????????????????????????????????????????
	//VisionManager类，见vision_manager.h
	VisionManager vMng_;

	//@brief camera_to_robot takes care of the transformation from camera to robot frame
	//用于储存相机坐标系和机器人坐标系之间的tf数据，实质上是从相机坐标系到机器人坐标系的转换矩阵
	tf::StampedTransform camera_to_robot_;

	//@brief tf_camera_to_robot is an instance of tf_listener
	//创建相机坐标系和机器人坐标系关系的tf监听器
	tf::TransformListener tf_camera_to_robot;

	//@brief obj_camera_frame, obj_robot_frame are instance of tf::Vector
	//两个坐标系变量
	tf::Vector3 obj_camera_frame, obj_robot_frame;

	//@brief homePose is StampedPose keeping Home position of arm
	//见http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html
	//geometry_msgs::PoseStamped是具有参考坐标系和时间戳的姿态
	geometry_msgs::PoseStamped homePose;

	//@brief my_plan is an instance of moveit! planning interface
	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	//@brief pregrasp_x, pregrasp_y and pregrasp_z is the pregrasp position of arm
	//抓取之前的准备位置
	float pregrasp_x, pregrasp_y, pregrasp_z;

	/**
	 * @brief      attainPosition achieved the given position of the arm
	 *
	 * @param[in]  x     x-position of gripping frame	
	 * @param[in]  y     y-position of gripping frame	
	 * @param[in]  z     z-position of gripping frame  为什么是gripping frame？？？？？？
	 */
	//声明attainPosition函数，输入一个位置点，控制机械臂到达该位置，姿态角不变
	void attainPosition(float x, float y, float z);

	//@brief      attainObject tries to get the gripper next to object
	//控制抓手接近被抓物体
	void attainObject();

	//@brief      grasp executes the grasping action
	//进行抓的动作
	void grasp();

	//@brief      lift attempts to lift the object
	//把被抓物体提起来
	void lift();

  public:
	/**
 	 * @brief      GraspingDemo behaviour Constructor
 	 *
 	 * @param[in]  n_          ros_NodeHandle
 	 * @param[in]  pregrasp_x  Desired PregraspingX
 	 * @param[in]  pregrasp_y  Desired PregraspingY
 	 * @param[in]  pregrasp_z  Desired PregraspingZ
 	 * @param[in]  length      The length of table
 	 * @param[in]  breadth     The breadth of table  为什么声明函数时这两个先赋好值
 	 */
 	//该类的构造函数，用于给变量赋初值
	GraspingDemo(ros::NodeHandle n_, float pregrasp_x, float pregrasp_y, float pregrasp_z, float length = 1, float breadth = 0.6);

	//@brief      imageCb is called when a new image is received from the camera
	//@param[in]  msg   Image received as a message
	//接收到传感器图像信息时调用该函数
	void imageCb(const sensor_msgs::ImageConstPtr &msg);

	//@brief      initiateGrasping initiates the grasping behaviour
	//初始化抓取行为
	void initiateGrasping();

	//@brief      Function brings the arm back to home configuration
	void goHome();
};

#endif
