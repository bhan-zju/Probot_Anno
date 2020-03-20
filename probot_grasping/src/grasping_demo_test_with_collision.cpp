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

#include "probot_grasping/grasping_demo.h"

//GraspingDemo类的构造函数,给变量赋初值
GraspingDemo::GraspingDemo(ros::NodeHandle n_, float pregrasp_x, float pregrasp_y, float pregrasp_z, float length, float breadth) :
    it_(n_), 
    armgroup("manipulator"), 
    grippergroup("gripper"), 
    vMng_(length, breadth)      //没见过这种
{
  this->nh_ = n_;

  //使用tf监听器tf_camera_to_robot，获取/base_link和/camera_link坐标系之间的tf数据，时间限制50s，超出则超时报错
  try
  {
    this->tf_camera_to_robot.waitForTransform("/base_link", "/camera_link", ros::Time(0), ros::Duration(50.0));
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("[adventure_tf]: (wait) %s", ex.what());
    ros::Duration(1.0).sleep();
  }

  //查询/base_link和/camera_link坐标系之间的tf数据，并存入camera_to_robot_中，即从相机坐标系到机器人坐标系的转换矩阵
  try
  {
    this->tf_camera_to_robot.lookupTransform("/base_link", "/camera_link", ros::Time(0), (this->camera_to_robot_));
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("[adventure_tf]: (lookup) %s", ex.what());
  }

  grasp_running = false;    //为什么这里不用this->grasp_running
  
  this->pregrasp_x = pregrasp_x;
  this->pregrasp_y = pregrasp_y;
  this->pregrasp_z = pregrasp_z;

  //开启新的线程
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(5.0).sleep();

  //调用attainPosition函数，控制机械臂到pregrasp准备位置
  ROS_INFO_STREAM("Getting into the Grasping Position....");
  attainPosition(pregrasp_x, pregrasp_y, pregrasp_z);

  // Subscribe to input video feed and publish object location
  // 订阅图像话题，一旦收到图像信息，就会进入到回调函数GraspingDemo::imageCb当中
  image_sub_ = it_.subscribe("/probot_anno/camera/image_raw", 1, &GraspingDemo::imageCb, this);
}

//该函数用于根据相机所得到的图像信息，计算出目标物体在机器人坐标系中的位置，函数的输入是图像的指针
void GraspingDemo::imageCb(const sensor_msgs::ImageConstPtr &msg)
{
  if (!grasp_running)
  {
    //将ROS传感器(即相机)图像转换为opencv图像CvImage
    //参考https://blog.csdn.net/u013794793/article/details/79925491
    ROS_INFO_STREAM("Processing the Image to locate the Object...");
    try
    {
      //将传感器的图像信息的地址存储到指针cv_ptr中
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);  //规定目标CvImage的编码是BGR8
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // ROS_INFO("Image Message Received");

    //调用vision_manager中的函数获取目标的位置，该位置坐标是基于相机坐标系
    float obj_x, obj_y;
    vMng_.get2DLocation(cv_ptr->image, obj_x, obj_y);

    // Temporary Debugging临时调试
    //输出相机坐标系下目标点的坐标
    std::cout<< " X-Co-ordinate in Camera Frame :" << obj_x << std::endl;
    std::cout<< " Y-Co-ordinate in Camera Frame :" << obj_y << std::endl;

    //通过坐标变换，将二维坐标变换为相机坐标系下的三维坐标，在本程序中与URDF建模有关系
    obj_camera_frame.setZ(-obj_y);
    obj_camera_frame.setY(-obj_x);
    obj_camera_frame.setX(0.45);

    obj_robot_frame = camera_to_robot_ * obj_camera_frame;
    grasp_running = true;

    // Temporary Debugging
    std::cout<< " X-Co-ordinate in Robot Frame :" << obj_robot_frame.getX() << std::endl;
    std::cout<< " Y-Co-ordinate in Robot Frame :" << obj_robot_frame.getY() << std::endl;
    std::cout<< " Z-Co-ordinate in Robot Frame :" << obj_robot_frame.getZ() << std::endl;
  }
}

//GraspingDemo类中的attainPosition函数，输入一个位置坐标，控制机械臂末端运动到该位置，并保持原有的姿态角
void GraspingDemo::attainPosition(float x, float y, float z)
{
  // ROS_INFO("The attain position function called");

  //？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？？不懂
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link_2");
  visual_tools.deleteAllMarkers();

  // For getting the pose获取当前位置
  geometry_msgs::PoseStamped currPose = armgroup.getCurrentPose();

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation = currPose.pose.orientation;

  // Starting Postion before picking
  target_pose1.position.x = x;
  target_pose1.position.y = y;
  target_pose1.position.z = z;
  armgroup.setPoseTarget(target_pose1);

  /* Uncomment Following section to visualize in rviz  在rviz中取消对下面部分的注释以可视化 */
  // We can print the name of the reference frame for this robot.
  // ROS_INFO("Reference frame: %s", armgroup.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  // ROS_INFO("Reference end-effector frame: %s", armgroup.getEndEffectorLink().c_str());

  // ROS_INFO("Group names: %s",  armgroup.getName().c_str());

  /*ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

   const robot_state::JointModelGroup *joint_model_group =
  armgroup.getCurrentState()->getJointModelGroup("arm");

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  // visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");*/

  armgroup.move();
}

void GraspingDemo::attainObject()
{
  // ROS_INFO("The attain Object function called");
  //先控制机械臂运动到物体上方0.04m处
  attainPosition(obj_robot_frame.getX(), obj_robot_frame.getY(), obj_robot_frame.getZ() + 0.06);

  // Open Gripper打开夹爪
  ros::WallDuration(1.0).sleep();
  grippergroup.setNamedTarget("open");
  grippergroup.move();

  // Slide down the Object
  //控制机械臂到达抓取位姿
  geometry_msgs::PoseStamped currPose = armgroup.getCurrentPose();
  geometry_msgs::Pose target_pose1;

  target_pose1.orientation = currPose.pose.orientation;
  target_pose1.position = currPose.pose.position;

  target_pose1.position.z = obj_robot_frame.getZ() - 0.03;
  armgroup.setPoseTarget(target_pose1);
  armgroup.move();
}

void GraspingDemo::grasp()
{
  // ROS_INFO("The Grasping function called");

  ros::WallDuration(1.0).sleep();
  grippergroup.setNamedTarget("close");
  grippergroup.move();
}

void GraspingDemo::lift()
{
  // ROS_INFO("The lift function called");

  // For getting the pose  获取当前位姿
  geometry_msgs::PoseStamped currPose = armgroup.getCurrentPose();

  geometry_msgs::Pose target_pose1;
  target_pose1.orientation = currPose.pose.orientation;
  target_pose1.position = currPose.pose.position;

  // Starting Postion after picking

  target_pose1.position.z = target_pose1.position.z + 0.10;
  armgroup.setPoseTarget(target_pose1);
  armgroup.move();
  ros::WallDuration(1.0).sleep();

  target_pose1.position.y = target_pose1.position.y + 0.35;
  target_pose1.position.x = target_pose1.position.x - 0.10;
  target_pose1.position.z = target_pose1.position.z - 0.05;
  armgroup.setPoseTarget(target_pose1);
  armgroup.move();
  ros::WallDuration(1.0).sleep();

  target_pose1.position.z = target_pose1.position.z - 0.10;
  
  armgroup.setPoseTarget(target_pose1);
  armgroup.move();

  // Open Gripper
  ros::WallDuration(1.0).sleep();
  grippergroup.setNamedTarget("open");
  grippergroup.move();

 
  target_pose1.position.z = target_pose1.position.z + 0.06;
  armgroup.setPoseTarget(target_pose1);
  armgroup.move();
}

void GraspingDemo::goHome()
{
  geometry_msgs::PoseStamped currPose = armgroup.getCurrentPose();

  // Go to Home Position
  attainPosition(pregrasp_x, pregrasp_y, pregrasp_z);
  attainPosition(homePose.pose.position.x, homePose.pose.position.y, homePose.pose.position.z);
}

void GraspingDemo::initiateGrasping()
{
  //开启一个新的线程
  ros::AsyncSpinner spinner(1);
  spinner.start();
  //等待3秒
  ros::WallDuration(3.0).sleep();

  //获取当前姿态
  homePose = armgroup.getCurrentPose();
  
  //移动机械臂到抓取位置
  ROS_INFO_STREAM("Approaching the Object....");
  attainObject();

  //抓取物体
  ROS_INFO_STREAM("Attempting to Grasp the Object now..");
  grasp();

  ROS_INFO_STREAM("Lifting the Object....");
  lift();

  //这里的home实际上是pregrasp位置
  ROS_INFO_STREAM("Going back to home position....");
  goHome();

  grasp_running = false;
}

//添加障碍物描述
void add_collision(ros::NodeHandle nh)
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
/*
    // 声明一个障碍物体
    moveit_msgs::CollisionObject table_box;
    table_box.id = "table_box_model";
    table_box.header.frame_id = "base_link";

    // 设置障碍物的外形、尺寸等属性 
    // shape_msgs::SolidPrimitive类型详见http://docs.ros.org/api/shape_msgs/html/msg/SolidPrimitive.html  
    shape_msgs::SolidPrimitive table_box_primitive;
    table_box_primitive.type = table_box_primitive.BOX;
    //add_object_primitive.dimensions是一个栈区
    table_box_primitive.dimensions.resize(3);   //栈区大小调整为3
    table_box_primitive.dimensions[0] = 0.30;  
    table_box_primitive.dimensions[1] = 0.30;
    table_box_primitive.dimensions[2] = 0.20;

    // 设置障碍物的位置
    geometry_msgs::Pose table_box_pose;
    table_box_pose.position.x =  0.40;
    table_box_pose.position.y =  0.0;
    table_box_pose.position.z =  0.1;

    // 将障碍物的属性、位置加入到障碍物的实例中
    table_box.primitives.push_back(table_box_primitive);
    table_box.primitive_poses.push_back(table_box_pose);
    table_box.operation = table_box.ADD;   //对于该物体的操作是添加到场景
*/
// 
    moveit_msgs::CollisionObject table_box2;
    table_box2.id = "table_box2_model";
    table_box2.header.frame_id = "base_link";

    shape_msgs::SolidPrimitive table_box2_primitive;
    table_box2_primitive.type = table_box2_primitive.BOX;

    table_box2_primitive.dimensions.resize(3); 
    table_box2_primitive.dimensions[0] = 0.30;  
    table_box2_primitive.dimensions[1] = 0.30;
    table_box2_primitive.dimensions[2] = 0.10;

    geometry_msgs::Pose table_box2_pose;
    table_box2_pose.position.x =  0.25;
    table_box2_pose.position.y =  0.35;
    table_box2_pose.position.z =  0.05;

    table_box2.primitives.push_back(table_box2_primitive);
    table_box2.primitive_poses.push_back(table_box2_pose);
    table_box2.operation = table_box2.ADD;

// 
    moveit_msgs::CollisionObject collision_box;
    collision_box.id = "collision_box_model";
    collision_box.header.frame_id = "base_link";

    shape_msgs::SolidPrimitive collision_box_primitive;
    collision_box_primitive.type = collision_box_primitive.BOX;

    collision_box_primitive.dimensions.resize(3); 
    collision_box_primitive.dimensions[0] = 0.30;  
    collision_box_primitive.dimensions[1] = 0.02;
    collision_box_primitive.dimensions[2] = 0.4;

    geometry_msgs::Pose collision_box_pose;
    collision_box_pose.position.x =  0.45;
    collision_box_pose.position.y =  0.17;
    collision_box_pose.position.z =  0.2;

    collision_box.primitives.push_back(collision_box_primitive);
    collision_box.primitive_poses.push_back(collision_box_pose);
    collision_box.operation = collision_box.ADD;

//
    moveit_msgs::CollisionObject table_wall1;
    table_wall1.id = "table_wall1_model";
    table_wall1.header.frame_id = "base_link";

    shape_msgs::SolidPrimitive table_wall1_primitive;
    table_wall1_primitive.type = table_wall1_primitive.BOX;

    table_wall1_primitive.dimensions.resize(3);  
    table_wall1_primitive.dimensions[0] = 0.30;  
    table_wall1_primitive.dimensions[1] = 0.01;
    table_wall1_primitive.dimensions[2] = 0.08;

    geometry_msgs::Pose table_wall1_pose;
    table_wall1_pose.position.x =  0.25;
    table_wall1_pose.position.y =  0.495;
    table_wall1_pose.position.z =  0.14;

    table_wall1.primitives.push_back(table_wall1_primitive);
    table_wall1.primitive_poses.push_back(table_wall1_pose);
    table_wall1.operation = table_wall1.ADD;  

//
    moveit_msgs::CollisionObject table_wall2;
    table_wall2.id = "table_wall2_model";
    table_wall2.header.frame_id = "base_link";

    shape_msgs::SolidPrimitive table_wall2_primitive;
    table_wall2_primitive.type = table_wall2_primitive.BOX;

    table_wall2_primitive.dimensions.resize(3);  
    table_wall2_primitive.dimensions[0] = 0.30;  
    table_wall2_primitive.dimensions[1] = 0.01;
    table_wall2_primitive.dimensions[2] = 0.08;

    geometry_msgs::Pose table_wall2_pose;
    table_wall2_pose.position.x =  0.25;
    table_wall2_pose.position.y =  0.205;
    table_wall2_pose.position.z =  0.14;

    table_wall2.primitives.push_back(table_wall2_primitive);
    table_wall2.primitive_poses.push_back(table_wall2_pose);
    table_wall2.operation = table_wall2.ADD;  

//
    moveit_msgs::CollisionObject table_wall3;
    table_wall3.id = "table_wall3_model";
    table_wall3.header.frame_id = "base_link";

    shape_msgs::SolidPrimitive table_wall3_primitive;
    table_wall3_primitive.type = table_wall3_primitive.BOX;

    table_wall3_primitive.dimensions.resize(3);  
    table_wall3_primitive.dimensions[0] = 0.01;  
    table_wall3_primitive.dimensions[1] = 0.28;
    table_wall3_primitive.dimensions[2] = 0.08;

    geometry_msgs::Pose table_wall3_pose;
    table_wall3_pose.position.x =  0.105;
    table_wall3_pose.position.y =  0.35;
    table_wall3_pose.position.z =  0.14;

    table_wall3.primitives.push_back(table_wall3_primitive);
    table_wall3.primitive_poses.push_back(table_wall3_pose);
    table_wall3.operation = table_wall3.ADD; 

//
    moveit_msgs::CollisionObject table_wall4;
    table_wall4.id = "table_wall4_model";
    table_wall4.header.frame_id = "base_link";

    shape_msgs::SolidPrimitive table_wall4_primitive;
    table_wall4_primitive.type = table_wall4_primitive.BOX;

    table_wall4_primitive.dimensions.resize(3);  
    table_wall4_primitive.dimensions[0] = 0.01;  
    table_wall4_primitive.dimensions[1] = 0.28;
    table_wall4_primitive.dimensions[2] = 0.08;

    geometry_msgs::Pose table_wall4_pose;
    table_wall4_pose.position.x =  0.395;
    table_wall4_pose.position.y =  0.35;
    table_wall4_pose.position.z =  0.14;

    table_wall4.primitives.push_back(table_wall4_primitive);
    table_wall4.primitive_poses.push_back(table_wall4_pose);
    table_wall4.operation = table_wall4.ADD; 

    // 所有障碍物加入列表后，再把障碍物加入到当前的情景中
//    planning_scene.world.collision_objects.push_back(table_box);
    planning_scene.world.collision_objects.push_back(table_box2);
    planning_scene.world.collision_objects.push_back(collision_box);
    planning_scene.world.collision_objects.push_back(table_wall1);
    planning_scene.world.collision_objects.push_back(table_wall2);
    planning_scene.world.collision_objects.push_back(table_wall3);
    planning_scene.world.collision_objects.push_back(table_wall4);
    
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);
}

int main(int argc, char **argv)
{
  //初始化节点
  ros::init(argc, argv, "simple_grasping");

  //声明一些初始参数，用于存储桌子的长、宽，机械臂的初始位置
  float length, breadth, pregrasp_x, pregrasp_y, pregrasp_z;

  //创建节点句柄
  ros::NodeHandle n;

  //将桌子的长、宽，机械臂的初始位置赋值给length, breadth, pregrasp_x, pregrasp_y, pregrasp_z
  //getParam是从启动的launch文件中取得这些参数，如果没有，则使用自己的赋值
  if (!n.getParam("probot_grasping/table_length", length))
    length = 0.3;
  if (!n.getParam("probot_grasping/table_breadth", breadth))
    breadth = 0.3;
  if (!n.getParam("probot_grasping/pregrasp_x", pregrasp_x))
    pregrasp_x = 0.20;
  if (!n.getParam("probot_grasping/pregrasp_y", pregrasp_y))
    pregrasp_y = -0.17;
  if (!n.getParam("probot_grasping/pregrasp_z", pregrasp_z))
    pregrasp_z = 0.28;
  //添加障碍物描述
  add_collision(n);
  ros::WallDuration(2.0).sleep();

  //定义一个GraspingDemo类型的变量simGrasp
  GraspingDemo simGrasp(n, pregrasp_x, pregrasp_y, pregrasp_z, length, breadth);
  ROS_INFO_STREAM("Waiting for five seconds..");
  //等待5秒
  ros::WallDuration(5.0).sleep();


  // Process image callback  检测到消息，则调用simGrasp的initiateGrasping()函数
  ros::spinOnce();

  simGrasp.initiateGrasping();

  return 0;
}
