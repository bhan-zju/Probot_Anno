// 求解逆运动学
// 该代码文件是第一版，向控制器传输信号的方式现已不再适用

#include <string>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include "ikfast.h"
#include "probot_anno_manipulator_ikfast_moveit_plugin.cpp"
#include "std_msgs/String.h"
#include <iostream>

using namespace ikfast_kinematics_plugin;
#define pi 3.142

//定义一个全局变量call，初始值为true
bool call = true;


//订阅者sub的回调函数
void chatterback(const std_msgs::String::ConstPtr& msg)
{
    call = false;
    string show = msg->data;
    std::cout << show << std::endl;
}

int main(int argc, char **argv)
{
    //初始化
    bool ret;
    ros::init(argc, argv, "ik");
    ros::NodeHandle node_handle; 
    ros::AsyncSpinner spinner(1);

    //创建一个发布速度指令的发布者，发布消息类型为 std_msgs::String，是一个字符串变量
    ros::Publisher speed_code_pub = node_handle.advertise<std_msgs::String>("speed_code",100);
    //创建一个订阅者，收到名为 chatter 的 topic 时进入回调函数chatterback
    ros::Subscriber sub = node_handle.subscribe("chatter",100,chatterback);
    std::stringstream sol_send;      //创建一个字符串流，用法见https://blog.csdn.net/shs1992shs/article/details/83051298
    std_msgs::String machinecode;     //

    spinner.start();
    moveit::planning_interface::MoveGroupInterface arm("manipulator");
     //设置目标位置所使用的参考坐标系
    std::string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);

    //创建运动学类实例
    IKFastKinematicsPlugin ik;
    //初始化对象：机器人模型、规划组、参考坐标、终端link、0.01是啥?
    ret = ik.IKFastKinematicsPlugin::initialize("robot_description","manipulator","base_link",arm.getEndEffectorLink(),0.01);
    // 设置机器人终端的目标位置
    geometry_msgs::Pose target_pose;
    target_pose.position.x = 0.2683;
    target_pose.position.y = 0.1;
    target_pose.position.z = 0.1387;
    target_pose.orientation.x = 1.0;

    /*此为门位置（笛卡尔空间运动准备位置）位姿
    target_pose.position.x = 0.22886;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.432;
    target_pose.orientation.x = 1.0;
     */

    //pose1为vector变量，装入一个位姿
    std::vector<geometry_msgs::Pose> pose1;
    pose1.push_back(target_pose);

    //设定参考点
    std::vector<double> seed1;
    seed1.push_back(0.0);
    seed1.push_back(0.0);
    seed1.push_back(0.0);
    seed1.push_back(0.0);
    seed1.push_back(0.0);
    seed1.push_back(0.0);
    //std::vector<std::vector<double> > sol_rad(1,vector<double>(6,0));
    //std::vector<std::vector<double> > sol_ang(1,vector<double>(6,0));
    //std::vector<std::vector<int> > sol_int_ang(1,vector<int>(6,0));

    std::vector<std::vector<double>> sol_rad;      //二维vector，逆解的变量，弧度制

    kinematics::KinematicsResult kinematic_result;

    //计算逆解，存入sol_rad，计算成功则ret=true    seed1是用来？
    ret = ik.getPositionIK(pose1, seed1, sol_rad, kinematic_result, kinematics::KinematicsQueryOptions());
    double sol_rad_export[sol_rad.size()][6];     //sol_rad.size()是计算出逆解的数量
    double sol_ang[sol_rad.size()][6];//逆解的变量，角度制
    int sol_int_ang[sol_rad.size()][6];//将角度取整
    std::string sol_string_ang[sol_rad.size()][6];;//需要发送的字符
    /*
    sol_ang[0][0] =45;
    sol_ang[0][1] =0;
    sol_ang[0][2] =0;
    sol_ang[0][3] =0;
    sol_ang[0][4] =0;
    sol_ang[0][5] =0;*/

    if(ret)//
    {
        std::cout << "IK solved successfully." << sol_rad.size() << std::endl;

        //角度制计算，并输出计算结果，rviz显示
        //将求解出来的弧度制逆解存入double型二维数组sol_rad_export
        for(int q = 0; q < sol_rad.size(); q++)
        {
            if (!sol_rad.empty())
            {
                memcpy(sol_rad_export[q], &sol_rad[q][0], sol_rad[0].size() * sizeof(double));
            }
        }
        //
        for(int i = 0; i < sol_rad.size();++i)
        {
            std::cout << "SOLUTION" << i << ":" << std::endl;

            for (int j = 0; j < 6; ++j)
            {
                //std::cout << " " << sol_rad[i][j];
                ROS_INFO_STREAM(" " << sol_rad[i][j]);         //ROS_INFO_STREAM和ROS_INFO的区别？？？
                sol_ang[i][j] = sol_rad_export[i][j] * 180 / pi;    //将逆解转换成角度制赋值给sol_ang
                std::cout << " " << sol_ang[i][j] << std::endl;
                sol_int_ang[i][j] = (int)(sol_ang[i][j]);     //将逆解的角度值取整
                sol_string_ang[i][j] = std::to_string(sol_int_ang[i][j]);   //to_string是将数字变成字符串
                //std::cout << "@@@" << endl;
            }
            std::cout << "Start simulating" << std::endl << std::endl;
            
            //运动到第i个解的姿态
            arm.setJointValueTarget(sol_rad[i]);
            arm.move();
            sleep(2);
            //显示结束，回到准备位置
            arm.setNamedTarget("home");
            arm.move();
            sleep(1);
            sol_send.clear();              //sol_send是之前定义的std::stringstream字符串流
            sol_send.str("");//清空缓冲区
            
            //生成关节角字符串，存入字符串流
            for(int j = 0; j < 6; ++j)
            {

                std::cout << j << " "<<sol_string_ang[i][j] << endl;
                sol_send << sol_string_ang[i][j];    //将角度值插入字符串流
                switch(j)
                {
                    case 0:sol_send << ('a');break;
                    case 1:sol_send << ('b');break;
                    case 2:sol_send << ('c');break;
                    case 3:sol_send << ('d');break;
                    case 4:sol_send << ('e');break;
                    case 5:sol_send << ('f');break;
                }                                       //插入每个关节相应的数字

            }

            machinecode.data = sol_send.str();     //将关节角字符串赋值给std_msgs::String machinecode中的data

            while(ros::ok() && call)
            {
                ROS_INFO("%s", machinecode.data.c_str());
                speed_code_pub.publish(machinecode);
                //检查一否一致，或者把接收的信息返回（在callback中发回）
                ros::spinOnce();

                sleep(10);
            }
            call = true;
            sol_send.clear();
            sol_send.str("");//清空缓冲区
            
            //回到home初始位置
            for(int j = 0; j < 6; ++j)
            {

                std::cout << j << " "<<'0' << endl;
                sol_send << '0';
                switch(j)
                {
                    case 0:sol_send << ('a');break;
                    case 1:sol_send << ('b');break;
                    case 2:sol_send << ('c');break;
                    case 3:sol_send << ('d');break;
                    case 4:sol_send << ('e');break;
                    case 5:sol_send << ('f');break;
                }

            }

            machinecode.data = sol_send.str();

            while(ros::ok() && call)
            {
                ROS_INFO("%s", machinecode.data.c_str());
                speed_code_pub.publish(machinecode);
                //检查一否一致，或者把接收的信息返回（在callback中发回）
                ros::spinOnce();

                sleep(10);
            }
            call = true;

        }
    }
    else
    {
        std::cout << "IK failed." << endl;
    }

    ros::shutdown(); 

    return 0;
}
