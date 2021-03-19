//
// Created by bhan on 2020/5/28.
//

#include <ros/ros.h>
#include "visual_servo_control.h"
#include "visual_servo_control.cpp"


int main(int argc, char** argv)
{
    // 初始化ros节点
    ros::init(argc, argv, "visual_servo_test");
    // 创建节点句柄
    ros::NodeHandle nh;

    VisualServoControl vsc(nh);

    ros::spinOnce();
    while (ros::ok() && (!vsc.stopControl))
    {
        // Process image callback  检测到消息，则调用simGrasp的initiateGrasping()函数
        vsc.processingStep();
    }

    std::cout << "Successfully visual servo control" << std::endl;
    return 0;
}
