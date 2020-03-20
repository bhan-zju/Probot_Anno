#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h> 
#include <Eigen/Eigen>



//接收到订阅的消息后，会进入消息回调函数
void positionCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	//将接收到的消息打印出来
    ROS_INFO("got position");
    std_msgs::Float32MultiArray position_msg = *msg;
    std::cout<<position_msg.data.size()<<std::endl;
	std::vector<Eigen::Vector3f> center;//创建存储色块质心的对象

    for(int i=0;i<position_msg.data.size();i=i+3){
        Eigen::Vector3f temp_vector;
        temp_vector(0)=position_msg.data[i];
        temp_vector(1)=position_msg.data[i+1];
        temp_vector(2)=position_msg.data[i+2];
        std::cout<<"color&position "<<temp_vector<<std::endl;
        center.push_back(temp_vector);
    }

}


int main(int argc, char * argv[]){

	// 初始化ROS节点
	ros::init(argc,argv,"color_position_subscriber");
	//创建节点句柄
	ros::NodeHandle n;
	// 创建一个Subscriber，
	ros::Subscriber position_sub = n.subscribe("/color_position",1,positionCallback);
	//循环等待回调函数
	ros::spin();

	return 0;

}