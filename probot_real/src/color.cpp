#include <ros/ros.h>
#include "kinect.h" // kinect头文件
#include <std_msgs/Float32MultiArray.h> //消息类型头文件
// opencv 头文件
#include <opencv2/core/core.hpp> 
#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/highgui/highgui.hpp>


//设置框选区域
cv::Point left_up = {130,40};//左上角的点
int roi_whight = 1600;//框的宽
int roi_high = 1000;//框的高
cv::Rect rect(left_up.x, left_up.y, roi_whight, roi_high);//框选区域，矩形

// 图像处理函数，
// 输入为Kinect采集的彩色图BGRA类型，还有blob检测器
// 输出 消息类型， 三个一组（type，x值，y值）
std_msgs::Float32MultiArray img_process(cv::Mat& color, cv::Ptr<cv::SimpleBlobDetector>& detector)
{
  std_msgs::Float32MultiArray position_msg; 
  cv::Mat rgb;
  cv::cvtColor(color, rgb, cv::COLOR_BGRA2BGR); //将BGRA类型转换为BGR类型

  cv::Mat col = rgb(rect).clone();//重新定义

  // cv::imshow("color", col); //显示框选的图
  // cv::waitKey(1);
  // image process start
  //转化为HSV颜色空间
  IplImage src = IplImage(col);
  IplImage* image = &src;
  IplImage* hsv = cvCreateImage(cvGetSize(image), 8, 3);
  cvCvtColor(image, hsv, CV_BGR2HSV);
  cv::Mat dst = cv::cvarrToMat(hsv); //得到了转换为HSV的图像
  // cv::imshow("3", dst);
  // cv::waitKey(1);
  // 定义两个全零的矩阵，用来存放二值化后的图
  cv::Mat img_red = cv::Mat::zeros(roi_high, roi_whight, CV_8UC1);
  cv::Mat img_yellow = cv::Mat::zeros(roi_high, roi_whight, CV_8UC1);
  //二值化
  for (int row = 0; row < dst.rows; row++)
  {
    for (int col = 0; col < dst.cols; col++)
    {
      /* 注意 Mat::at 函数是个模板函数, 需要指明参数类型, 因为这张图是具有HSV三通道的图,
        所以它的参数类型可以传递一个 Vec3b, 这是一个存放 3 个 uchar 数据的 Vec(向量). */
      if (dst.at<cv::Vec3b>(row, col)[0] > 25 && dst.at<cv::Vec3b>(row, col)[0] < 35)//黄色
      {
        img_yellow.at<uchar>(row, col) = 255;
      }
      else if (((dst.at<cv::Vec3b>(row, col)[0] > 0 && dst.at<cv::Vec3b>(row, col)[0] < 10)) || ((dst.at<cv::Vec3b>(row, col)[0] > 155 && dst.at<cv::Vec3b>(row, col)[0] < 180)))//红色
      {
        img_red.at<uchar>(row, col) = 255;
      }
     
    }
  }
  // cv::imshow("red", img_red); //显示二值化后的图
  // cv::imshow("yellow", img_yellow);
  // cv::waitKey(1);

  std::vector<cv::KeyPoint> keypoints; //定义关键点 就是检测后的色块中心坐标点
  cv::Mat img_with_keypoints; //定义待现实的结果图像
  detector->detect(img_red, keypoints);//blob检测
  std::cout << "red" << keypoints.size() << std::endl; //输出检测到的点数目
  
	std::vector<cv::KeyPoint>::iterator it = keypoints.begin(); //从头遍历检测到的关键点
	while (it != keypoints.end())
	{
		it->pt.x = it->pt.x + left_up.x;//+left_up.x是映射到原图像的坐标
		it->pt.y = it->pt.y + left_up.y;//+left_up.y是映射到原图像的坐标
    position_msg.data.push_back(1); //颜色类型 红色为1
    position_msg.data.push_back(it->pt.x); //色块中心像素坐标
    position_msg.data.push_back(it->pt.y);
		it++;	
	}
  drawKeypoints(rgb, keypoints, img_with_keypoints, cv::Scalar(255, 0, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS); //在rgb原图上画上色块位置

  detector->detect(img_yellow, keypoints);//blob检测
  std::cout << "yellow" << keypoints.size() << std::endl;//输出检测到的点数目
  
  it = keypoints.begin();//从头遍历检测到的关键点
	while (it != keypoints.end())
	{
    it->pt.x = it->pt.x + left_up.x;//+left_up.x是映射到原图像的坐标
		it->pt.y = it->pt.y + left_up.y;//+left_up.y是映射到原图像的坐标
    position_msg.data.push_back(2); //颜色类型 黄色为2
    position_msg.data.push_back(it->pt.x); //色块中心像素坐标
    position_msg.data.push_back(it->pt.y);
		it++;	
	}
  drawKeypoints(img_with_keypoints, keypoints, img_with_keypoints, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);//在上一张图上画上这次检测的色块位置
  
  cv::imshow("result", img_with_keypoints); //显示结果图像
  cv::waitKey(1);

  return position_msg; //返回消息
}


int main(int argc, char * argv[]){
  
  Kinect kinect(OPENGL); //create and initiate kinect
  cv::Mat color(1080, 1920, CV_8UC4);//创建变量  
  std_msgs::Float32MultiArray position_msg; //创建消息
  //初始化blob检测器的参数
  cv::SimpleBlobDetector::Params params;
  //阈值控制
  params.minThreshold = 0;
  params.maxThreshold = 255;
  params.thresholdStep = 10;
  //颜色控制
  params.filterByColor = false;
  params.blobColor = 255;
  //像素面积大小控制
  params.filterByArea = true;
  params.minArea = 3000;
  params.maxArea = 100000;
  //形状（圆）
  params.filterByCircularity = false;
  params.minCircularity = 0.9;
  //形状（凸）
  params.filterByConvexity = true;
  params.minConvexity = 0.8;
  //斑点惯性率
  params.filterByInertia = false;
  params.minInertiaRatio = 0.5;
  cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params); //创建blob检测器

	//ROS node init
	ros::init(argc, argv, "color_position_publisher");
	//创建节点句柄
	ros::NodeHandle n;
	//创建一个publisher，
	ros::Publisher position_pub = n.advertise<std_msgs::Float32MultiArray>("/color_position",1);
  
	//设置循环刷新的频率
	ros::Rate loop_rate(20);
	

  while(ros::ok()){

    kinect.getColor(color); //调用kinect得到图像

    // image process end
    position_msg = img_process(color,detector);

    // send 
    position_pub.publish(position_msg);

		//按照循环频率延时
		loop_rate.sleep();
  }

  kinect.shutDown();
	
  return 0;
}

