/**
 * kinect.h  V 0.1 
 * A driver for Kinect V2
 * If you want using kinect.h to into your project, please 
 * move kinect.h to "{workspaceFolder}/include" 
 * add set(INCLUDE "${CMAKE_SOURCE_DIR}/include") in CMakeLists.txt
 * add include_directories(${INCLUDE}) in CMakeLists.txt
 * 
 * @Author zyli
 * @Time   2019.11.11
 * */

#pragma once

// C++ headers
#include <iostream>
#include <string>
#include <signal.h> 
#include <cstdlib>
#include <chrono> 
// opencv headers 
#include <opencv2/opencv.hpp>
// kinect driver libfreenect2 headers 
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/logger.h>
// PCL headers
/*#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>*/
#include <Eigen/Core>


// what type of processor
enum Processor{
	CPU, OPENCL, OPENGL, CUDA
};
bool stop = false; //stop 标志位

void sigint_handler(int s){
	stop = true;
}

class Kinect {

public:
    //Create and initiate
    Kinect(Processor freenectprocessor = CPU) : listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth), 
	                                       undistorted(512, 424, 4), registered(512, 424, 4), big_mat(1920, 1082, 4), qnan(std::numeric_limits<float>::quiet_NaN()){
        signal(SIGINT,sigint_handler);

        if(freenect2.enumerateDevices() == 0)
        {
            std::cout << "no kinect2 connected!" << std::endl;
            exit(-1);
        }

        switch (freenectprocessor)
        {
            case CPU:
                std::cout << "creating Cpu processor" << std::endl;
                dev = freenect2.openDefaultDevice (new libfreenect2::CpuPacketPipeline ());
                std::cout << "created" << std::endl;
                break;
            case OPENGL:
                std::cout << "creating OpenGL processor" << std::endl;
                dev = freenect2.openDefaultDevice (new libfreenect2::OpenGLPacketPipeline ());
                break;
            default:
                std::cout << "creating Cpu processor" << std::endl;
                dev = freenect2.openDefaultDevice (new libfreenect2::CpuPacketPipeline ());
                break;
        }

        dev->setColorFrameListener(&listener);
        dev->setIrAndDepthFrameListener(&listener);
        dev->start();

        logger = libfreenect2::getGlobalLogger();
        registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());

        prepareMake3D(dev->getIrCameraParams());
    }

    //Stop and close 
    void shutDown(){
		dev->stop();
  		dev->close();
	}
    
	// Only color
	void getColor(cv::Mat & color_mat){
		listener.waitForNewFrame(frames);
		libfreenect2::Frame * rgb = frames[libfreenect2::Frame::Color];

		cv::Mat tmp_color(rgb->height, rgb->width, CV_8UC4, rgb->data);
		cv::flip(tmp_color, color_mat, 1);

		listener.release(frames);
	}

    // Only depth
	void getDepth(cv::Mat depth_mat){
		listener.waitForNewFrame(frames);
		libfreenect2::Frame * depth = frames[libfreenect2::Frame::Depth];
		
		cv::Mat depth_tmp(depth->height, depth->width, CV_32FC1, depth->data);
        cv::flip(depth_tmp, depth_mat, 1);

		listener.release(frames);
	}

	// Depth and color 
	void get(cv::Mat & color_mat, cv::Mat & depth_mat){
		listener.waitForNewFrame(frames);
		libfreenect2::Frame * rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame * depth = frames[libfreenect2::Frame::Depth];

		registration->apply(rgb, depth, &undistorted, &registered, false, &big_mat);

		cv::Mat tmp_depth(undistorted.height, undistorted.width, CV_32FC1, undistorted.data);
		cv::Mat tmp_color(rgb->height, rgb->width, CV_8UC4, rgb->data);

		cv::flip(tmp_depth, depth_mat, 1);
		cv::flip(tmp_color, color_mat, 1);

		listener.release(frames);
	}

    /*// Get point cloud
    void getCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud){
        listener.waitForNewFrame(frames);
		libfreenect2::Frame * rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame * depth = frames[libfreenect2::Frame::Depth];

		registration->apply(rgb, depth, &undistorted, &registered, true, &big_mat);
		const std::size_t w = undistorted.width;
		const std::size_t h = undistorted.height;

        cv::Mat tmp_itD0(undistorted.height, undistorted.width, CV_8UC4, undistorted.data);
        cv::Mat tmp_itRGB0(registered.height, registered.width, CV_8UC4, registered.data);

        cv::flip(tmp_itD0,tmp_itD0,-1);
        cv::flip(tmp_itRGB0,tmp_itRGB0,-1);

        const float * itD0 = (float *) tmp_itD0.ptr();
        const char * itRGB0 = (char *) tmp_itRGB0.ptr();
        
		pcl::PointXYZRGBA * itP = &cloud->points[0];
        bool is_dense = true;
		
		for(std::size_t y = 0; y < h; ++y){

			const unsigned int offset = y * w;
			const float * itD = itD0 + offset;
			const char * itRGB = itRGB0 + offset * 4;
			const float dy = rowmap(y);

			for(std::size_t x = 0; x < w; ++x, ++itP, ++itD, itRGB += 4 )
			{
				const float depth_value = *itD / 1000.0f;
				
				if(!std::isnan(depth_value) && !(std::abs(depth_value) < 0.0001)){
	
					const float rx = colmap(x) * depth_value;
                	const float ry = dy * depth_value;               
					itP->z = depth_value;
					itP->x = rx;
					itP->y = ry;

					itP->b = itRGB[0];
					itP->g = itRGB[1];
					itP->r = itRGB[2];
					itP->a = itRGB[3];
				} else {
					itP->z = qnan;
					itP->x = qnan;
					itP->y = qnan;

					itP->b = qnan;
					itP->g = qnan;
					itP->r = qnan;
					itP->a = qnan;
					is_dense = false;
 				}
			}
		}
		cloud->is_dense = is_dense;
		listener.release(frames);
    }*/

private:

    void prepareMake3D(const libfreenect2::Freenect2Device::IrCameraParams & depth_p)
	{
		const int w = 512;
		const int h = 424;
	    float * pm1 = colmap.data();
	    float * pm2 = rowmap.data();
	    for(int i = 0; i < w; i++)
	    {
	        *pm1++ = (i-depth_p.cx + 0.5) / depth_p.fx;
	    }
	    for (int i = 0; i < h; i++)
	    {
	        *pm2++ = (i-depth_p.cy + 0.5) / depth_p.fy;
	    }
	}

    libfreenect2::Freenect2 freenect2;
	libfreenect2::Freenect2Device * dev = 0;
	libfreenect2::PacketPipeline * pipeline = 0;
	libfreenect2::Registration * registration = 0;
	libfreenect2::SyncMultiFrameListener listener;
	libfreenect2::Logger * logger = nullptr;
	libfreenect2::FrameMap frames;
	libfreenect2::Frame undistorted,registered,big_mat;
	Eigen::Matrix<float,512,1> colmap;
	Eigen::Matrix<float,424,1> rowmap;
	std::string serial;
    float qnan;

};
