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

#ifndef PROBOT_VISION_MANAGER
#define PROBOT_VISION_MANAGER

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"

class VisionManager
{
  public:
	/**
   * @brief      VisionManager Constructor构造函数
   * 构造函数的两个输入为：
   * @param[in]  length   The length of the table桌子的长
   * @param[in]  breadth  The breadth of the table桌子的宽
   */
	VisionManager(float length, float breadth);


	/**
	 * @brief      Gets the 2d location of object in camera frame
	 *             从2维图像中找到目标物体的位置
	 * @param[in]  img   The image
	 * @param      x     x postion of the object
	 * @param      y     y position of the object
	 */
	void get2DLocation(cv::Mat img, float &x, float &y);


  private:
	/**
 	 * @brief      detect2DObject processes the image to isolate object
 	 *             检测物体的位置，pixel是像素
 	 * @param      pixel_x  position of the object in x-pixels
 	 * @param      pixel_y  position of the object in y-pixels
 	 */
	void detect2DObject(float &pixel_x, float &pixel_y, cv::Rect &tablePos);


	/**
	 * @brief      convertToMM converts pixel measurement to metric
	 *	       将像素位置转换为米
	 * @param      pixel_mm_x  The pixel millimeters per x
	 * @param      pixel_mm_y  The pixel millimeters per y
	 */
	void convertToMM(float &pixel_mm_x, float &pixel_mm_y);


	//@brief      detectTable isolates the table to get pixel to metric conversion
	//  在图像中找到矩形桌面并计算每毫米的像素数pixel_per_mm
	void detectTable(cv::Rect &tablePos);


	//@brief pixels per mm in x for the camera
	//  在相机x方向上每毫米多少像素
	float pixels_permm_x;


	//@brief pixels per mm in y for the camera
	//  在相机y方向上每毫米多少像素
	float pixels_permm_y;


	//@brief curr_pixel_centre_x is the object location in x
	//
	float curr_pixel_centre_x;


	//@brief curr_pixel_centre_y is the object location in y
	//
	float curr_pixel_centre_y;


	//@brief table length in meters桌子的长
	float table_length;

	//@brief table breadth in meters桌子的宽
	float table_breadth;

	//@brief centre of the image in pixels x
	//图像x方向的中心点
	float img_centre_x_;

	//@brief centre of the image in pixels y
	//图像y方向的中心点
	float img_centre_y_;

	//@brief curr_img is the image currently being processed
	// 被处理的图像数据
	cv::Mat curr_img;
};

#endif
