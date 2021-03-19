#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import time 
import rospy
import roslib
import copy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray


def get_specific_color(img, hsv_th):
    """
    得到特定颜色的图像区域，输入参数：hsv_th 为hsv阈值，例如 [[96,100,50], [125,255,255]]
    输出只包含特定颜色的3通道图像.
    """
    # Convert BGR to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Threshold the HSV image to get only BGR colors
    if hsv_th[0][0] < hsv_th[1][0]:  # 正常阈值
        mask = cv2.inRange(hsv, np.array(hsv_th[0]), np.array(hsv_th[1]))
    else:  # 阈值范围需要修改为两段
        th1, th2 = copy.deepcopy(hsv_th), copy.deepcopy(hsv_th)  # 要使用深度拷贝，否则可能出错
        th1[0][0], th2[1][0] = 0, 180  # 修改阈值
        mask = cv2.inRange(hsv, np.array(th1[0]), np.array(th1[1])) | \
                cv2.inRange(hsv, np.array(th2[0]), np.array(th2[1]))
    # Bitwise-AND mask and original image
    return cv2.bitwise_and(img, img, mask=mask)

def get_max_contour(contours):
    contour = contours[0]
    if len(contours) > 1:  # 如果轮廓数目大于1， 计算最大轮廓
        max_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            # print(area)
            if area > max_area:
                max_area = area
                contour = cnt
    return contour

def nothing():
        pass  # 空函数


def image_thresh(gray):
    """
    将图像二值化，并适当去噪
    """
    blur = cv2.blur(gray,(3,3)) # 中值滤波
    ret, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    # cv2.imshow('thresh_{}'.format(self.res_id), thresh)     
    kernel = np.ones((3,3),np.uint8)  # 开运算会一定程度上减弱边缘，但是裁剪后的轮廓最好是加一下这个，可以滤出一下不需要的信息
    opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel, iterations = 2)
    erosion = cv2.erode(opening,kernel,iterations=1)
    return erosion


class image_process:

    def __init__(self): 
        rospy.init_node('hand_image_process', anonymous=True)
        self.test_flag = 0
        self.bridge = CvBridge()  # ros和opencv的桥梁
        self.img_src = np.zeros((800, 800, 3), dtype=np.uint8)  # 初始图像
        self.cnt_img = self.img_src.copy() # 结果图像
        # 订阅 相机 话题，原始图像，注册回调函数 image_sub_callback
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_sub_callback)
        self.center_pub = rospy.Publisher('/local_camera/region',Float32MultiArray)

    def image_sub_callback(self, data):
        ''' callback of image_sub '''
        # start = time.time() # 开始时间
        # 更新图像
        # 通过cv_bridge 得到opencv格式的image
        self.img_src = self.bridge.imgmsg_to_cv2(data, "bgr8")

        ############################## TEST
        # self.test_specific_color(self.img_src)
        ############################## TEST
        # 定义类
        green = get_specific_color(self.img_src, hsv_th=[[25,100,50], [45,255,255]])
        gray = cv2.cvtColor(green, cv2.COLOR_BGR2GRAY)
        gray = gray.astype(np.uint8)
        # 滤波
        thresh = image_thresh(gray)
        # 提取轮廓
        _, contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if len(contours) == 0:
            return
        # 取最大的轮廓
        cnt = get_max_contour(contours)
        # self.res_img = cv2.drawContours(green, contours, 0, (125, 255, 255), 2) 
        # (x,y),(w,h),ang = cv2.minAreaRect(contours[0])  # 得到外接矩形  xy为左上角，wh 宽和高
        rect = cv2.minAreaRect(cnt) # 得到最小外接矩形的（中心(x,y), (宽,高), 旋转角度）
        box = cv2.boxPoints(rect) # 获取最小外接矩形的4个顶点坐标(ps: cv2.boxPoints(rect) for OpenCV 3.x)
        # print(box)
        # x,y = rect[0]
        # w,h = rect[1]
        # ang = rect[2]
        # print(x,y,w,h,ang)

        # 定义发送数据
        region = Float32MultiArray()
        # for i in range(4):  
        #     for j in range(2):
        #         region.data.append(box[i][j])
        # region.data.append(rect[2])

        angle = -rect[2]
        angle = angle if angle < 45 else (angle - 90)

        if box[0][0] > box[2][0]:# 3 2 1 0
            region.data.append(box[3][0])
            region.data.append(box[3][1])
            region.data.append(box[2][0])
            region.data.append(box[2][1])
            region.data.append(box[1][0])
            region.data.append(box[1][1])
            region.data.append(box[0][0])
            region.data.append(box[0][1])
        else:# 2 1 0 3
            region.data.append(box[2][0])
            region.data.append(box[2][1])
            region.data.append(box[1][0])
            region.data.append(box[1][1])
            region.data.append(box[0][0])
            region.data.append(box[0][1])
            region.data.append(box[3][0])
            region.data.append(box[3][1])

        region.data.append(angle)

        self.center_pub.publish(region)  # 发布消息
        # print(x,y, x+w/2, y+h/2)
        ######################################## 显示图像，占时间可以注释
        self.res_img = self.img_src.copy()  # 显示图像
        box = np.int0(box)
        # 画出来
        cv2.drawContours(self.res_img, [box], 0, (255, 0, 0), 1)
        cv2.imshow("image", self.img_src) # 显示原图像
        cv2.imshow("res_image", self.res_img) # 显示处理图像
        cv2.waitKey(3)
        ######################################## 显示图像，占时间可以注释
        # end = time.time()
        # print('total time: {:.5f}s'.format(end - start))  # 0.010


    def test_specific_color(self, img):
        """
        test specific color region in rgb image
        """
        # Convert BGR to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        if self.test_flag == 0:
            cv2.namedWindow('hsv_image')  # 调参用
            cv2.createTrackbar('HL','hsv_image',70,180,nothing)
            cv2.createTrackbar('HH','hsv_image',125,180,nothing)
            cv2.createTrackbar('SL','hsv_image',100,255,nothing)
            cv2.createTrackbar('SH','hsv_image',255,255,nothing)
            cv2.createTrackbar('VL','hsv_image',50,255,nothing)
            cv2.createTrackbar('VH','hsv_image',255,255,nothing)
            self.test_flag += 1
        
        hl = cv2.getTrackbarPos('HL','hsv_image')
        hh = cv2.getTrackbarPos('HH','hsv_image')
        sl = cv2.getTrackbarPos('SL','hsv_image')
        sh = cv2.getTrackbarPos('SH','hsv_image')
        vl = cv2.getTrackbarPos('VL','hsv_image')
        vh = cv2.getTrackbarPos('VH','hsv_image')

        # define threshold range of color in HSV
        hsv_th = [[hl,sl,vl], [hh,sh,vh]]

        # Threshold the HSV image to get only BGR colors
        if hsv_th[0][0] < hsv_th[1][0]:  # 正常阈值
            mask = cv2.inRange(hsv, np.array(hsv_th[0]), np.array(hsv_th[1]))
        else:  # 阈值范围需要修改为两段
            th1, th2 = copy.deepcopy(hsv_th), copy.deepcopy(hsv_th)  # 要使用深度拷贝，否则可能出错
            th1[0][0], th2[1][0] = 0, 180  # 修改阈值
            mask = cv2.inRange(hsv, np.array(th1[0]), np.array(th1[1])) | \
                   cv2.inRange(hsv, np.array(th2[0]), np.array(th2[1]))

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(img, img, mask=mask)
        cv2.imshow('hsv_image', res)
        cv2.waitKey(3)
           

if __name__ == '__main__':
    img_proc = image_process()
    
    try:
        rospy.spin()  # 循环等待回调函数
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()



