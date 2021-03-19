#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import time 
import rospy
import roslib
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray


def get_specific_color(img, color):
    """
    get specific color region in hsv image
    """
    # Convert BGR to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    if color == 'blue':
        # define range of blue color in HSV
        lower_blue = np.array([100,43,46])
        upper_blue = np.array([124,255,255])
        # Threshold the HSV image to get only blue colors
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        # Bitwise-AND mask and original image
        return cv2.bitwise_and(img, img, mask=mask_blue)
    elif color == 'green':
        # define range of green color in HSV
        lower_green = np.array([35,43,46])
        upper_green = np.array([77,255,255])
        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        return cv2.bitwise_and(img, img, mask=mask_green)
    elif color == 'red':
        # define range of red color in HSV
        lower_red1 = np.array([0,43,46])
        upper_red1 = np.array([10,255,255])
        lower_red2 = np.array([156,43,46])
        upper_red2 = np.array([180,255,255])
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = mask_red1 | mask_red2
        return cv2.bitwise_and(img, img, mask=mask_red)
    elif color == 'yellow':
        # define range of green color in HSV
        lower = np.array([26,43,46])
        upper = np.array([34,255,255])
        mask = cv2.inRange(hsv, lower, upper)
        return cv2.bitwise_and(img, img, mask=mask)
    elif color == 'orange':
        # define range of green color in HSV
        lower = np.array([11,43,46])
        upper = np.array([25,255,255])
        mask = cv2.inRange(hsv, lower, upper)
        return cv2.bitwise_and(img, img, mask=mask)
    elif color == 'cyan_blue':
        # define range of green color in HSV
        lower = np.array([78,43,46])
        upper = np.array([99,255,255])
        mask = cv2.inRange(hsv, lower, upper)
        return cv2.bitwise_and(img, img, mask=mask)
    elif color == 'purple':
        # define range of green color in HSV
        lower = np.array([125,43,46])
        upper = np.array([155,255,255])
        mask = cv2.inRange(hsv, lower, upper)
        return cv2.bitwise_and(img, img, mask=mask)
    elif color == 'black':  # 背景是黑色，所以返回的图像是全黑的
        # define range of green color in HSV
        lower = np.array([0,0,0])
        upper = np.array([180,255,46])
        mask = cv2.inRange(hsv, lower, upper)
        return cv2.bitwise_and(img, img, mask=mask)
    elif color == 'gray':
        # define range of green color in HSV
        lower = np.array([0,0,46])
        upper = np.array([180,43,220])
        mask = cv2.inRange(hsv, lower, upper)
        return cv2.bitwise_and(img, img, mask=mask)
    elif color == 'white':
        # define range of green color in HSV
        lower = np.array([0,0,221])
        upper = np.array([180,30,255])
        mask = cv2.inRange(hsv, lower, upper)
        return cv2.bitwise_and(img, img, mask=mask)



class image_process:

    def __init__(self): 
        rospy.init_node('hand_image_process', anonymous=True)
        self.bridge = CvBridge()  # ros和opencv的桥梁
        self.img_src = np.zeros((800, 800, 3), dtype=np.uint8)  # 初始图像
        self.cnt_img = self.img_src.copy() # 结果图像
        # 订阅 相机 话题，原始图像，注册回调函数 image_sub_callback
        self.image_sub = rospy.Subscriber("/local_camera/image_raw", Image, self.image_sub_callback)
        self.center_pub = rospy.Publisher('/local_camera/region',Float32MultiArray)

    def image_sub_callback(self, data):
        ''' callback of image_sub '''
        # start = time.time() # 开始时间
        # 更新图像
        # 通过cv_bridge 得到opencv格式的image
        self.img_src = self.bridge.imgmsg_to_cv2(data, "bgr8")
        green = get_specific_color(self.img_src, 'green') # 检测的颜色 可以改
        gray = cv2.cvtColor(green, cv2.COLOR_BGR2GRAY)
        gray = gray.astype(np.uint8)
        # 提取轮廓
        _, contours, _ = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if len(contours) == 0:
            return
        # self.res_img = cv2.drawContours(green, contours, 0, (125, 255, 255), 2) 
        # (x,y),(w,h),ang = cv2.minAreaRect(contours[0])  # 得到外接矩形  xy为左上角，wh 宽和高
        rect = cv2.minAreaRect(contours[0]) # 得到最小外接矩形的（中心(x,y), (宽,高), 旋转角度）
        box = cv2.boxPoints(rect) # 获取最小外接矩形的4个顶点坐标(ps: cv2.boxPoints(rect) for OpenCV 3.x)
        # print(box)
        # x,y = rect[0]
        # w,h = rect[1]
        # ang = rect[2]
        # print(x,y,w,h,ang)

        # 定义发送数据
        region = Float32MultiArray()
        for i in range(4):  
            for j in range(2):
                region.data.append(box[i][j])
        # # 左上角
        # region.data.append(box[0][0])
        # region.data.append(box[0][1])
        # # 右上角
        # region.data.append(box[0][0])
        # region.data.append(y)
        # # 右下角
        # region.data.append(x+w)
        # region.data.append(y+h)
        # # 左下角
        # region.data.append(x)
        # region.data.append(y+h)

        region.data.append(rect[2])

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

           

if __name__ == '__main__':
    img_proc = image_process()
    
    try:
        rospy.spin()  # 循环等待回调函数
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()



