#!/usr/bin/env python
# -*- coding: utf-8 -*- 

# visual servo 

import cv2 as cv
import rospy as ros
import numpy as np
import glob
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray

tar_feature = np.array([[150.0],[375.0],[150.0],[300.0],[225.0],[300.0],[225.0],[375.0]])# 目标图像特征
# HSV threshold
hsv_th = {'blue'   : [[97,125,35], [125,255,255]], 
          'green'  : [[80,141,84], [96,255,255]],  
          'red'    : [[[0,160,68], [10,255,255]], [[160,160,68], [180,255,255]]]}


class image_proc:
    
    def __init__(self):
        ros.init_node("image_proc_node")
        ros.loginfo("starting image process node...")

        ros.on_shutdown(self.cleanup)
        self.bridge = CvBridge()
        self.image_sub = ros.Subscriber("/camera/color/image_raw", Image, self.img_sub_callBack, queue_size=1)
        self.points_pub = ros.Publisher("/usb_cam/points", Float32MultiArray)#发布四个点的像素坐标
        self.points_pub2handcamera = ros.Publisher("handcamera", Float32MultiArray)#发布四个点的像素坐标
        self.image_src = np.zeros((480, 640, 3), dtype=np.uint8)
        self.trackBar_flag = 0 #建立滑动条后置一     


    def img_sub_callBack(self, data):
        try:
            self.image_src = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
       
        hsv_mask = self.hsv_process(self.image_src, test_flag=True)               # HSV 阈值调整

        cnts = cv.findContours(hsv_mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[1]
        if len(cnts) > 0:
            cnt = cnts[0]
            if len(cnts) > 1:
                max_area = cv.contourArea(cnt)
                for i in range(1, len(cnts)):
                    area = cv.contourArea(cnts[i])
                    if area > max_area:
                        cnt = cnts[i]

            x,y,w,h = cv.boundingRect(cnt)
            rect = cv.minAreaRect(cnt)# 得到最小外接矩形的（中心(x,y), (宽,高), 旋转角度）
            points = cv.boxPoints(rect)# 获取最小外接矩形的4个顶点坐标
            # print('rect[0]:', rect[0])
            # print('rect[1]:', rect[1])

            process_result = Float32MultiArray()# 定义要发送的数据 四个顶点坐标

            for i in range(4):
                for j in range(2):
                    process_result.data.append(points[i][j])
            process_result.data.append(rect[0][0])# 最小矩形的中心坐标  索引8
            process_result.data.append(rect[0][1])
            process_result.data.append(rect[1][0])# 最小矩形的宽和高  索引10
            process_result.data.append(rect[1][1])
            process_result.data.append(rect[2])# 最小矩形的旋转角度  索引12

            handcamera_msg = Float32MultiArray()    # 韩奔师兄所需数据
            for i in range(4):
                for j in range(2):
                    handcamera_msg.data.append(points[i][j])
            handcamera_msg.data.append(rect[2])# 最小矩形的旋转角度  索引12
            print('handcamera: {}'.format(handcamera_msg))
            self.points_pub2handcamera.publish(handcamera_msg)

            cv.rectangle(self.image_src, (x,y), (x+w,y+h), (50,255,50), 2)
            # cv.drawContours(self.image_src, [cnt], 0, (255,0,0), 2)#绘制轮廓

            for i in range(4):
                str_coord = '(' + str(int(points[i][0])) + ',' + str(int(points[i][1])) + ')'
                cv.circle(self.image_src, (points[i][0],points[i][1]), 5, (0,0,200), -1)
                cv.putText(self.image_src,str_coord,(10,30*(i+1)),cv.FONT_HERSHEY_SIMPLEX,0.7,(255,255,0),2,cv.LINE_AA)
            self.draw_tar_feature()

            self.points_pub.publish(process_result)

        cv.imshow("detect_window", self.image_src)
        cv.waitKey(5)

    def hsv_process(self, image, test_flag=False):
        # global lower, upper
        hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        hsv_image = cv.GaussianBlur(hsv_image, (21,21), 0)

        th_blue = np.array(hsv_th['blue'])
        th_green = np.array(hsv_th['green'])
        th_red1 = np.array(hsv_th['red'][0])
        th_red2 = np.array(hsv_th['red'][1])

        hsv_mask_blue = cv.inRange(hsv_image, th_blue[0], th_blue[1])
        hsv_mask_green = cv.inRange(hsv_image, th_green[0], th_green[1])
        hsv_mask_red1 = cv.inRange(hsv_image, th_red1[0], th_red1[1])
        hsv_mask_red2 = cv.inRange(hsv_image, th_red2[0], th_red2[1])
        hsv_mask_red = hsv_mask_red1 | hsv_mask_red2
        hsv_mask = hsv_mask_blue | hsv_mask_green | hsv_mask_red

        blue_part = cv.bitwise_and(image, image, mask=hsv_mask_blue)
        green_part = cv.bitwise_and(image, image, mask=hsv_mask_green)
        red_part = cv.bitwise_and(image, image, mask=hsv_mask_red)

        image_sprtn = np.stack((blue_part[:,:,0], green_part[:,:,1], red_part[:,:,2]), axis=2)
        cv.imshow("hsv_mask_window", image_sprtn)

        if test_flag:
            if self.trackBar_flag == 0:
                self.trackBar_flag = 1
                cv.namedWindow("hsv_test_window")
                cv.createTrackbar("H_min", "hsv_test_window", 0, 180, onTrackbar)
                cv.createTrackbar("H_max", "hsv_test_window", 180, 180, onTrackbar)
                cv.createTrackbar("S_min", "hsv_test_window", 0, 255, onTrackbar)
                cv.createTrackbar("S_max", "hsv_test_window", 255, 255, onTrackbar)
                cv.createTrackbar("V_min", "hsv_test_window", 0, 255, onTrackbar)
                cv.createTrackbar("V_max", "hsv_test_window", 255, 255, onTrackbar)


            Hmin = cv.getTrackbarPos("H_min", "hsv_test_window")
            Hmax = cv.getTrackbarPos("H_max", "hsv_test_window")
            Smin = cv.getTrackbarPos("S_min", "hsv_test_window")
            Smax = cv.getTrackbarPos("S_max", "hsv_test_window")
            Vmin = cv.getTrackbarPos("V_min", "hsv_test_window")
            Vmax = cv.getTrackbarPos("V_max", "hsv_test_window")

            lower = np.array([Hmin, Smin, Vmin]) #测试调整阈值用
            upper = np.array([Hmax, Smax, Vmax])

            hsv_mask_test = cv.inRange(hsv_image, lower, upper) 
            image_sprtn_test = cv.bitwise_and(image, image, mask=hsv_mask_test) #合并mask与原图像，实现分割
            cv.imshow("hsv_test_window", image_sprtn_test)
        
        cv.waitKey(3)
        return hsv_mask

    def draw_tar_feature(self):
        for i in range(0,8,2):
            cv.circle(self.image_src, (int(tar_feature[i][0]),int(tar_feature[i+1][0])), 5, (0,200,0), -1)

    def cleanup(self):
        print("Shutting down vision node..")
        cv.destroyAllWindows()

def onTrackbar(x):
    pass
    
if __name__ == '__main__':
    try:
        image_proc()
        ros.spin()
    except KeyboardInterrupt:
        print("shuting down detect node.")
        cv.destroyAllWindows()


    
