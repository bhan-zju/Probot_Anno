# -*- coding=utf-8 -*-

import cv2
import numpy as np


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


if __name__ == '__main__':

    img = cv2.imread('1.jpg')

    res_blue = get_specific_color(img, 'blue')
    res_green = get_specific_color(img, 'green')
    res_red = get_specific_color(img, 'red')
    yellow = get_specific_color(img, 'yellow')
    orange = get_specific_color(img, 'orange')
    cyan_blue = get_specific_color(img, 'cyan_blue')
    purple = get_specific_color(img, 'purple')
    black = get_specific_color(img, 'black')
    white = get_specific_color(img, 'white')
    gray = get_specific_color(img, 'gray')

    cv2.imshow('bgr',img)
    # cv2.imshow('hsv_h',hsv[:, :, 0])
    # cv2.imshow('hsv_s',hsv[:, :, 1])
    # cv2.imshow('hsv_v',hsv[:, :, 2])
    cv2.imshow('blue',res_blue)
    cv2.imshow('green',res_green)
    cv2.imshow('red',res_red)
    cv2.imshow('yellow',yellow)
    cv2.imshow('orange',orange)
    cv2.imshow('cyan_blue',cyan_blue)
    cv2.imshow('purple',purple)
    cv2.imshow('black',black)
    cv2.imshow('white',white)
    cv2.imshow('gray',gray)

    cv2.waitKey(0)

    cv2.destroyAllWindows()
