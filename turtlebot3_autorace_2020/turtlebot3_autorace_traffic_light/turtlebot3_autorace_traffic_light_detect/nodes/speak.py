#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, cv2, math, time
import numpy as np
import cv2.aruco as aruco
import subprocess
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from dynamic_reconfigure.server import Server
from turtlebot3_autorace_traffic_light_detect.cfg import DetectLaneParamsConfig
import pyttsx3
import time

engine=pyttsx3.init()
engine.setProperty('rate',100)

last_ID=-1
t0=0

class Speaker:

    def __init__(self):
        self.cvBridge = CvBridge()
        
        # you can choose image type "compressed", "raw"
        # 不需要哪个就把哪个注释掉
        self.sub_image_type = "raw" 
        #self.pub_image_type = "compressed"  

        #  订阅图像信息
        if self.sub_image_type == "compressed":
            # subscribes compressed image
            self.sub_image_original = rospy.Subscriber('/camera/image/compressed', CompressedImage, self.image_callback, queue_size = 1)
            #self.sub_image_original = rospy.Subscriber('/detect/image_input/compressed', CompressedImage, self.image_callback)
        elif self.sub_image_type == "raw":
            # subscribes raw image
            self.sub_image_original = rospy.Subscriber('/camera/image', Image, self.image_callback, queue_size = 1)
            #self.sub_image_original = rospy.Subscriber('/detect/image_input', Image, self.image_callback)


    def image_callback(self, image_msg):
        global last_ID,t0
        # 获取图像信息
        if self.sub_image_type == "compressed":
            #converting compressed image to opencv image
            np_arr = np.fromstring(image_msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif self.sub_image_type == "raw":
            image = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")

        # Aruco相关参数设置
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_50)
        param = aruco.DetectorParameters_create()
        # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        matrix = np.array([[322.0704122808738, 0., 199.2680620421962],
        [0., 320.8673986158544, 155.2533082600705],
        [0., 0., 1.]])
        dist = np.array([[0.1639958233797625, -0.271840030972792, 0.001055841660100477, -0.00166555973740089, 0.]]) 
            
        # 判断是否识别到aruco
        corners, markerID, rejected = aruco.detectMarkers(image, aruco_dict, parameters=param)
        if len(corners)>0:
            if time.time()-t0<15:
                pass
            else:
                rvec, tvec = aruco.estimatePoseSingleMarkers(corners, 0.05, matrix, dist)
                (rvec-tvec).any()
                for i in range(rvec.shape[0]):
                    aruco.drawDetectedMarkers(image, corners,markerID)
                    print("[INFO] Detected ArUco marker ID: {}".format(markerID))
                    engine.say('ID is'+str(markerID))
                    engine.runAndWait()
                    last_ID=markerID[0][0]
                    t0=time.time()
                    break   
        # 窗口绘图
        cv2.imshow("window", image)
        cv2.waitKey(10)


if __name__ == '__main__':
    rospy.init_node('Speaker')
    node = Speaker()
    rospy.spin()