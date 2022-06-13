#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, cv2, math, time
import numpy as np
import cv2.aruco as aruco
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from dynamic_reconfigure.server import Server
from turtlebot3_autorace_traffic_light_detect.cfg import DetectLaneParamsConfig

bottom_x = 156
top_x = 73
bottom_y = 95
top_y = 27

# selecting 4 points from the original image
pts_src = np.array([[160 - top_x, 180 - top_y], [160 + top_x, 180 - top_y], [160 + bottom_x, 120 + bottom_y], [160 - bottom_x, 120 + bottom_y]])

# selecting 4 points from image that will be transformed
pts_dst = np.array([[64, 0], [256, 0], [256, 240], [64, 240]])

#Calculate the homography matrix
H, status = cv2.findHomography(pts_src, pts_dst)

t0 = time.time() # unit: s

class Follower:

    def __init__(self):
        self.cvBridge = CvBridge()
        self.counter = 1
        self.isInitialized = False
        self.begin_detect_slot = False
        self.begin_aruco = False
        
        # you can choose image type "compressed", "raw"
        # 不需要哪个就把哪个注释掉
        self.sub_image_type = "raw" 
        #self.pub_image_type = "compressed"  

        #  订阅图像信息
        if self.sub_image_type == "compressed":
            # subscribes compressed image
            self.sub_image_original = rospy.Subscriber('/camera/image/compressed', CompressedImage, self.image_callback)
            #self.sub_image_original = rospy.Subscriber('/detect/image_input/compressed', CompressedImage, self.image_callback)
        elif self.sub_image_type == "raw":
            # subscribes raw image
            self.sub_image_original = rospy.Subscriber('/camera/image', Image, self.image_callback)
            #self.sub_image_original = rospy.Subscriber('/detect/image_input', Image, self.image_callback)
        
        self.lastError = 0

        # 发布速度信息
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.twist = Twist()
        self.stopped=False

        rospy.on_shutdown(self.fnShutDown)


    def image_callback(self, image_msg):
        if not self.isInitialized:
            self.initial_pose()

        # 获取图像信息
        if self.sub_image_type == "compressed":
            #converting compressed image to opencv image
            np_arr = np.fromstring(image_msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif self.sub_image_type == "raw":
            image = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")

        # 从原相机获取图像
        #image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    
        # 将图像转换为HSV格式
        image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h, w, d = image.shape
        # print(h,w)

        # 黑色的HSV表示
        lower_black = np.array([0,0,0])
        upper_black = np.array([179,255,90])

        #  Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(image_hsv, lower_black, upper_black)

        # Bird-eye view
        BEV_image = cv2.warpPerspective(image, H, (w,h))
        BEV_mask = cv2.warpPerspective(mask, H, (w,h)) # total mask
        BEV_mask1 = BEV_mask.copy() # left mask
        BEV_mask2 = BEV_mask.copy() # right mask

        # 对于左边的线，将右半边设为黑色
        BEV_mask1[0:h, w/2:w] = 0
        # 对于右边的线，将左半边设为黑色
        BEV_mask2[0:h, 0:w/2] = 0

        # 右半边矩形检测窗
        right_p1_x=170
        right_p1_y=180
        right_p2_x=220
        right_p2_y=240
        # 左半边矩形检测窗
        left_p1_x=80
        left_p1_y=230
        left_p2_x=130
        left_p2_y=240

        # 右半边缺口检测
        slot_p1_x =  170
        slot_p1_y = 220
        slot_p2_x =  300
        slot_p2_y = 240

        # in opencv
        # x is horizental direction
        # y is vertial direction

        # in numpy array
        # x is vertial direction
        # y is horizental direction
        BEV_probe_r=BEV_mask[right_p1_y:right_p2_y,right_p1_x:right_p2_x]
        BEV_probe_l=BEV_mask[left_p1_y:left_p2_y,left_p1_x:left_p2_x]
        BEV_slot = BEV_mask[slot_p1_y:slot_p2_y, slot_p1_x:slot_p2_x]

        M_total = cv2.moments(BEV_mask)
        M_probe_r=cv2.moments(BEV_probe_r)
        M_probe_l=cv2.moments(BEV_probe_l)
        M_slot = cv2.moments(BEV_slot)
        M1 = cv2.moments(BEV_mask1)
        M2 = cv2.moments(BEV_mask2)

        cx1 = int(M_total['m10']/M_total['m00'])
        cy1 = int(M_total['m01']/M_total['m00'])

        # Aruco相关参数设置
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_50)
        param = aruco.DetectorParameters_create()
        # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        matrix = np.array([[322.0704122808738, 0., 199.2680620421962],
        [0., 320.8673986158544, 155.2533082600705],
        [0., 0., 1.]])
        dist = np.array([[0.1639958233797625, -0.271840030972792, 0.001055841660100477, -0.00166555973740089, 0.]]) 
        
        # 当前时间
        t1 = time.time() 

        # 根据小车运行时间判断是否开始检测右线缺口
        if (t1-t0) > 70 and (t1-t0) < 80:
            self.begin_detect_slot = True
        else:
            self.begin_detect_slot = False

        if not self.begin_detect_slot: 
            # 未到时间， 无需检测
            self.begin_aruco = False
        else:
            # 检测缺口
            if M_slot['m00'] < 1000:
                self.begin_aruco = True
            else:
                self.begin_aruco = False
        
        if not self.begin_aruco: 
            # 正常寻线，还无需转弯
            if M_probe_r['m00']>1000:
                self.twist.linear.x = 0.2
                self.twist.angular.z = min(0.2+M_probe_r['m00']/150000,2)
                # print(M_probe_r['m00'])
                # print('turn left!')
            elif M_probe_l['m00']>1000:
                self.twist.linear.x = 0.2
                self.twist.angular.z = -1.2
                # print('turn right!')
            else:
                # print('forward')
                self.twist.linear.x = 0.20
                self.twist.angular.z = 0
            
            # 判断是否识别到aruco
            corners, markerID, rejected = aruco.detectMarkers(image, aruco_dict, parameters=param)
            if len(corners)>0 and not self.stopped: 
                print('detected')
                rvec, tvec = aruco.estimatePoseSingleMarkers(corners, 0.05, matrix, dist)
                (rvec-tvec).any()
                for i in range(rvec.shape[0]):
                    aruco.drawDetectedMarkers(image, corners,markerID)
                    #corners=markerCorner.reshape((4,2))
                    #(topLeft,topRight,bottomRight,bottomleft)=corners
                    #topRight = (int(topRight[0]), int(topRight[1]))
                    #bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                    #bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                    #topLeft = (int(topLeft[0]), int(topLeft[1]))
                    #cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
                    #cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
                    #cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
                    #cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
                    distance = tvec[0][0][2]
                    print("[INFO] ArUco marker ID: {}".format(markerID))
                    print("distance: ", distance, "m")
                    if distance<=0.10:
                        self.twist.linear.x=0
                        self.twist.angular.z=0
                        self.cmd_vel_pub.publish(self.twist)
                        self.stopped=True
                        time.sleep(10)
        else:
            # 转弯
            self.turn_right()
            # # 先保持直线行驶
            # if M_probe_r['m00']>1000:
            #     self.twist.linear.x = 0.2
            #     self.twist.angular.z = min(0.2+M_probe_r['m00']/150000,2)
            #     print(M_probe_r['m00'])
            #     print('turn left!')
            # elif M_probe_l['m00']>1000:
            #     self.twist.linear.x = 0.2
            #     self.twist.angular.z = -1.2
            #     print('turn right!')
            # else:
            #     print('forward')
            #     self.twist.linear.x = 0.20
            #     self.twist.angular.z = 0
            



        # sobel_x=np.array(
        #     [[-1, 1],
        #      [-1 ,1]]
        # )
        # sobel_y=np.array(
        #     [[-1,-1],
        #      [1 ,1]]
        # )
        # BEV_mask_x=cv2.filter2D(BEV_mask,-1,kernel=sobel_x)
        # BEV_mask_y=cv2.filter2D(BEV_mask,-1,kernel=sobel_y)

        # canny
        # BEV_canny=cv2.Canny(BEV_mask,100,150)


        # BEV_hough_line=cv2.HoughLines(BEV_canny,1,np.pi/180,100,10)
        # BEV_mask=cv2.Sobel(BEV_mask,cv2.CV_16S,0,1,3)
        # BEV_mask=cv2.convertScaleAbs(BEV_mask)

        # print(BEV_hough_line[0])
        # for rho, theta in BEV_hough_line[:,0,:]:
        #     a=np.cos(theta)
        #     b=np.sin(theta)
        #     x0=a*rho
        #     y0=b*rho
        #     x1=int(x0+250*(-b))
        #     y1=int(y0+250*(a))
        #     x2=int(x0-250*(-b))
        #     y2=int(y0-250*(a))
        #     cv2.line(BEV_image,(x1,y1),(x2,y2),(0,255,0),2)

        # Kp = 0.08
        # Kd = 0.007
        # print('probe')
        # print(M_probe['m00'])
        # print('total')
        # print(M_total['m00'])

        # else:
        #     if M_total['m00'] > 0:
        #         # if M2['m00'] > 0 and M1['m00'] <= 0:
        #         #     self.twist.linear.x = 0.17
        #         #     self.twist.angular.z = 0
        #         #     print("No Left side. ")
        #         #     self.cmd_vel_pub.publish(self.twist)
        #         # else:

        #         err = w/2 - cx1
        #         # print("Forward")
        #         self.twist.linear.x = 0.2

        #         ang = Kp * err + Kd * (err - self.lastError)
        #         self.lastError = err
        #         # ang = -(err*90.0/160)/10

        #         self.twist.angular.z = 0#-max(ang, -2.0) if ang < 0 else -min(ang, 2.0)
        #         print(ang)
        
        # 发布速度信息！
        self.cmd_vel_pub.publish(self.twist)
        
        # 窗口绘图
        cv2.circle(BEV_mask, (cx1, cy1), 10, (255,255,255), -1)
        cv2.circle(BEV_image, (cx1, cy1), 10, (0,0,255), -1)
        cv2.rectangle(BEV_image,(right_p1_x,right_p1_y),(right_p2_x,right_p2_y),[0,255,0],2)
        cv2.rectangle(BEV_image,(left_p1_x,left_p1_y),(left_p2_x,left_p2_y),[255,0,0],2)
        cv2.rectangle(BEV_image,(slot_p1_x,slot_p1_y),(slot_p2_x,slot_p2_y),[0,0,255],2)
        cv2.imshow("window", image)
        cv2.imshow("window2",BEV_image)
        cv2.imshow('mask', BEV_mask)
        cv2.imshow('BEV_probe_r',BEV_probe_r)
        cv2.imshow('BEV_probe_l',BEV_probe_l)
        # cv2.imshow('maskx', BEV_mask_x)
        # cv2.imshow('masky', BEV_mask_y)
        # cv2.imshow('maskcannt', BEV_canny)

        cv2.waitKey(10)


    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist) 

    def turn_right(self):
        global t0
        twist = Twist()

        # Go forward
        twist.linear.x = 0.2
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist) 
        # time.sleep(1.39)
        time.sleep(2)

        # Rotate rightward
        twist.linear.x = 0
        twist.angular.z = -1.0
        self.cmd_vel_pub.publish(twist) 
        time.sleep(1.57)

        # stop
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist) 
        t0 = 0

    def initial_pose(self):
        twist = Twist()
         # Go forward
        twist.linear.x = 0.0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist) 
        time.sleep(2)

        # Go forward
        twist.linear.x = 0.20
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist) 
        time.sleep(3.8)

       # Rotate rightforward
        twist.linear.x = 0
        twist.angular.z = -1.0
        self.cmd_vel_pub.publish(twist) 
        time.sleep(1.57)

        # stop
        twist.linear.x = 0
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist) 
        print('Initialzied')
        self.isInitialized = True


if __name__ == '__main__':
    rospy.init_node('LineFollower')
    node = Follower()
    rospy.spin()