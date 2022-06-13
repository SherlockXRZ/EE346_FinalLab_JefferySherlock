#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
import cv2.aruco as aruco
import time
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:

        def __init__(self):

                self.bridge = cv_bridge.CvBridge()

                self.image_sub = rospy.Subscriber('camera/image',
                        Image, self.image_callback)

                self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=10)

                self.twist = Twist()
                
                self.Nfound = True

                print("[INFO] Node \"Follower\" Initializing ...")


        def image_callback(self, msg):
                aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
                parameters = aruco.DetectorParameters_create()
                
                font = cv2.FONT_HERSHEY_SIMPLEX  # font for displaying text (below)
                
                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                
                # move
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                h, w, d = image.shape

                lower_yellow = numpy.array([ 26, 43, 46])
                upper_yellow = numpy.array([ 34, 255, 255])

                lower_white = numpy.array([0, 0, 80])
                upper_white = numpy.array([180, 30, 255])
                
                homography = numpy.array([[-2.23471253e-01, -1.33270129e+00,  1.95755401e+02],
                                      [-1.94004843e-16, -2.16157649e+00,  2.78843367e+02],
                                      [-0.00000000e+00, -8.32938308e-03,  1.00000000e+00]])

                BEV_image = cv2.warpPerspective(image, homography, (w, h))
                BEV_hsv= cv2.cvtColor(BEV_image, cv2.COLOR_BGR2HSV)
                
                mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)
                mask2 = cv2.inRange(hsv, lower_white, upper_white)
                
                BEV_mask1 = cv2.inRange(BEV_hsv, lower_yellow, upper_yellow)
                BEV_mask2 = cv2.inRange(BEV_hsv, lower_white, upper_white)

                search_top = int(round(2*h/3))
                mask1[0:search_top, 0:w] = 0
                mask2[0:search_top, 0:w] = 0

                MC1 = cv2.moments(mask1)
                MC2 = cv2.moments(mask2)

                BEV_mask1[0:search_top, 0:w] = 0
                BEV_mask2[0:search_top, 0:w] = 0
                BEV_mask1[search_top:h, 0:int(round(w/3))] = 0
                BEV_mask1[search_top:h, int(round(2*w/3)):w] = 0
                BEV_mask2[search_top:h, 0:int(round(w/3))] = 0
                BEV_mask2[search_top:h, int(round(2*w/3)):w] = 0
                
                M1 = cv2.moments(BEV_mask1)
                M2 = cv2.moments(BEV_mask2)

                if MC1['m00'] > 0 and MC2['m00'] > 0:
                        ux1 = int(MC1['m10']/MC1['m00'])
                        uy1 = int(MC1['m01']/MC1['m00'])
                        ux2 = int(MC2['m10']/MC2['m00'])
                        uy2 = int(MC2['m01']/MC2['m00'])

                        fpt_ux = int((ux1 + ux2)/2)
                        fpt_uy = int((uy1 + uy2)/2 + 2*h/3)

                        cv2.circle(image, (ux1, uy1), 10, (0,255,255), -1)
                        cv2.circle(image, (ux2, uy2), 10, (255,255,255), -1)
                        cv2.circle(image, (fpt_ux, fpt_uy), 10, (128,128,128), -1)

                if M1['m00'] > 0 and M2['m00'] > 0:
                        cx1 = int(M1['m10']/M1['m00'])
                        cy1 = int(M1['m01']/M1['m00'])

                        cx2 = int(M2['m10']/M2['m00'])
                        cy2 = int(M2['m01']/M2['m00'])

                        fpt_x = int(round((cx1 + cx2)/2))
                        fpt_y = int(round((cy1 + cy2)/2 + 2*h/3))

                        cv2.circle(BEV_image, (cx1, cy1), 10, (0,255,255), -1)
                        cv2.circle(BEV_image, (cx2, cy2), 10, (255,255,255), -1)
                        cv2.circle(BEV_image, (fpt_x, fpt_y), 10, (128,128,128), -1)

                        err = w/2 - fpt_x

                        self.twist.linear.x = 0.33
                        self.twist.angular.z = (err*90.0/160)/10
                        self.cmd_vel_pub.publish(self.twist)
                
                # detect and stop
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                
                mtx = numpy.array([[265, 0., 160],
                                   [0., 265, 120],
                                   [0., 0., 1.]])
                dist = numpy.array([[0., 0., 0., 0., 0.]]) 
                
                corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
                
                if ids is not None and self.Nfound:
                    # If turtlebot3 has detected Aruco Marker
                    # estimate the pose of each marker, return rvet and tvec from camera coeficcients
                    rvec, tvec = aruco.estimatePoseSingleMarkers(corners, 0.015, mtx, dist)

                     # get rid of that nasty numpy value array error
                    (rvec - tvec).any() 

                    for i in range(rvec.shape[0]):
                        aruco.drawAxis(image, mtx, dist, rvec[i, :, :], tvec[i, :, :], 0.03)
                        aruco.drawDetectedMarkers(image, corners)
                        # Draw the ID of Aruco Marker
                        cv2.putText(image, "Id: " + str(ids), (0, 64), font, 0.75, (0, 255, 0), 2, cv2.LINE_AA)

                    distance = tvec[0][0][2]   # unit: meter
                    cv2.putText(image, "distance: " + str(distance), (0, 128), font, 0.75, (0, 255, 0), 2, cv2.LINE_AA)

                    if distance<0.15 :
                        print("[INFO] Aruco Marker Detected!  ID: " + str(ids))
                        print("[INFO] Turtlebot3 stops for 10 seconds")
                        self.twist.angular.z = 0.0
                        self.twist.linear.x = 0.0
                        self.cmd_vel_pub.publish(self.twist)
                        cv2.imshow('Camera View', image)
                        cv2.imshow('Bird Eye View',BEV_image)
                        cv2.waitKey(1)
                        time.sleep(10)
                        print("[INFO] Turtlebot3 starts again.")
                        self.Nfound = False
                else:
                    # if turtlebot3 has not detected Aruco Marker yet
                    cv2.putText(image, "Not Found Marker yet", (0, 64), font, 0.75, (0, 255, 0), 2, cv2.LINE_AA)
                    cv2.imshow('Camera View', image)
                    cv2.imshow('Bird Eye View',BEV_image)
                    cv2.waitKey(1)

rospy.init_node('lane_follower')
follower = Follower()
rospy.spin()
