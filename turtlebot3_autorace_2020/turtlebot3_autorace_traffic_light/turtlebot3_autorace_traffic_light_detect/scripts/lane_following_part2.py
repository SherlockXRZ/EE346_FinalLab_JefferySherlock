#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
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

        def image_callback(self, msg):

                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                
                h, w, d = image.shape

                ori = numpy.array([[50, 210], [250,210], [110, 160],[200, 160]])
                dst = numpy.array([[80, 240], [220,240], [80, 80],[220, 80]])
                
                hinfo, status = cv2.findHomography(ori, dst)
                homoOut = cv2.warpPerspective(image, hinfo, (w,h))
                
                hsv = cv2.cvtColor(homoOut, cv2.COLOR_BGR2HSV)
                
                cv2.circle(image, (50, 210), 5, (0,255,255), -1)
                cv2.circle(image, (250,210), 5, (0,255,255), -1)
                cv2.circle(image, (110, 160), 5, (0,255,255), -1)
                cv2.circle(image, (200, 160), 5, (0,255,255), -1)
                
                cv2.circle(homoOut, (80, 240), 5, (0,255,255), -1)
                cv2.circle(homoOut, (220,240), 5, (0,255,255), -1)
                cv2.circle(homoOut, (80, 80), 5, (0,255,255), -1)
                cv2.circle(homoOut, (220, 80), 5, (0,255,255), -1)
                
                lower_yellow = numpy.array([ 10, 10, 10])
                upper_yellow = numpy.array([255, 255, 250])
                lower_white = numpy.array([0, 0, 80])
                upper_white = numpy.array([180, 43, 220])
                
                mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)
                mask2 = cv2.inRange(hsv, lower_white, upper_white)

                search_top = int(h/3)
                search_wide=int(w/2)
                search_thin=int(w/8)
                mask1[0:search_top, 0:w] = 0
                mask2[0:search_top, 0:w] = 0
                mask1[0:h, 0:search_thin] = 0
                mask1[0:h, w-search_wide:w] = 0
                mask2[0:h, 0:search_wide] = 0
                mask2[0:h, w-search_thin:w] = 0

                M1 = cv2.moments(mask1)
                M2 = cv2.moments(mask2)

                if M1['m00'] > 0:
                    cx1 = int(M1['m10']/M1['m00'])
                    cy1 = int(M1['m01']/M1['m00'])

                    cx2 = int(M2['m10']/M2['m00'])
                    cy2 = int(M2['m01']/M2['m00'])

                    fpt_x = int((cx1 + cx2)/2)
                    fpt_y = int((cy1 + cy2)/2 + 2*h/3)

                    cv2.circle(homoOut, (cx1, cy1), 10, (0,255,255), -1)
                    cv2.circle(homoOut, (cx2, cy2), 10, (255,255,255), -1)
                    cv2.circle(homoOut, (fpt_x, fpt_y), 10, (128,128,128), -1)

                    err = w/2 - fpt_x

                    self.twist.linear.x = 0.3
                    self.twist.angular.z = (err*90.0/160)/10
                    self.cmd_vel_pub.publish(self.twist)
                cv2.imshow("window", image)
                cv2.imshow("window2",homoOut)
                cv2.waitKey(1)

rospy.init_node('lane_follower')
follower = Follower()
rospy.spin()
