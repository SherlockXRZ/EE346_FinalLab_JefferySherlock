#!/usr/bin/env python

from array import array
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

# Calculate the homography
#pts_src = numpy.array([[60,208], [251, 207], [119, 160], [200, 160]])
#pts_dst = numpy.array([[80,240], [175,240], [80, 80], [175, 80]])
#H, status = cv2.findHomography(pts_src, pts_dst)

#pts_src = numpy.float32([(145, 120), (115, 210), (205, 210), (175, 120)])
#pts_dst = numpy.float32([(130, 0), (130, 210), (190, 210), (190, 0)])
#H = cv2.getPerspectiveTransform(pts_src, pts_dst)

H = numpy.array([[-2.23471253e-01, -1.33270129e+00,  1.95755401e+02],
                                      [-1.94004843e-16, -2.16157649e+00,  2.78843367e+02],
                                      [-0.00000000e+00, -8.32938308e-03,  1.00000000e+00]])

class Follower:

        def __init__(self):

                self.bridge = cv_bridge.CvBridge()

                self.image_sub = rospy.Subscriber('camera/image',
                        Image, self.image_callback)

                self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=10)

                self.twist = Twist()

                print("[INFO] Node \"Follower\" Initializing ...")

        def image_callback(self, msg):

                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

                lower_yellow = numpy.array([ 26, 43, 46])
                upper_yellow = numpy.array([34, 255, 255])

                lower_white = numpy.array([0, 0, 80])
                upper_white = numpy.array([180, 30, 255])
                
                mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)
                mask2 = cv2.inRange(hsv, lower_white, upper_white)

                h, w, d = image.shape
                search_top = int(2*h/3)
                mask1[0:search_top, 0:w] = 0
                mask2[0:search_top, 0:w] = 0

                M1 = cv2.moments(mask1)
                M2 = cv2.moments(mask2)

                # get the Bird Eye View
                BEV_image = cv2.warpPerspective(image, H, (w, h))
                BEV_hsv = cv2.cvtColor(BEV_image, cv2.COLOR_BGR2HSV)

                BEV_mask1 = cv2.inRange(BEV_hsv, lower_yellow, upper_yellow)
                BEV_mask2 = cv2.inRange(BEV_hsv, lower_white, upper_white) 
                BEV_mask1[0:search_top, 0:w] = 0
                BEV_mask2[0:search_top, 0:w] = 0
                BEV_mask1[search_top:h, 0:int(round(w/3))] = 0
                BEV_mask1[search_top:h, int(round(2*w/3)):w] = 0
                BEV_mask2[search_top:h, 0:int(round(w/3))] = 0
                BEV_mask2[search_top:h, int(round(2*w/3)):w] = 0

                MB1 = cv2.moments(BEV_mask1)
                MB2 = cv2.moments(BEV_mask2)

                if MB1['m00'] > 0 and MB2['m00'] > 0:
                        px1 = int(MB1['m10']/MB1['m00'])
                        py1 = int(MB1['m01']/MB1['m00'])
                        px2 = int(MB2['m10']/MB2['m00'])
                        py2 = int(MB2['m01']/MB2['m00'])

                        fpt_px = int((px1 + px2)/2)
                        fpt_py = int((py1 + py2)/2 + 2*h/3)

                        cv2.circle(BEV_image, (px1, py1), 10, (0,255,255), -1)
                        cv2.circle(BEV_image, (px2, py2), 10, (255,255,255), -1)

                if M1['m00'] > 0 and M2['m00'] > 0:
                    cx1 = int(M1['m10']/M1['m00'])
                    cy1 = int(M1['m01']/M1['m00'])

                    cx2 = int(M2['m10']/M2['m00'])
                    cy2 = int(M2['m01']/M2['m00'])

                    fpt_x = (cx1 + cx2)/2
                    fpt_y = (cy1 + cy2)/2 + 2*h/3

                    cv2.circle(image, (cx1, cy1), 10, (0,255,255), -1)
                    cv2.circle(image, (cx2, cy2), 10, (255,255,255), -1)
                    cv2.circle(image, (fpt_x, fpt_y), 10, (128,128,128), -1)
                    
                    err = w/2 - fpt_x

                    self.twist.linear.x = 0.33
                    self.twist.angular.z = (err*90.0/160)/15
                    self.cmd_vel_pub.publish(self.twist)
                cv2.imshow("Camera_View", image)
                cv2.imshow("Bird_Eye_View", BEV_image)
                cv2.waitKey(1)

rospy.init_node('lane_follower')
follower = Follower()
rospy.spin()
