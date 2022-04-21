#!/usr/bin/env python

import rospy, cv2, cv_bridge, math, numpy as np
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
		self.previous_x = 0.45

        def image_callback(self, msg):

                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

		# finding homography matrix
		#h, status = cv2.findHomography(pts_src, pts_dst)
		K = np.array([[265,0,160],[0,265,120],[0,0,1]])
		R = np.array([[1,0,1],[0,0,0],[0,-1,0]])
		t = np.array([[0,-0.115,0]]).T
		n = np.array([[0,-1,0]]).T
		d = 0.115
		H = np.dot(K,np.dot(R-np.dot(t,n.T)/d,np.linalg.inv(K)))
		H2 = np.array([[-0.434,-1.33,229],[0,-2.88,462],[0,-0.00833,1]])

		# homography process
		BEV = cv2.warpPerspective(image, H2, (320, 240))

		# fill the empty space with black triangles on left and right side of bottom
		#triangle1 = np.array([[0, 599], [0, 340], [200, 599]], np.int32)
		#triangle2 = np.array([[999, 599], [999, 340], [799, 599]], np.int32)
		#black = (0, 0, 0)
		#white = (255, 255, 255)
		#BEV = cv2.fillPoly(BEV, [triangle1, triangle2], black)
		
		hsv = cv2.cvtColor(BEV, cv2.COLOR_BGR2HSV)

                lower_yellow = np.array([20, 100, 100])
                upper_yellow = np.array([30, 255, 255])

                lower_white = np.array([0, 0, 80])
                upper_white = np.array([25, 43, 255])
                
                mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)
                mask2 = cv2.inRange(hsv, lower_white, upper_white)
                mask3 = cv2.inRange(hsv, lower_yellow, upper_yellow)
                mask4 = cv2.inRange(hsv, lower_white, upper_white)
		mask5 = cv2.inRange(hsv, lower_yellow, upper_yellow)
                mask6 = cv2.inRange(hsv, lower_white, upper_white)
                
		h, w, d = hsv.shape
                l = 20
		r = w - 21
		mask1[50:h, 0:w] = 0
		mask1[0:50, 0:l] = 0
		#mask1[0:50, r:w] = 0
		mask2[50:h, 0:w] = 0
		mask2[0:50, 0:l] = 0
		#mask2[0:50, r:w] = 0
		mask3[0:25, 0:w] = 0
		mask3[75:h, 0:w] = 0
		mask3[25:75, 0:l] = 0
		#mask3[25:75, r:w] = 0
                mask4[0:25, 0:w] = 0
		mask4[75:h, 0:w] = 0
		mask4[25:75, 0:l] = 0
		#mask4[25:75, r:w] = 0
		mask5[0:50, 0:w] = 0
		mask5[100:h, 0:w] = 0
		mask5[50:100, 0:l] = 0
		#mask5[50:100, r:w] = 0
                mask6[0:50, 0:w] = 0
		mask6[100:h, 0:w] = 0
		mask6[50:100, 0:l] = 0
		#mask6[50:100, r:w] = 0

                M1 = cv2.moments(mask1)
                M2 = cv2.moments(mask2)
                M3 = cv2.moments(mask3)
                M4 = cv2.moments(mask4)
                M5 = cv2.moments(mask5)
                M6 = cv2.moments(mask6)

                if M1['m00'] > 0 and M2['m00'] > 0 and M3['m00'] > 0 and M4['m00'] > 0 and M5['m00'] > 0 and M6['m00'] > 0:
                    cx1 = int(M1['m10']/M1['m00'])
                    cy1 = int(M1['m01']/M1['m00'])

                    cx2 = int(M2['m10']/M2['m00'])
                    cy2 = int(M2['m01']/M2['m00'])

                    cx3 = int(M3['m10']/M3['m00'])
                    cy3 = int(M3['m01']/M3['m00'])

                    cx4 = int(M4['m10']/M4['m00'])
                    cy4 = int(M4['m01']/M4['m00'])

                    cx5 = int(M5['m10']/M5['m00'])
                    cy5 = int(M5['m01']/M5['m00'])

                    cx6 = int(M6['m10']/M6['m00'])
                    cy6 = int(M6['m01']/M6['m00'])

                    m_xa = (cx1 + cx2)/2
		    m_ya = (cy1 + cy2)/2
		    m_xb = (cx3 + cx4)/2
		    m_yb = (cy3 + cy4)/2
		    m_xc = (cx5 + cx6)/2
		    m_yc = (cy5 + cy6)/2

                    cv2.circle(BEV, (cx1, cy1), 5, (255,0,255), -1)
                    cv2.circle(BEV, (cx2, cy2), 5, (255,255,0), -1)
                    cv2.circle(BEV, (cx3, cy3), 5, (255,0,255), -1)
                    cv2.circle(BEV, (cx4, cy4), 5, (255,255,0), -1)
                    cv2.circle(BEV, (cx5, cy5), 5, (255,0,255), -1)
                    cv2.circle(BEV, (cx6, cy6), 5, (255,255,0), -1)
                    cv2.circle(BEV, (m_xa, m_ya), 5, (128,128,128), -1)
		    cv2.circle(BEV, (m_xb, m_yb), 5, (128,128,128), -1)
		    cv2.circle(BEV, (m_xc, m_yc), 5, (128,128,128), -1)

                    # err = w/2 - fpt_x
		    alpha = math.atan2(m_xb-w/2, h-m_yb)
		    beta = math.pi/2 - alpha - math.atan2(m_yc-m_ya,m_xa-m_xc)
		    # !
		    #print alpha,beta,math.atan2(m_yb-m_ya,m_xa-m_xb)
		    PID_a = pid_controller(11, 0.00, 1)
		    PID_b = pid_controller(0.0000, 0.00, 0.00)
		    if alpha < 0.01 and self.twist.linear.x < 0.5:
                    	self.twist.linear.x = self.previous_x + 0.005
		    elif alpha < 0.01 and self.twist.linear.x >= 0.5:
			self.twist.linear.x = self.previous_x
		    else:
			self.twist.linear.x = 0.4
		    self.previous_x = self.twist.linear.x
		    print self.twist.linear.x
                    self.twist.angular.z = -PID_a.set_current_error(alpha)+PID_b.set_current_error(beta)
                    self.cmd_vel_pub.publish(self.twist)

                cv2.imshow("window", image)
		cv2.imshow("BEV", BEV)
                cv2.waitKey(1)

class pid_controller:

    def __init__(self, p_coef, i_coef, d_coef):

        self.kp = p_coef
        self.ki = i_coef
        self.kd = d_coef
        self._previous_error = 0.0
	self.sum = 0.0

    def set_current_error(self, error):

        output0 = error * self.kp

        error_diff = error - self._previous_error
        output1 = self.kd * error_diff

        self.sum += error
        output2 = self.ki * self.sum

        self._previous_error = error

        output = output0 + output1 + output2

        return output

rospy.init_node('lane_follower')
follower = Follower()
rospy.spin()
