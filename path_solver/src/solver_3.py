#!/usr/bin/env python
import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class LineFollower:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.twist = Twist()

    def camera_callback(self, data):
        image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([10, 100, 100])
        upper_yellow = np.array([50, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        height, width, _ = image.shape
        search_top = 3*height//4 + 40
        search_bot = 3*height//4 + 60
        mask[0:search_top, 0:width] = 0
        mask[search_bot:height, 0:width] = 0

        lower_purple = np.array([130, 100, 100])
        upper_purple = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_purple, upper_purple)
        mask2[0:search_top, 0:width] = 0
        mask2[search_bot:height, 0:width] = 0

        M = cv2.moments(mask)
        M2 = cv2.moments(mask2)

        if M2['m00'] > 0:
            print('-------------purple found---------------')
            cx = int(M2['m10']/M2['m00'])
            cy = int(M2['m01']/M2['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            err = cx - width//2
            self.twist.linear.x = 0.4
            self.twist.angular.z = -float(err) / 100
            for i in range(10):
                self.twist.linear.x = 0.4
                self.twist.angular.z = -float(err) / 100
                self.cmd_vel_pub.publish(self.twist)
                rospy.sleep(0.25)
                
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
            rospy.signal_shutdown('end of path found')
        elif M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            err = cx - width//2
            self.twist.linear.x = 0.4
            self.twist.angular.z = -float(err) / 100
            self.cmd_vel_pub.publish(self.twist)
        else:
            self.twist.linear.x = 0
            self.twist.angular.z = .5
            self.cmd_vel_pub.publish(self.twist)
        cv2.imshow('Original', image)
        cv2.imshow('HSV', hsv)
        cv2.imshow('MASK', mask)
        cv2.imshow('MASK_2', mask2)
        cv2.waitKey(1)

def main():

    rospy.init_node('line_follower_node')
    follower = LineFollower()
    rospy.spin()

if __name__ == "__main__":
    main()