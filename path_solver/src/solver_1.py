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
        self.image_sub = rospy.Subscriber(
            "/camera/rgb/image_raw", Image, self.camera_callback)
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
        # Change unneccessary pixels to 0
        mask[0:search_top, 0:width] = 0
        mask[search_bot:height, 0:width] = 0
        # Capture moments
        M = cv2.moments(mask)
        if M['m00'] > 0:  # zero-order moment. Determines the area of blob
            cx = int(M['m10']/M['m00']) # first-order moment for center of x
            cy = int(M['m01']/M['m00']) # second-order moment for center of y
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
        cv2.waitKey(1)


def main():

    rospy.init_node('line_follower_node')
    rospy.spin()


if __name__ == "__main__":
    main()
