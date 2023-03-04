#!/usr/bin/env python
import rospy
import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

# Determines which color to search for
count = 0
# Determines whether to move toward or away from box
found = False


class LineFollower:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.twist = Twist()

    def camera_callback(self, data):
        global count
        global found
        MAX_AREA = 3000000
        MIN_AREA = 400000
        image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Yellow Mask
        lower_yellow = np.array([10, 100, 100])
        upper_yellow = np.array([50, 255, 255])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        height, width, _ = image.shape
        search_top = 3*height//4 + 40
        search_bot = 3*height//4 + 60
        mask_yellow[0:search_top, 0:width] = 0
        mask_yellow[search_bot:height, 0:width] = 0

        M_yellow = cv2.moments(mask_yellow)

        # Black mask
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 30])
        mask_black = cv2.inRange(hsv, lower_black, upper_black)

        height, width, _ = image.shape
        search_top = 3*height//6
        search_bot = 3*height//6 + 80
        mask_black[0:search_top, 0:width] = 0
        mask_black[search_bot:height, 0:width] = 0

        M_black = cv2.moments(mask_black)

        # Blue mask
        lower_blue = np.array([100, 124, 0])
        upper_blue = np.array([128, 255, 255])
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        height, width, _ = image.shape
        search_top = 3*height//6
        search_bot = 3*height//6 + 80
        mask_blue[0:search_top, 0:width] = 0
        mask_blue[search_bot:height, 0:width] = 0

        M_blue = cv2.moments(mask_blue)

        # Red mask
        lower_red = np.array([0, 43, 0])
        upper_red = np.array([12, 255, 255])
        mask_red = cv2.inRange(hsv, lower_red, upper_red)

        height, width, _ = image.shape
        search_top = 3*height//6
        search_bot = 3*height//6 + 80
        mask_red[0:search_top, 0:width] = 0
        mask_red[search_bot:height, 0:width] = 0

        M_red = cv2.moments(mask_red)

        if M_blue['m00'] > MAX_AREA:
            self.twist.linear.x = 0
            self.twist.angular.z = -0.5
            self.cmd_vel_pub.publish(self.twist)

        elif M_red['m00'] > MAX_AREA:
            self.twist.linear.x = 0
            self.twist.angular.z = 0.5
            self.cmd_vel_pub.publish(self.twist)

        elif M_black['m00'] > (MAX_AREA + 1000000):
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
            rospy.signal_shutdown('job complete')

        elif M_yellow['m00'] > 0:
            cx = int(M_yellow['m10']/M_yellow['m00'])
            cy = int(M_yellow['m01']/M_yellow['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            err = cx - width//2
            self.twist.linear.x = 0.4
            self.twist.angular.z = -float(err) / 100
            self.cmd_vel_pub.publish(self.twist)
        else:
            self.twist.linear.x = 0
            self.twist.angular.z = .5
            self.cmd_vel_pub.publish(self.twist)
        cv2.imshow('Black', mask_black)
        cv2.imshow('Red', mask_red)
        cv2.imshow('Yellow', mask_yellow)
        cv2.imshow('Blue', mask_blue)

        cv2.waitKey(1)


def main():

    rospy.init_node('line_follower_node')
    follower = LineFollower()
    rospy.spin()


if __name__ == "__main__":
    main()
