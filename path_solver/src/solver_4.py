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
        MAX_AREA = 4000000
        MIN_AREA = 400000
        image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Black mask
        if count == 0:
            lower = np.array([0, 0, 0])
            upper = np.array([180, 255, 30])
            mask = cv2.inRange(hsv, lower, upper)

            height, width, _ = image.shape
            search_top = 3*height//6
            search_bot = 3*height//6 + 80
            mask[0:search_top, 0:width] = 0
            mask[search_bot:height, 0:width] = 0

            M = cv2.moments(mask)
        # Blue mask
        elif count == 1:
            lower = np.array([100, 124, 0])
            upper = np.array([128, 255, 255])
            mask = cv2.inRange(hsv, lower, upper)

            height, width, _ = image.shape
            search_top = 3*height//6
            search_bot = 3*height//6 + 80
            mask[0:search_top, 0:width] = 0
            mask[search_bot:height, 0:width] = 0

            M = cv2.moments(mask)

        # Orange Mask
        elif count == 2:
            lower = np.array([13, 59, 59])
            upper = np.array([50, 255, 255])
            mask = cv2.inRange(hsv, lower, upper)

            height, width, _ = image.shape
            search_top = 3*height//6
            search_bot = 3*height//6 + 80
            mask[0:search_top, 0:width] = 0
            mask[search_bot:height, 0:width] = 0

            M = cv2.moments(mask)

        # Green Mask
        elif count == 3:
            lower = np.array([52, 124, 74])
            upper = np.array([102, 255, 255])
            mask = cv2.inRange(hsv, lower, upper)

            height, width, _ = image.shape
            search_top = 3*height//6
            search_bot = 3*height//6 + 80
            mask[0:search_top, 0:width] = 0
            mask[search_bot:height, 0:width] = 0

            M = cv2.moments(mask)

        # Red mask
        elif count == 4:
            lower = np.array([0, 43, 0])
            upper = np.array([12, 255, 255])
            mask = cv2.inRange(hsv, lower, upper)

            height, width, _ = image.shape
            search_top = 3*height//6
            search_bot = 3*height//6 + 80
            mask[0:search_top, 0:width] = 0
            mask[search_bot:height, 0:width] = 0

            M = cv2.moments(mask)

        # All blocks found
        elif count == 5:
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
            count = 0
            return

        # Move toward target
        if MAX_AREA > M['m00'] > 0 and found is False:
            print(M['m00'])
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            err = cx - width//2
            self.twist.linear.x = 0.4
            self.twist.angular.z = -float(err) / 100
            self.cmd_vel_pub.publish(self.twist)

        # Stop in front of target
        elif M['m00'] > MAX_AREA and found is False:
            print('------', M['m00'])
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
            found = True
            rospy.sleep(5)

        # Move away from target
        elif M['m00'] > MIN_AREA and found is True:
            print(M['m00'])
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)
            err = cx - width//2
            self.twist.linear.x = -0.4
            self.twist.angular.z = -float(err) / 100
            self.cmd_vel_pub.publish(self.twist)

        # Stop at MIN_AREA 
        elif M['m00'] <= MIN_AREA and found is True:
            found = False
            count += 1
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)

        # Scan
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
    follower = LineFollower()
    rospy.spin()


if __name__ == "__main__":
    main()
