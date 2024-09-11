#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point

def publish_coordinates():
    rospy.init_node('coordinate_publisher')
    pub = rospy.Publisher('/coordinates', Point, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        try:
            # Get user input for coordinates and theta
            x = float(input("Enter target x coordinate: "))
            y = float(input("Enter target y coordinate: "))
        except ValueError:
            rospy.logwarn("Invalid input. Please enter numeric values.")
            continue

        target = Point()
        target.x = x
        target.y = y
        target.z = 0
        pub.publish(target)
        rospy.loginfo(f"Published target coordinates: x={x}, y={y}")
        rate.sleep()

if __name__ == '__main__':
    publish_coordinates()
    
