#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.previous_error = 0
        self.integral = 0

    def compute(self, target, current):
        self.error = target - current
        self.integral += self.error
        self.derivative = self.error - self.previous_error
        self.previous_error = self.error
        return self.Kp * self.error + self.Ki * self.integral + self.Kd * self.derivative

class GoalSubscriber:
    def __init__(self):
        rospy.init_node('goal_subscriber', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/coordinates', Point, self.send_goal)
        rospy.Subscriber('/odom', Odometry, self.update_position)
        self.current_position = (0, 0)
        self.current_orientation = 0
        self.rate = rospy.Rate(10)

        self.linear_pid = PID(Kp=1, Ki=0, Kd=0.1)
        self.angular_pid = PID(Kp=1, Ki=0, Kd=0.1)

        self.max_linear_speed = 0.5 
        self.max_angular_speed = 1.0 


    def update_position(self, msg):
        self.current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = self.quaternion_to_euler(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        self.current_orientation = yaw


    
    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z


    def send_goal(self, data):
        target_x = data.x
        target_y = data.y
        rospy.loginfo(f"Received goal: x={target_x}, y={target_y}")

        distance = math.sqrt((target_x - self.current_position[0]) ** 2 +
                             (target_y - self.current_position[1]) ** 2)
        angle_to_goal = math.atan2(target_y - self.current_position[1],
                                   target_x - self.current_position[0])

        #Rotate towards the goal
        while abs(angle_to_goal - self.current_orientation) > 0.05 and not rospy.is_shutdown():
            angle_diff = angle_to_goal - self.current_orientation
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

            angular_speed = self.angular_pid.compute(angle_diff, 0)
            angular_speed = max(min(angular_speed, self.max_angular_speed), -self.max_angular_speed)

            twist = Twist()
            twist.linear.x = 0
            twist.angular.z = angular_speed
            self.pub.publish(twist)
            self.rate.sleep()

            
            rospy.sleep(0.1)

        #Move towards the goal
        while distance > 0.01 and not rospy.is_shutdown():
            distance = math.sqrt((target_x - self.current_position[0]) ** 2 +
                                 (target_y - self.current_position[1]) ** 2)
            angle_to_goal = math.atan2(target_y - self.current_position[1],
                                       target_x - self.current_position[0])
            angle_diff = angle_to_goal - self.current_orientation
            angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

            linear_speed = self.linear_pid.compute(distance, 0)
            angular_speed = self.angular_pid.compute(angle_diff, 0)

            # Apply speed limits
            linear_speed = max(min(linear_speed, self.max_linear_speed), -self.max_linear_speed)
            angular_speed = max(min(angular_speed, self.max_angular_speed), -self.max_angular_speed)

            twist = Twist()
            twist.linear.x = linear_speed
            twist.angular.z = angular_speed
            self.pub.publish(twist)
            self.rate.sleep()


        self.stop_robot()

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.pub.publish(twist)

if __name__ == '__main__':
    subscriber = GoalSubscriber()
    rospy.spin()