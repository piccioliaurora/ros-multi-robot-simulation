#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point, Twist
from math import atan2, sqrt
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class Robot2:
    def __init__(self):
        rospy.init_node('robot2_follower')
        self.vel_pub = rospy.Publisher('/robot2/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/robot1/pose', Point, self.robot1_pose_callback)
        rospy.Subscriber('/robot2/odom', Odometry, self.odom_callback)

        self.target_x = None
        self.target_y = None

    def robot1_pose_callback(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        (_, _, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def move_towards_target(self):
        if self.target_x is None or self.target_y is None:
            return

        goal_x = self.target_x
        goal_y = self.target_y

        inc_x = goal_x - self.x
        inc_y = goal_y - self.y

        distance_to_goal = sqrt(inc_x**2 + inc_y**2)

        angle_to_goal = atan2(inc_y, inc_x)

        speed = Twist()

        if distance_to_goal > 0.5:
            if abs(angle_to_goal - self.theta) > 0.1:
                speed.linear.x = 0.0
                speed.angular.z = 0.3 if angle_to_goal - self.theta > 0 else -0.3
            else:
                speed.linear.x = 0.6
                speed.angular.z = 0.0
        else:
            speed.linear.x = 0.0
            speed.angular.z = 0.0
            self.vel_pub.publish(speed)
            print("Arrived to Robot1!")
            rospy.signal_shutdown("Robot2 finished!")

        self.vel_pub.publish(speed)
        rospy.sleep(0.1)  

    def run(self):
        while not rospy.is_shutdown():
            self.move_towards_target()

if __name__ == '__main__':
    try:
        robot2 = Robot2()
        robot2.run()
    except rospy.ROSInterruptException:
        pass
