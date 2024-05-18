#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from geometry_msgs.msg import Twist, Point

class Robot1:
    def __init__(self):
        rospy.init_node('robot1_detector')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/robot1/camera/image_raw', Image, self.image_callback)
        self.laser_sub = rospy.Subscriber('/robot1/scan', LaserScan, self.laser_callback)
        self.odom_sub = rospy.Subscriber('/robot1/odom', Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=1)
        self.robot1_pose_pub = rospy.Publisher('/robot1/pose', Point, queue_size=1)

        self.lower_red = np.array([0, 100, 100])
        self.upper_red = np.array([10, 255, 255])

        self.min_range = 0
        self.image_width = 1920
        self.image_height = 1080
        self.obstacle_detected = False
        self.red_object_detected = False
        self.object_centered = False
        self.final_pose = None
        self.stopped = False

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
            return
        
        #converto l'immagine nel formato HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        #serve per ottenere solamente il colore rosso
        mask = cv2.inRange(hsv_image, self.lower_red, self.upper_red)
        
        #serve per ottenere i contorni dell'oggetto rosso
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            self.red_object_detected = True

            #calcolo il centro dell'oggetto rosso relativo all'immagine
            moments = cv2.moments(contours[0])
            if moments['m00'] != 0:
                cx = int(moments['m10'] / moments['m00'])
                cy = int(moments['m01'] / moments['m00'])
                
                #calcolo l'errore orizzontale e verticale dal centro dell'immagine 
                #indicano di quanto l'oggetto è spostato rispetto al centro lungo gli assi x e y
                error_x = cx - self.image_width / 2
                error_y = cy - self.image_height / 2

                #controllo se è abbastanza al centro
                if abs(error_x) <= 10 and abs(error_y) <= 10:
                    self.object_centered = True
                #nel caso in cui non sia centrato calcolo i comandi di velocità
                else:
                    #calcolo la velocita angolare proporzionale all'errore orizzontale
                    angular_z = -error_x / (self.image_width / 2)
                    #calcolo la velocità lineare proporzionale all'errore verticale
                    linear_x = max(0.1, min(0.5, 0.5 - error_y / (self.image_height / 2)))
                    
                    twist_msg = Twist()
                    twist_msg.linear.x = linear_x
                    twist_msg.angular.z = angular_z
                    self.cmd_vel_pub.publish(twist_msg)

    def laser_callback(self, data):
        self.min_range = min(data.ranges)
        if self.min_range < 0.5:  
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def odom_callback(self, data):
        #aggionro la posizione finale in base all'odometria
        self.final_pose = data.pose.pose.position

    def stop_robot(self):
        self.stopped=True
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        print("I stopped myself!")
        self.publish_robot1_pose()
        print("I published my final pose!")
        rospy.signal_shutdown("Robot1 finished")
    
    #nel caso in cui non trovo un oggetto rosso subito, il robot vaga nella stanza circolarmente
    def wander(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.2  
        twist_msg.angular.z = 0.5  
        self.cmd_vel_pub.publish(twist_msg)

    def publish_robot1_pose(self):
        if self.final_pose is not None:
            pose_msg = Point()
            pose_msg.x = self.final_pose.x
            pose_msg.y = self.final_pose.y
            pose_msg.z = self.final_pose.z
            self.robot1_pose_pub.publish(pose_msg)

    def run(self):
        #mi muovo finchè non trovo un oggetto rosso
        while not rospy.is_shutdown() and not robot1.red_object_detected:
            if not robot1.obstacle_detected:
                robot1.wander()  
            else:
                robot1.stop_robot()

        if robot1.red_object_detected:
            print("Red object found!")

            #il robot gira finchè non centra l'oggetto rosso
            while not rospy.is_shutdown() and not robot1.object_centered:
                if not robot1.obstacle_detected:
                    rospy.sleep(0.1)  
                else:
                    robot1.stop_robot()

            #quando l'oggetto rosso è centrato si muove dritto
            twist_msg = Twist()
            twist_msg.linear.x = 0.5  
            robot1.cmd_vel_pub.publish(twist_msg)


if __name__ == '__main__':
    try:
        robot1 = Robot1()
        robot1.run()
    except rospy.ROSInterruptException:
        pass
