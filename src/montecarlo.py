#!/usr/bin/env python
import numpy as np
import tf
import rospy
import bagpy
import math
from bagpy import bagreader
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry      
import pandas as pd
from sensor_msgs.msg import LaserScan

class Particula:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
    
    def prediction(self, v, omega, dt):

        self.x = self.x + v * math.cos(self.theta) * dt
        self.y = self.y + v * math.sin(self.theta) * dt
        self.theta = self.theta + omega * dt


class Subscriber ():
    def __init__(self):
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

    def odom_callback(self,msg):
        """ Args:
        x (float): The robot's x-coordinate.
        y (float): The robot's y-coordinate.
        theta (float): The robot's orientation angle in radians.
        v (float): The robot's linear velocity in m/s.
        omega (float): The robot's angular velocity in radians/s.
        dt (float): The time step between predictions in seconds."""
        
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        (_, _, self.theta) = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.v = msg.twist.twist.linear.x
        self.omega = msg.twist.twist.angular.z
        
    def scan_callback(self,msg):
        self.ranges = msg.ranges
        return self.ranges  
        


if __name__ == '__main__':
    rospy.init_node('montecarlo')
    Subscriber()
    rospy.spin()





