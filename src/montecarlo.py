#!/usr/bin/env python3
import numpy as np
import tf
import rospy
import bagpy
import pickle
import yaml
import random
from bagpy import bagreader
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry  
from geometry_msgs.msg import PoseStamped    
import pandas as pd
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray

class Particula:
    def __init__(self, x, y, theta, numeroParticulas):
        self.x = x
        self.y = y
        self.theta = theta
        self.qz = [0] * numeroParticulas
        self.qw = [0] * numeroParticulas
    
    def prediction(self, v, omega, dt):

        self.x = self.x + v * np.cos(self.theta) * dt
        self.y = self.y + v * np.sin(self.theta) * dt
        self.theta = self.theta + omega * dt
        for i in range(len(self.theta)):
            ([_, _, self.qz[i], self.qw[i]]) = tf.transformations.quaternion_from_euler(0, 0, self.theta[i])
        

            
    
    def update(self):
        print('to do')

    """ def InitializeParticles(self, numeroParticulas):
        self.particles = []
        free_space_indices = np.where(self.map_data == 0)
        free_space_indices = np.column_stack(free_space_indices)
        for i in range(numeroParticulas):
            rand_index = random.choice(range(len(free_space_indices)))
            x = free_space_indices[rand_index][0]
            y = free_space_indices[rand_index][1]
            theta = random.uniform(-np.pi, np.pi)
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = x * self.resolution + self.origin_x
            pose.pose.position.y = y * self.resolution + self.origin_y
            pose.pose.orientation.z = np.sin(theta / 2)
            pose.pose.orientation.w = np.cos(theta / 2)
            self.particles.append(pose) """



 
class mcl():
    def __init__(self):
        self.t1 = 0
        self.odom_sub=rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.scan_sub= rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.position_pub = rospy.Publisher("/particles", MarkerArray, queue_size=1)
        self.timer()
        self.arrayx = []
        self.arrayy = []
        self.MkArray = MarkerArray()
        self.resolution = 0.0500000

    def odom_callback(self,msg):
        self.x = msg.pose.pose.position.x 
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        (_, _, self.theta) = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.v = msg.twist.twist.linear.x
        self.omega = msg.twist.twist.angular.z
        self.t2 = msg.header.stamp.secs + msg.header.stamp.nsecs * (10**-9)
        if self.t1 != 0:
            Particula.prediction(self.particles, self.v, self.omega, self.t2-self.t1)
        self.t1 = self.t2
        
        
    def scan_callback(self,msg):
        self.scan = msg.ranges

    def conversao_metros(self,x):
        return (x-75)*self.resolution
    
    def map_callback(self, msg):
        # Convert the 1D array to a 2D numpy array
        width = msg.info.width
        height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.map_data = np.array(msg.data).reshape((height, width))
        for x in range(width):
            for y in range(height):
                if(self.map_data[x,y] == 100):
                    self.arrayx.append(x)
                    self.arrayy.append(y)

        max_x = max(self.arrayx)
        max_y = max(self.arrayy)
        min_x = min(self.arrayx)
        min_y = min(self.arrayy)
        self.numero_particulas = 500

        self.particles = Particula(np.random.uniform(self.conversao_metros(min_x), self.conversao_metros(max_x), self.numero_particulas), np.random.uniform(self.conversao_metros(min_y), self.conversao_metros(max_y), self.numero_particulas), np.random.uniform(-np.pi, np.pi,self.numero_particulas), self.numero_particulas)

    def timer(self):
        self.timer = rospy.Timer(rospy.Duration(1), self.publish)
        self.h_timerActivate = True

    def publish(self, msg):
        for i in range(len(self.particles.x)):
            msg = Marker()
            msg.action = msg.ADD
            msg.type = msg.SPHERE
            msg.id = i;
            msg.pose.position.x = self.particles.x[i]
            msg.pose.position.y = self.particles.y[i]
            msg.pose.orientation.z = self.particles.qz[i]
            msg.pose.orientation.w = self.particles.qw[i]
            msg.color.a = 1.0
            msg.color.r = 0.0
            msg.color.g = 1.0
            msg.color.b = 0.0
            #msg.scale.x = 0.25
            #msg.scale.y = 0.05
            #msg.scale.z = 0.05
            msg.scale.x = 0.02
            msg.scale.y = 0.02
            msg.scale.z = 0.02
            msg.header.stamp = rospy.Time.now() 
            msg.header.frame_id = "base_footprint"
            if(i>self.numero_particulas):
                self.MkArray.markers.pop(0)
            self.MkArray.markers.append(msg)
        self.position_pub.publish(self.MkArray)   


        
if __name__ == '__main__':
    rospy.init_node('montecarlo')
    mcl()
    rospy.spin()





