#!/usr/bin/env python3
import rospy
import tf
import tf.transformations as tr
from std_msgs.msg import String, Header, ColorRGBA
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from math import sqrt, cos, sin, pi, atan2
from threading import Thread, Lock
from math import pi, log, exp
import random
import numpy as np
import sys
import pickle

import numpy as np
import rospy
import bagpy
import pickle
import yaml
import random
from bagpy import bagreader
from matplotlib import pyplot
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry  
from geometry_msgs.msg import PoseStamped    
import pandas as pd                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             

from visualization_msgs.msg import Marker, MarkerArray
import re
import numpy

class Particle(object):
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

class ParticleFilter(object):
    def __init__(self, num_particles, map, width, height, resolution, origin,
                 laser_min_range, laser_max_range, laser_min_angle, laser_max_angle):

        self.num_particles = num_particles

        # Workspace boundaries
        self.map = map
        self.position_particula = []
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin = origin

     
        self.tamanho_livre = None

        self.laser_max_angle = laser_max_angle
        self.laser_min_angle = laser_min_angle
        self.laser_max_range = laser_max_range
        self.laser_min_range = laser_min_range

        # Previous odometry measurement of the robot
        self.last_robot_odom = None

        # Current odometry measurement of the robot
        self.robot_odom = None

        # Relative motion since the last time particles were updated
        self.dx = 0
        self.dy = 0
        self.dyaw = 0

        self.particles = []
        self.weights = []

    def conversao_metros(self,x):
        return (x)*self.resolution+self.origin[0]

    def espaco_livre(self):
        arraymap = []
        for x in range(self.width):
            for y in range(self.height):
                if(self.map[x][y] == 254):
                    arraymap.append([x,y])
        self.tamanho_livre = len(arraymap)
        x = np.random.randint(self.tamanho_livre, size=self.num_particles)
        for i in x:
            self.position_particula.append(arraymap[i])
        self.position_particula=np.array(self.position_particula)
        

    def init_particles(self):
        self.particles = Particle(self.conversao_metros(self.position_particula[:,1]), self.conversao_metros(self.position_particula[:,0]), np.random.uniform(-np.pi, np.pi,self.num_particles))
        

    def predict_particle_odometry(self, particle, v, omega, dt):
        if(v >= 0.001 or omega>= 0.01):
            particle.x = particle.x + v * np.cos(particle.theta) * dt
            particle.y = particle.y + v * np.sin(particle.theta) * dt
            particle.theta = particle.theta + omega * dt

        
            


    def metric_to_grid_coords(self, x, y):
        """Converts metric coordinates to occupancy grid coordinates"""

        gx = (x - self.origin[0]) / self.resolution
        gy = (y - self.origin[1]) / self.resolution
        row = min(max(int(gy), 0), self.height)
        col = min(max(int(gx), 0), self.width)
        return (row, col)

class MonteCarloLocalization(object):

    def __init__(self, num_particles):
        rospy.init_node('monte_carlo_localization', anonymous=True)
  
        self.width = 0
        self.height = 0
        [self.map, self.resolution, self.origin] = self.inicialization()
        self.num_particles = num_particles
        
        self.MkArray = MarkerArray()
        self.timer()
        self.t1 = 0
        self.t2 = 0
        self.r = np.random.uniform(0.0, 1.0, self.num_particles)
        self.g = np.random.uniform(0.0, 1.0, self.num_particles)
        self.b = np.random.uniform(0.0, 1.0, self.num_particles)

        self.pf = ParticleFilter(num_particles, self.map, self.width, self.height, self.resolution, self.origin, 0, 0, 0, 0)
        self.pf.espaco_livre()
        self.pf.init_particles()
        self.last_scan = None

        rospy.Subscriber("/odom", Odometry, self.odometry_callback)
        #rospy.Subscriber("/scan", LaserScan, self.laser_scan_callback)
        print(self.pf.particles.theta)
        self.position_pub = rospy.Publisher("/particles", MarkerArray, queue_size=1)

    def odometry_callback(self, msg):
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z
        self.t2 = msg.header.stamp.secs + msg.header.stamp.nsecs * (10**-9)
        if self.t1 != 0 and self.pf.tamanho_livre != None:
            self.pf.predict_particle_odometry(self.pf.particles, v, omega, self.t2-self.t1)
        self.t1 = self.t2


    #def laser_scan_callback(self, msg):
    #    self.pf.laser_min_angle = msg.angle_min
    #    self.pf.laser_max_angle = msg.angle_max
    #    self.pf.laser_min_range = msg.range_min
    #    self.pf.laser_max_range = msg.range_max
#
    #    dt_since_last_scan = 0
#
    #    if self.last_scan:
    #        dt_since_last_scan = (msg.header.stamp - self.last_scan.header.stamp).to_sec()
#
    #
    #    self.pf.handle_observation(msg, dt_since_last_scan)
#
    #    self.pf.dx = 0
    #    self.pf.dy = 0
    #    self.pf.dyaw = 0
#
    #    self.last_scan = msg

    def timer(self):
        self.timer = rospy.Timer(rospy.Duration(1), self.publish)
        self.h_timerActivate = True

    def publish(self, msg):
        qz = [0]*self.num_particles
        qw = [0]*self.num_particles
 
        for t in range(len(self.pf.particles.x)):
            ([_, _, qz[t], qw[t]]) = tf.transformations.quaternion_from_euler(0, 0, self.pf.particles.theta[t])
        if(self.pf.tamanho_livre != None):    
            for i in range(len(self.pf.particles.x)):
                msg = Marker()
                msg.action = msg.ADD
                msg.type = msg.ARROW
                msg.id = i
                msg.pose.position.x = self.pf.particles.x[i]
                msg.pose.position.y = self.pf.particles.y[i]
                msg.pose.orientation.w = qw[i]
                msg.pose.orientation.z = qz[i]
                msg.color.a = 1.0
                msg.color.r = self.r[i]
                msg.color.g = self.g[i]
                msg.color.b = self.b[i]
                msg.scale.x = 0.25
                msg.scale.y = 0.05
                msg.scale.z = 0.05
                #msg.scale.x = 0.02
                #msg.scale.y = 0.02
                #msg.scale.z = 0.02
                msg.header.stamp = rospy.Time.now() 
                msg.header.frame_id = "base_footprint"
                if(i>self.num_particles):
                    self.MkArray.markers.pop(0)
                self.MkArray.markers.append(msg)
            self.position_pub.publish(self.MkArray)   


    def read_pgm(self, filename, byteorder='>'):
        with open(filename, 'rb') as f:
            buffer = f.read()
        try:
            header, self.width, self.height, maxval = re.search(
                b"(^P5\s(?:\s*#.*[\r\n])*"
                b"(\d+)\s(?:\s*#.*[\r\n])*"
                b"(\d+)\s(?:\s*#.*[\r\n])*"
                b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
        except AttributeError:
            raise ValueError("Not a raw PGM file: '%s'" % filename)
        return numpy.frombuffer(buffer,
                                dtype='u1' if int(maxval) < 256 else byteorder+'u2',
                                count=int(self.width)*int(self.height),
                                offset=len(header)
                                ).reshape((int(self.height), int(self.width)))


    def inicialization(self):
        map = self.read_pgm("/home/tomas/catkin_ws/src/sintetic/maps/quadrado.pgm", byteorder='<')
        with open('/home/tomas/catkin_ws/src/sintetic/maps/quadrado.yaml', 'r') as file:
        # Load the YAML contents
            yaml_data = yaml.safe_load(file)

        # Access the parameters
        resolution = yaml_data['resolution']
        origin = yaml_data['origin']

        clean_string = self.width.decode('utf-8').strip('b')
        self.width = int(clean_string)

        clean_string = self.height.decode('utf-8').strip('b')
        self.height = int(clean_string)
        #pyplot.imshow(map, pyplot.cm.gray)
        #pyplot.show()
        #gray = 205, free = 254, occupied = 0
        return map, resolution, origin

if __name__ == '__main__':
    num_particles = 50

    mcl = MonteCarloLocalization(num_particles)

    rospy.spin()
