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
from matplotlib import transforms

class Particle(object):
    def __init__(self, x, y, theta, weight):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight


class ParticleFilter(object):
    def __init__(self, num_particles, map, width, height, resolution, origin,
                 laser_min_range, laser_max_range, laser_min_angle, laser_max_angle):

        self.num_particles = num_particles
        self.laserscan_available = True
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
        #self.movimento = []
        #self.movimento.x = 0
        #self.movimento.y = 0
        #self.movimento.theta = 0

        # Relative motion since the last time particles were updated
        self.dx = 0
        self.dy = 0
        self.dyaw = 0

        self.particles = []

    def conversao_metros(self, x):
        return x * self.resolution + self.origin[0]

    def conversao_pixeis(self, x):
        a = ((x - self.origin[0]) / self.resolution).astype(int)
        return np.where((a >= self.width) | (a<0), 0, a)


    def espaco_livre(self):
        indices = np.argwhere(self.map == 254)
        arraymap = indices.tolist()
        self.tamanho_livre = len(arraymap)
        x = np.random.choice(self.tamanho_livre, size=self.num_particles, replace=True)
        self.position_particula = np.array(arraymap)[x]

    def init_particles(self):
        for i in self.position_particula:
            x = self.conversao_metros(i[1])
            y = self.conversao_metros(i[0])
            theta = np.random.uniform(-np.pi, np.pi)
            self.particles.append(Particle(x, y, theta, 1/self.num_particles))

    def predict_particle_odometry(self, particle, v, omega, dt):
        if (v >= 0.001 or omega >= 0.01):
            particle.theta = particle.theta + omega * dt
            particle.x = particle.x + v * np.cos(particle.theta) * dt
            particle.y = particle.y + v * np.sin(particle.theta) * dt


    

    def handle_observation(self, laser_scan):
        """Does prediction, weight update, and resampling."""
        self.laserscan_available  = False
        weight_total = 0
        for particle in self.particles:
            # for each particle, compute the laser scan difference
            particle.weight = self.get_prediction_error_squared(laser_scan, particle)
            weight_total += particle.weight
        
        if weight_total != 0:
            weights = np.array([particle.weight for particle in self.particles])
            weights /= weight_total
         
    
            self.resample()
        #for particle in self.particles:
        #    pyplot.plot(self.conversao_pixeis(particle.x), self.conversao_pixeis(particle.y), marker="o", markersize=2, markeredgecolor="red", markerfacecolor="green")
        #pyplot.imshow(self.map, cmap='gray')
        #pyplot.show()
        self.laserscan_available  = True
        

    def get_prediction_error_squared(self, laser_scan_msg, particle):
        
        row = self.conversao_pixeis(particle.x)
        col = self.conversao_pixeis(particle.y)

        if self.map[row][col] == 0 or self.map[row][col] == 205:
            return 0
     
        actual_ranges, angle = self.convert_laser(laser_scan_msg, self.laser_min_range, self.laser_max_range)
    
        predict_ranges = self.raytracing(particle.x, particle.y, particle.theta,
                                        self.laser_min_range, self.laser_max_range)
        
        diff = actual_ranges - predict_ranges[:,2]
        
        norm_error = 1/np.linalg.norm(diff)
        return norm_error


    
    def draw_line_until_dark_dot(self, x, y, ranges):
        #height, width = map.shape

        # Plot the initial point
        pyplot.scatter(self.conversao_pixeis(x), self.conversao_pixeis(y), color='blue')
        x = [self.conversao_pixeis(x)] * len(ranges[:,0])
        y = [self.conversao_pixeis(y)] * len(ranges[:,1])
        
        # Iterate over angles from 0 to 359 degrees with 1-degree intervals
        pyplot.plot([x, ranges[:,0]], [y, ranges[:,1]], color='red', linewidth=0.5)
        # Show the map with the red lines and initial point
        pyplot.imshow(self.map, cmap='gray')
        pyplot.show()

    def convert_laser(self, scan, min_range, max_range):
        step = 4
        angle = np.arange(0, 360, step)
        
        wanted_ranges = np.array([s if isinstance(s, (int, float)) else s[0] for s in scan[::-step]])
        mask = (wanted_ranges >= min_range) & (wanted_ranges <= max_range)
        real_ranges = np.where(mask, wanted_ranges, np.clip(wanted_ranges, min_range, max_range))
    
        return real_ranges, angle
    
    """ def raytracing(self, x, y, theta, min_range, max_range):
        ranges = [] 
        angles = np.arange(0,360, 1)
        phi = theta + np.radians(angles)
        r = np.arange(min_range, max_range, self.resolution)
        xm = x + r[:, np.newaxis] * np.cos(phi) 
        ym = y + r[:, np.newaxis] * np.sin(phi)
        row = self.conversao_pixeis(xm)
        col = self.conversao_pixeis(ym)
        flat_row = row.ravel()
        flat_col = col.ravel()
        np.where((self.map[flat_col, flat_row] == 0) | (self.map[flat_col, flat_row] == 205),0,0)
        #np.where(np.logical_or(self.map[col[:,:]][row[:,:]] == 0,self.map[col[:,:]][row[:,:]] == 205), xm, xm)    
        
                        
        ranges.append([row,col,r])  
        ranges = np.array(ranges)
        #self.draw_line_until_dark_dot(x, y, ranges)
        return ranges 
         """

    def raytracing(self, x, y, theta, min_range, max_range):
        ranges = []
        angles = np.arange(0, 360, 4)
        phi_values = theta + np.radians(angles)
        r_values = np.arange(0, max_range, self.resolution)

        for phi in phi_values:
            x_values = x + r_values * np.cos(phi)
            y_values = y + r_values * np.sin(phi)
            row_values = self.conversao_pixeis(x_values)
            col_values = self.conversao_pixeis(y_values)
            mask = (self.map[col_values, row_values] == 0) | (self.map[col_values, row_values] == 205)
            hit_indices = np.argmax(mask)

            if hit_indices > 0:
                row = row_values[hit_indices]
                col = col_values[hit_indices]
                r = r_values[hit_indices]
            else:
                row = row_values[-1]
                col = col_values[-1]
                r = max_range

            ranges.append([row, col, r])

        ranges = np.array(ranges)
        #self.draw_line_until_dark_dot(x, y, ranges)
        return ranges


    def resample(self):
        new_particles = []
        r = np.random.uniform(0, 1/self.num_particles)
        c = self.particles[0].weight    
        i = 0
        for m in range(self.num_particles):
            u = r + m * (1/self.num_particles)
            while u >= c:
                i += 1
                c += self.particles[i].weight
            new_particles.append(Particle(self.particles[i].x + np.random.normal(0, 0.05), self.particles[i].y + np.random.normal(0, 0.05), self.particles[i].theta + np.random.normal(0, 0.05), 1/self.num_particles))        
        c = 0
        self.particles = new_particles
       
    

    def sigmoid(self, x):
        """Numerically-stable sigmoid function."""
        if x >= 0:
            z = exp(-x)
            return 1 / (1 + z)
        else:
            # if x is less than zero then z will be small, denom can't be
            # zero because it's 1+z.
            z = exp(x)
            return z / (1 + z)

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

        self.pf = ParticleFilter(num_particles, self.map, self.width, self.height, self.resolution, self.origin, 0, 0,
                                 0, 0)
        self.pf.espaco_livre()
        self.pf.init_particles()
  
        self.last_scan = None

        rospy.Subscriber("/odom", Odometry, self.odometry_callback)
        rospy.Subscriber("/scan", LaserScan, self.laser_scan_callback)

        self.position_pub = rospy.Publisher("/particles", MarkerArray, queue_size=1)

    def odometry_callback(self, msg):
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z
        self.t2 = msg.header.stamp.secs + msg.header.stamp.nsecs * (10 ** -9)
        if self.t1 != 0 and self.pf.tamanho_livre is not None:
            for particle in self.pf.particles:
                self.pf.predict_particle_odometry(particle, v, omega, self.t2 - self.t1)
        self.t1 = self.t2

    def laser_scan_callback(self, msg):
        self.pf.laser_min_angle = msg.angle_min
        self.pf.laser_max_angle = msg.angle_max
        self.pf.laser_min_range = msg.range_min
        self.pf.laser_max_range = msg.range_max
        
        self.pf.handle_observation(msg.ranges)

    def timer(self):
        self.timer = rospy.Timer(rospy.Duration(0.5), self.publish)
        self.h_timerActivate = True

    def publish(self, msg):
        qz = [0] * self.num_particles
        qw = [0] * self.num_particles

        for t in range(len(self.pf.particles)):
            ([_, _, qz[t], qw[t]]) = tf.transformations.quaternion_from_euler(0, 0, self.pf.particles[t].theta)
        if self.pf.tamanho_livre is not None:
            for i in range(len(self.pf.particles)):
                msg = Marker()
                msg.action = msg.ADD
                msg.type = msg.ARROW
                msg.id = i
                msg.pose.position.x = self.pf.particles[i].x
                msg.pose.position.y = self.pf.particles[i].y
                msg.pose.orientation.w = qw[i]
                msg.pose.orientation.z = qz[i]
                msg.color.a = 0.8
                msg.color.r = 0.0
                msg.color.g = 1.0
                msg.color.b = 0.0
                msg.scale.x = 0.1
                msg.scale.y = 0.02
                msg.scale.z = 0.02
                #msg.scale.x = 0.1
                #msg.scale.y = 0.1
                #msg.scale.z = 0.1
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "base_footprint"
                if i > self.num_particles:
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
                                dtype='u1' if int(maxval) < 256 else byteorder + 'u2',
                                count=int(self.width) * int(self.height),
                                offset=len(header)
                                ).reshape((int(self.height), int(self.width)))

    def inicialization(self):
        map = self.read_pgm("/home/tomas/catkin_ws/src/sintetic/maps/gmapping_02.pgm", byteorder='<')
        with open('/home/tomas/catkin_ws/src/sintetic/maps/gmapping_02.yaml', 'r') as file:
            # Load the YAML contents
            yaml_data = yaml.safe_load(file)

        # Access the parameters
        resolution = yaml_data['resolution']
        origin = yaml_data['origin']

        clean_string = self.width.decode('utf-8').strip('b')
        self.width = int(clean_string)

        clean_string = self.height.decode('utf-8').strip('b')
        self.height = int(clean_string)
        # pyplot.imshow(map, pyplot.cm.gray)
        # pyplot.show()
        # gray = 205, free = 254, occupied = 0
        return map, resolution, origin


if __name__ == '__main__':
    num_particles = 500

    mcl = MonteCarloLocalization(num_particles)

    rospy.spin()
