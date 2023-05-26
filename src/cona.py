#!/usr/bin/env python3
import rospy
import tf
import tf.transformations as tr
import math
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
                 laser_min_range, laser_max_range, laser_min_angle, laser_max_angle, mcl):

        self.num_particles = num_particles
    
        # Workspace boundaries
        self.map = map
        self.position_particula = []
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin = origin
        self.mcl = mcl
        self.tamanho_livre = None

        self.laser_max_angle = laser_max_angle
        self.laser_min_angle = laser_min_angle
        self.laser_max_range = laser_max_range
        self.laser_min_range = laser_min_range

        self.Q = np.zeros((90, 90))
        # Set the diagonal values to the desired value
        np.fill_diagonal(self.Q, 0.3)
        
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
            #x = self.conversao_metros(np.array(60))
            #y = self.conversao_metros(np.array(80))
            #theta = 0
            self.particles.append(Particle(x, y, theta, 1/self.num_particles))

    def predict_particle_odometry(self, particle, v, omega, dt):
        if (v >= 0.001 or omega >= 0.01):
            particle.theta = particle.theta + omega * dt + np.random.normal(0, 0.01)
            particle.x = particle.x + v * np.cos(particle.theta) * dt + np.random.normal(0, 0.0082)
            particle.y = particle.y + v * np.sin(particle.theta) * dt + np.random.normal(0, 0.0082)
    
    def handle_observation(self, laser_scan):
        """Does prediction, weight update, and resampling."""
        actual_ranges = self.convert_laser(laser_scan, self.laser_min_range, self.laser_max_range)

        [self.laser_diff(actual_ranges, particle) for particle in self.particles]
        
        weight_total = sum([particle.weight for particle in self.particles])
        for particle in self.particles:
                particle.weight = particle.weight / weight_total
        N_eff = 1 / sum([particle.weight**2 for particle in self.particles])
        #max_weight = np.argmax(self.particles[:,0].weight)
        #self.mcl.publish_max(max_weight)

        
        if N_eff <= 0.6 * self.num_particles:
            self.particles = self.resample()
        

    def laser_diff(self, actual_ranges, particle):
        
        row = self.conversao_pixeis(particle.x)
        col = self.conversao_pixeis(particle.y)

        if self.map[row][col] == 0 or self.map[row][col] == 205:
            particle.weight = particle.weight * 0.001
            return
        
        predict_ranges = self.raytracing(particle.x, particle.y, particle.theta, self.laser_max_range)
        predict_ranges = np.array(predict_ranges)
        diff = actual_ranges - predict_ranges[:,2]
        diff = np.array(diff)
        diff_transpose = diff[:, np.newaxis]

        numerador = exp(-0.5*np.dot(np.dot(diff, self.Q), diff_transpose))
        denominador = np.sqrt((2*np.pi)*np.linalg.det(self.Q))
        particle.weight = (numerador / denominador) * particle.weight 
    
    def convert_laser(self, scan, min_range, max_range):
        step = 4
        wanted_ranges = np.array([s for s in scan[::-step]])
        real_ranges = np.clip(wanted_ranges, min_range, max_range)
        return real_ranges

    def raytracing(self, x, y, theta, max_range):
        ranges = []

        # Precompute trigonometric values
        angles = np.arange(0, 360, 4)
        phi_values = theta + np.radians(angles)

        # Vectorized calculations
        r_values = np.arange(0, max_range, self.resolution)
        x_values = x + np.outer(r_values, np.cos(phi_values))
        y_values = y + np.outer(r_values, np.sin(phi_values))
        row_values = self.conversao_pixeis(x_values)
        col_values = self.conversao_pixeis(y_values)

        # Find wall indices
        mask = (self.map[col_values, row_values] == 0) | (self.map[col_values, row_values] == 205)
        hit_indices = np.argmax(mask, axis=0)

        for i, hit_index in enumerate(hit_indices):
            if hit_index > 0:
                row = row_values[hit_index, i]
                col = col_values[hit_index, i]
                r = r_values[hit_index]
            else:
                row = row_values[-1, i]
                col = col_values[-1, i]
                r = max_range

            ranges.append([row, col, r])

        ranges = np.array(ranges)
        
        self.draw_line_until_dark_dot(x, y, ranges)
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
            new_particles.append(Particle(self.particles[i].x, self.particles[i].y, self.particles[i].theta, 1/self.num_particles))        
        return new_particles

    def draw_line_until_dark_dot(self, x, y, ranges):
        #height, width = map.shape

        # Plot the initial point
        pyplot.scatter(self.conversao_pixeis(x), self.conversao_pixeis(y), color='blue')
        x = [self.conversao_pixeis(x)] * len(ranges[:,1])
        y = [self.conversao_pixeis(y)] * len(ranges[:,0])
        
        # Iterate over angles from 0 to 359 degrees with 1-degree intervals
        pyplot.plot([x, ranges[:,0]], [y, ranges[:,1]], color='red', linewidth=0.5)
        # Show the map with the red lines and initial point
        pyplot.imshow(self.map, cmap='gray')
        pyplot.show()


class MonteCarloLocalization(object):

    def __init__(self, num_particles):
        rospy.init_node('monte_carlo_localization', anonymous=True)

        self.width = 0
        self.height = 0
        [self.map, self.resolution, self.origin] = self.inicialization()
        self.num_particles = num_particles

        self.MkArray = MarkerArray()
       
        self.t1 = 0
        self.t2 = 0
        self.r = np.random.uniform(0.0, 1.0, self.num_particles)
        self.g = np.random.uniform(0.0, 1.0, self.num_particles)
        self.b = np.random.uniform(0.0, 1.0, self.num_particles)

        self.pf = ParticleFilter(num_particles, self.map, self.width, self.height, self.resolution, self.origin, 0, 0,
                                 0, 0, self)
        self.pf.espaco_livre()
        self.pf.init_particles()

        self.position_pub = rospy.Publisher("/particles", MarkerArray, queue_size=1)
        self.position_max_pub = rospy.Publisher("/particle", Marker, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odometry_callback)
        rospy.Subscriber("/scan", LaserScan, self.laser_scan_callback)

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

    def publish_max(self, max_weight_index):
        i = max_weight_index
        qz = tf.transformations.quaternion_from_euler(0, 0, self.pf.particles[i].theta)[2]
        qw = tf.transformations.quaternion_from_euler(0, 0, self.pf.particles[i].theta)[3]
        if self.pf.tamanho_livre is not None:
                marker = Marker()
                marker.action = Marker.ADD
                marker.type = Marker.SPHERE
                marker.pose.position.x = self.pf.particles[i].x
                marker.pose.position.y = self.pf.particles[i].y
                marker.pose.orientation.w = qw
                marker.pose.orientation.z = qz
                marker.color.a = 0.5
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.scale.x = 0.15
                marker.scale.y = 0.15
                marker.scale.z = 0.15
                marker.header.stamp = rospy.Time.now()
                marker.header.frame_id = "base_footprint"
                self.position_max_pub.publish(marker)
                
    def publish(self):
        self.MkArray.markers = []

        qz = [tf.transformations.quaternion_from_euler(0, 0, particle.theta)[2] for particle in self.pf.particles]
        qw = [tf.transformations.quaternion_from_euler(0, 0, particle.theta)[3] for particle in self.pf.particles]

        if self.pf.tamanho_livre is not None:
            marker_index = 0
            for i, particle in enumerate(self.pf.particles):
                marker = Marker()
                marker.action = Marker.ADD
                marker.type = Marker.ARROW
                marker.id = i
                marker.pose.position.x = particle.x
                marker.pose.position.y = particle.y
                #marker.pose.position.y = self.pf.conversao_metros(abs(160-self.pf.conversao_pixeis(particle.y)))
                
               
                marker.pose.orientation.w = qw[i]
                marker.pose.orientation.z = qz[i]
                marker.color.a = 0.8
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.scale.x = 0.15
                marker.scale.y = 0.02
                marker.scale.z = 0.02
                #marker.scale.x = 0.1
                #marker.scale.y = 0.1
                #marker.scale.z = 0.1
                marker.header.stamp = rospy.Time.now()
                marker.header.frame_id = "base_footprint"
                
                self.MkArray.markers.append(marker)
                
                marker_index += 1
                if marker_index > self.num_particles + 1:
                    self.MkArray.markers.pop(0)
                
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
        map = self.read_pgm("/home/tomas/catkin_ws/src/sintetic/maps/lab.pgm", byteorder='<')
        with open('/home/tomas/catkin_ws/src/sintetic/maps/lab.yaml', 'r') as file:
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
        # gray = 205, free = 254, occupied = 0
        return map, resolution, origin

    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()

if __name__ == '__main__':
    num_particles = 1
    
    mcl = MonteCarloLocalization(num_particles)

    mcl.run()
