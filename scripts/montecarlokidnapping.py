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
import scipy
from matplotlib import transforms

class Particle(object):
    def __init__(self, x, y, theta, weight):
        self.x = x
        self.x_laser = x
        self.y = y
        self.y_laser = y
        self.theta = theta
        self.theta_laser = theta
        self.weight = weight
        
        
class ParticleFilter(object):
    def __init__(self, num_particles, map, width, height, resolution, origin,
                 laser_min_range, laser_max_range, laser_min_angle, laser_max_angle, mcl):

        self.num_particles = num_particles
        
        # Workspace boundaries
        self.map = map
        self.mapa = np.where(self.map == 205, 0, self.map)
        self.mapa = scipy.ndimage.distance_transform_edt(self.mapa)
        
        self.position_particula = []
        #self.pesos_particula = np.array([[0.0]*round(0.01*self.num_particles), [0.0]*round(0.01*self.num_particles), [0.0]*round(0.01*self.num_particles), [0.0]*round(0.01*self.num_particles)]) #pesos das particulas/ posicao x/ posicao y
        self.pesos_particula = np.array([[0.0]*10, [0.0]*10, [0.0]*10, [0.0]*10]) #pesos das particulas/ posicao x/ posicao y
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin = origin
        self.mcl = mcl
        self.tamanho_livre = None
        self.indices = []
        self.laser_max_angle = laser_max_angle
        self.laser_min_angle = laser_min_angle
        self.laser_max_range = laser_max_range
        self.laser_min_range = laser_min_range
        self.weight_average =0
        self.weight_slow =0
        self.weight_fast =0
        self.sigma = 20
        self.min_weight = [0,0]
        self.denominador = np.sqrt(2*np.pi*(self.sigma)) #Fixo, logo metendo aqui assim acelera as contas
        self.particles = []
        self.wallcolission = 0

    def conversao_metros(self, x, y):
        x = x * self.resolution + self.origin[0]
        y = (self.height - y) * self.resolution + self.origin[1]
        return x, y

    def conversao_pixeis(self, x, y):
        a = ((x - self.origin[0]) / self.resolution).astype(int)
        x = np.where((a >= self.width) | (a < 0), 0, a)
        b = (self.height - ((y - self.origin[1]) / self.resolution)).astype(int)
        y = np.where((b >= self.height) | (b < 0), 0, b)
        return x, y

    def espaco_livre(self):
        self.indices = np.argwhere(self.map == 254)
        indices_final = []
        for i in self.indices:
            if i[0] > 310 and i[0] < 430 and i[1] > 310 and i[1] < 340:
                indices_final.append(i)
        #indices_final = np.array(indices_final)
        indices_final = np.array(self.indices)
        self.tamanho_livre = len(indices_final)
        x = np.random.choice(self.tamanho_livre, size=self.num_particles, replace=True)
        self.position_particula = np.array(indices_final)[x]
        self.tamanho_livre = len(self.indices)


    def init_particles(self):
        for i in self.position_particula:
            x, y = self.conversao_metros(i[1], i[0])
            theta = np.random.uniform(0, 2*np.pi)

            #x, y = self.conversao_metros(np.array(np.random.uniform(40, 80)), np.array(np.random.uniform(60, 100)))
            #theta = 0
            self.particles.append(Particle(x, y, theta, 1/self.num_particles))

    def predict_particle_odometry(self, particle, v, omega, dt):
        if (v >= 0.001 or v <= -0.001 or omega >= 0.01 or omega <= -0.01):
            particle.theta = particle.theta + omega * dt + np.random.normal(0, 0.002)
            particle.x = particle.x + v * np.cos(particle.theta) * dt + np.random.normal(0, 0.004)
            particle.y = particle.y + v * np.sin(particle.theta) * dt + np.random.normal(0, 0.004)
    
    def update_particle(self, laser_scan, v, omega):
        """weight update, and resampling."""
        actual_ranges = self.convert_laser(laser_scan, self.laser_min_range, self.laser_max_range)
        if (v >= 0.001 or v <= -0.001 or omega >= 0.01 or omega <= -0.01): 
            for particle in self.particles:
                particle.x_laser = particle.x
                particle.y_laser = particle.y
                particle.theta_laser = particle.theta   
            [self.laser_diff(actual_ranges, particle) for particle in self.particles] 
            weight_total = sum([particle.weight for particle in self.particles])
            if weight_total == 0:
                self.particles = self.reset_particulas(self.pesos_particula[:,0])
            else: 
                self.weight_average = 0;
                for particle in self.particles:
                        self.weight_average += particle.weight *(1/(self.num_particles-self.wallcolission))
                        particle.weight = particle.weight / weight_total
                
                self.weight_fast += 0.8* (self.weight_average - self.weight_fast)
                self.weight_slow += 0.1* (self.weight_average - self.weight_slow)
                N_eff = 1 / sum([particle.weight**2 for particle in self.particles])
                self.mcl.publish_max(self.pesos_particula)
                for i in range(len(self.pesos_particula[0,:])):
                    self.pesos_particula[0,i] = 0;
                self.min_weight[0] = 0;
                self.wallcolission = 0
                novas_particulas = 0 
                #print("weight_fast", round(self.weight_fast,5), "weight_slow", round(self.weight_slow,5), "weight_average", self.weight_average)
                if N_eff <= 0.5 * self.num_particles:
                    b = np.random.uniform(0,1, size = self.num_particles)
                    maximo = max(0, 1.0 - self.weight_fast/self.weight_slow)
                    #print("maximo", maximo)
                    for i in b:
                        if(i < maximo):
                            novas_particulas+=1
                        
                    print("novas_particulas", novas_particulas)
                    self.particles = self.resample(novas_particulas)
    
    def reset_particulas(self, pesos_particula):
        new_particles = []
        indices_final = []
        x,y = self.conversao_pixeis(pesos_particula[1], pesos_particula[2])
        for i in self.indices:
            if i[0] > y - 30 and i[0] < y + 30 and i[1] > x - 30 and i[1] < x + 30:
                indices_final.append(i)
        indices_final = np.array(indices_final)
        arraymap = indices_final.tolist()
        tamanho_livre = len(arraymap)
        x = np.random.choice(tamanho_livre, size=self.num_particles, replace=True)
        position_particula = np.array(arraymap)[x]
        for i in position_particula:
            x, y = self.conversao_metros(i[1], i[0])
            theta = np.random.uniform(pesos_particula[3] - 1/4*np.pi, pesos_particula[3] + 1/4*np.pi)
            new_particles.append(Particle(x, y, theta, 1/self.num_particles))   
        return new_particles
    

    def laser_diff(self, actual_ranges, particle):
        
        row, col = self.conversao_pixeis(particle.x_laser, particle.y_laser)
        if self.map[col][row] == 0 or self.map[col][row] == 205:
            particle.weight = 0
            self.wallcolission+=1
            return
        
        predict_ranges = self.raytracing(particle.x_laser, particle.y_laser, particle.theta_laser, self.laser_max_range)
        predict_ranges = np.array(predict_ranges)

        actual_ranges = np.around(actual_ranges, 2)
        diff = actual_ranges -  predict_ranges[:,2]
        diff = np.array(diff)
        
        diff_transpose = diff[:, np.newaxis]
        numerador = np.exp(-0.5*np.dot(diff, diff_transpose)*(1/self.sigma))
        particle.weight = (numerador[0] / self.denominador) * particle.weight
        a = np.where(self.pesos_particula[0,:] == 0)
        if(np.size(a) == 0):
            if(self.min_weight[0] < particle.weight):
                self.pesos_particula[0,self.min_weight[1]] = particle.weight
                self.pesos_particula[1,self.min_weight[1]] = particle.x
                self.pesos_particula[2,self.min_weight[1]] = particle.y
                self.pesos_particula[3,self.min_weight[1]] = particle.theta
                i = np.argmin(self.pesos_particula[0,:])
                self.min_weight[0] = self.pesos_particula[0,i]
                self.min_weight[1] = i
        else:
            if(self.min_weight[0] == 0):   #Apenas interessa a primeira vez iteração
                self.min_weight[0] = particle.weight
                self.min_weight[1] = a[0]
            elif(self.min_weight[0] > particle.weight):
                self.min_weight[0] = particle.weight
                self.min_weight[1] = a[0]
            self.pesos_particula[0,a[0]] = particle.weight
            self.pesos_particula[1,a[0]] = particle.x
            self.pesos_particula[2,a[0]] = particle.y
            self.pesos_particula[3,a[0]] = particle.theta
    
    def convert_laser(self, scan, min_range, max_range):
        step = 4
        wanted_ranges = np.array([s for s in scan[::step]])
        real_ranges = np.clip(wanted_ranges, min_range, max_range)
        return real_ranges

    def raytracing(self, x, y, theta, max_range):
        ranges = []
        angles = np.arange(0, 360, 4)
        # Vectorized calculations
        r_values = np.arange(0, self.laser_max_range, self.resolution)
        phi_values = theta + np.radians(angles)
        x_values = x + np.outer(r_values, np.cos(phi_values))
        y_values = y + np.outer(r_values, np.sin(phi_values))
        row_values, col_values = self.conversao_pixeis(x_values, y_values)
        
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
        #self.draw_line_until_dark_dot(x, y, ranges)
        
        return ranges
    
    def resample(self, novas_particulas):
        new_particles = []
        if(novas_particulas != self.num_particles):
            r = np.random.uniform(0, 1/(self.num_particles-novas_particulas))
            c = self.particles[0].weight    
            i = 0
        
            for m in range(self.num_particles-novas_particulas):
                u = r + m * (1/(self.num_particles-novas_particulas))
                while u >= c:
                    i += 1
                    c += self.particles[i].weight  
                new_particles.append(Particle(self.particles[i].x, self.particles[i].y, self.particles[i].theta, 1/self.num_particles))

        if novas_particulas > 0:
            for i in range(novas_particulas):
                h = np.random.choice(self.tamanho_livre, 1, replace=True)
                x,y = self.conversao_metros(self.indices[h,1], self.indices[h,0])
                theta = np.random.uniform(0, 2*np.pi)
                new_particles.append(Particle(x[0], y[0], theta, 1/self.num_particles))      
        return new_particles

    def draw_line_until_dark_dot(self, x, y, ranges):
        # Convert the ranges to pixel coordinates
        x, y = self.conversao_pixeis(x, y)
        pyplot.scatter(x, y, color='blue')
        x = np.array([x] * len(ranges[:,1]))
        y = np.array([y] * len(ranges[:,0]))
        pyplot.scatter(x, y, color='blue')
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
        self.MkArraymax = MarkerArray()
       
        self.t1 = 0
        self.t2 = 0
        self.v = 0
        self.omega = 0
        self.r = np.random.uniform(0.0, 1.0, self.num_particles)
        self.g = np.random.uniform(0.0, 1.0, self.num_particles)
        self.b = np.random.uniform(0.0, 1.0, self.num_particles)

        self.pf = ParticleFilter(num_particles, self.map, self.width, self.height, self.resolution, self.origin, 0, 0,
                                 0, 0, self)
        self.pf.espaco_livre()
        self.pf.init_particles()

        self.position_pub = rospy.Publisher("/particles", MarkerArray, queue_size=1)
        self.position_max_pub = rospy.Publisher("/MaxWeight", MarkerArray, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odometry_callback)
        rospy.Subscriber("/scan", LaserScan, self.laser_scan_callback)
        self.latest_scan = None

    def odometry_callback(self, msg):
        self.v = msg.twist.twist.linear.x
        self.omega = msg.twist.twist.angular.z
        self.t2 = msg.header.stamp.secs + msg.header.stamp.nsecs * (10 ** -9)
        print(self.t2)
        if self.t1 != 0 and self.pf.tamanho_livre is not None and (self.t2 > 375 or self.t2 < 355):
            for particle in self.pf.particles:
                self.pf.predict_particle_odometry(particle, self.v, self.omega, self.t2 - self.t1)
        self.t1 = self.t2
        

    def laser_scan_callback(self, msg):
        self.latest_scan = msg
    
    def laser_scan_update(self):
        if self.latest_scan is not None and (self.t2 > 375 or self.t2 < 355):
            self.pf.laser_min_angle = self.latest_scan.angle_min
            self.pf.laser_max_angle = self.latest_scan.angle_max
            self.pf.laser_min_range = self.latest_scan.range_min
            self.pf.laser_max_range = self.latest_scan.range_max
            self.pf.update_particle(self.latest_scan.ranges, self.v, self.omega)
            self.latest_scan = None

    def publish_max(self, pesos_particulas):
        self.MkArraymax.markers = []
        if self.pf.tamanho_livre is not None:  
                marker_index = 0; 
                x = 0
                y = 0
                div = 0
                for i in range(len(pesos_particulas[0,:])):
                    x += pesos_particulas[1,i]*pesos_particulas[0,i]
                    y += pesos_particulas[2,i]*pesos_particulas[0,i]
                    while(pesos_particulas[3,i] > np.pi or pesos_particulas[3,i] < -np.pi):
                        if pesos_particulas[3,i] > np.pi:
                            pesos_particulas[3,i] -= 2*np.pi
                        elif pesos_particulas[3,i] < -np.pi:
                            pesos_particulas[3,i] += 2*np.pi
                    div += pesos_particulas[0,i]
                sum_sin = sum(w * math.sin(angle) for w, angle in zip(pesos_particulas[0,:], pesos_particulas[3,:]))
                sum_cos = sum(w * math.cos(angle) for w, angle in zip(pesos_particulas[0,:], pesos_particulas[3,:]))
                theta = math.atan2(sum_sin, sum_cos)
                x = x/div
                y = y/div
                qz = tf.transformations.quaternion_from_euler(0, 0, theta)[2]
                qw = tf.transformations.quaternion_from_euler(0, 0, theta)[3]
                marker = Marker()
                marker.action = Marker.ADD
                marker.type = Marker.SPHERE
                marker.id = 1
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.orientation.w = qw
                marker.pose.orientation.z = qz
                marker.color.a = 1
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                marker.header.stamp = rospy.Time.now()
                marker.header.frame_id = "map"
                self.MkArraymax.markers.append(marker)
                marker_index += 1
                if marker_index > 1:
                    self.MkArraymax.markers.pop(0)
                self.position_max_pub.publish(self.MkArraymax.markers)
                    
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
                marker.pose.orientation.w = qw[i]
                marker.pose.orientation.z = qz[i]
                marker.color.a = 0.2
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
                marker.header.frame_id = "map"

                #x, y = self.pf.conversao_pixeis(particle.x, particle.y)
                #pyplot.scatter(x, y, color='blue')
                
                self.MkArray.markers.append(marker)
                
                marker_index += 1
                if marker_index > self.num_particles + 1:
                    self.MkArray.markers.pop(0)
            #pyplot.imshow(self.map, cmap='gray')
            #pyplot.show()
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
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.laser_scan_update()
            self.publish()
            rate.sleep()

if __name__ == '__main__':
    num_particles = 800
    
    mcl = MonteCarloLocalization(num_particles)

    mcl.run()