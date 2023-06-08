#!/usr/bin/env python3
from visualization_msgs.msg import MarkerArray
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import pandas as pd
import time
import rospy
import yaml
import pandas as pd
import re
import numpy
import numpy as np
import signal
import sys


particle_x = -1
particle_y = 0
theta = 0
t1 = 0
t2 = 0
v = 0
omega = 0

amcl_values = []
time_values= []
amcl_time_values= []
particles_values = []
allParticles_values = []
dead_x=[]
dead_y=[]
resolution = 0
origin = [0, 0, 0]
width = 0
height = 0
map = None
fig = None
ax = None
scatter_dead_reckoning = None
scatter_amcl = None
scatter_particles = None
scatter_particles2 = None
fig2=None
ax2=None
line_amcl=None
line_particles = None

def conversao_pixeis(x, y):
    a = ((x - origin[0]) / resolution).astype(int)
    x = np.where((a >= width) | (a < 0), 0, a)
    b = (height - ((y - origin[1]) / resolution)).astype(int)
    y = np.where((b >= height) | (b < 0), 0, b)
    return x, y

def odometry_callback(msg):
    global t1, t2, v, omega, particle_x, particle_y, theta
    v = msg.twist.twist.linear.x
    omega = msg.twist.twist.angular.z
    t2 = msg.header.stamp.secs + msg.header.stamp.nsecs * (10 ** -9)
    if t1 != 0 and t2 > t1:
        particle_x = particle_x + v * np.cos(theta) * (t2 - t1)
        particle_y = particle_y + v * np.sin(theta) * (t2 - t1)
        theta = theta + omega * (t2 - t1)
    t1 = t2

    dead_x.append(particle_x)
    dead_y.append(particle_y)

def amcl_callback(msg):
    position = msg.pose.pose.position
    amcl_values.append(position)
    tempo = [msg.header.stamp.secs + msg.header.stamp.nsecs * (10 ** -9)]
    amcl_time_values.extend(tempo)

def particles_callback(msg):
    positions = [particle.pose.position for particle in msg.markers]
    particles_values.extend(positions)
    tempo = [particle.header.stamp.secs + particle.header.stamp.nsecs * (10 ** -9) for particle in msg.markers]
    time_values.extend(tempo)

def allParticles_callback(msg):
    positions = [particle.pose.position for particle in msg.markers]
    allParticles_values.extend(positions)
    tempo = [particle.header.stamp.secs + particle.header.stamp.nsecs * (10 ** -9)for particle in msg.markers]
    time_values.extend(tempo)

def allParticles_position():
    global resolution, origin, width, height, map, scatter_amcl, scatter_particles, line_amcl, line_particles,scatter_particles2

    # Plot the AMCL positions
    amcl_x = [position.x for position in amcl_values]
    amcl_y = [position.y for position in amcl_values]

    amcl_x_pixels, amcl_y_pixels = conversao_pixeis(amcl_x, amcl_y) 

    # Plot the particle positions
    particles_x = [position.x for position in allParticles_values]
    particles_y = [position.y for position in allParticles_values]

    particles_x_pixels, particles_y_pixels = conversao_pixeis(particles_x, particles_y)

    # Update the scatter plot with the new data
    scatter_amcl.set_offsets(np.column_stack((amcl_x_pixels, amcl_y_pixels)))
    scatter_particles.set_offsets(np.column_stack((particles_x_pixels, particles_y_pixels)))

    # Redraw the plot
    plt.pause(0.001)

def get_position():
    global resolution, origin, width, height, map, scatter_amcl, scatter_particles, line_amcl, line_particles,scatter_particles2,scatter_dead_reckoning
    
    # Plot the AMCL positions
    amcl_x = [position.x for position in amcl_values]
    amcl_y = [position.y for position in amcl_values]

    amcl_x_pixels, amcl_y_pixels = conversao_pixeis(amcl_x, amcl_y) 

    # Plot the particle positions
    particles_x = [position.x for position in particles_values]
    particles_y = [position.y for position in particles_values]

    particles_x_pixels, particles_y_pixels = conversao_pixeis(particles_x, particles_y)

    dead_x_pixels, dead_y_pixels = conversao_pixeis(dead_x, dead_y)
    
    # Update the scatter plot with the new data
    scatter_amcl.set_offsets(np.column_stack((amcl_x_pixels, amcl_y_pixels)))
    scatter_particles.set_offsets(np.column_stack((particles_x_pixels, particles_y_pixels)))
    #scatter_dead_reckoning.set_offsets(np.column_stack((dead_x_pixels,dead_y_pixels)))
    """ line_amcl[0].set_data(amcl_x_pixels, amcl_y_pixels)
    scatter_particles2.set_offsets(np.column_stack((particles_x_pixels, particles_y_pixels)))
    ax2.relim()  # Update the limits of the axes
    ax2.autoscale_view()  # Auto-scale the view

    # Redraw the plot
    fig2.canvas.draw()
    fig2.canvas.flush_events() """

    # Redraw the plot
    plt.pause(0.001)

def read_pgm(filename, byteorder='>'):
    global width, height
    with open(filename, 'rb') as f:
        buffer = f.read()
    try:
        header, width, height, maxval = re.search(
            b"(^P5\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n])*"
            b"(\d+)\s(?:\s*#.*[\r\n]\s)*)", buffer).groups()
    except AttributeError:
        raise ValueError("Not a raw PGM file: '%s'" % filename)
    return numpy.frombuffer(buffer,
                            dtype='u1' if int(maxval) < 256 else byteorder + 'u2',
                            count=int(width) * int(height),
                            offset=len(header)
                            ).reshape((int(height), int(width))), int(width), int(height)

def initialization():
    global resolution, origin, width, height, map, fig, ax, scatter_amcl, scatter_particles,fig2,ax2,line_amcl,line_particles,scatter_particles2,scatter_dead_reckoning

    map, width, height = read_pgm("/home/tomas/catkin_ws/src/sintetic/maps/piso5.pgm", byteorder='<')

    with open("/home/tomas/catkin_ws/src/sintetic/maps/piso5.yaml", 'r') as file:
        # Load the YAML contents
        yaml_data = yaml.safe_load(file)

    # Access the parameters
    resolution = yaml_data['resolution']
    origin = np.array(yaml_data['origin'])

    clean_string = str(width)
    width = int(clean_string)

    clean_string = str(height)
    height = int(clean_string)

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.imshow(map, cmap='gray', origin='lower')
    scatter_amcl = ax.scatter([], [], color='red', s=5, label='amcl Position')
    scatter_particles = ax.scatter([], [], color='blue', s=5, label='Inicial Particle Positions')
    #scatter_dead_reckoning = ax.scatter([], [], color='green', s=5, label='Dead Reckoning')
    ax.legend()

    """ fig2, ax2 = plt.subplots(figsize=(10, 6))
    line_amcl = ax2.plot([], [], color='green', linestyle='-', linewidth=2, label='AMCL')
    scatter_particles2 = ax2.scatter([], [], color='purple', s=5, label='Particle Positions') """
    #line_particles = ax2.plot([], [], color='yellow', linestyle='-', linewidth=2, label='particle positions')

    #ax2.legend()

def main():
    plt.ion()
    rospy.init_node('graphs_node', anonymous=True)

    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, amcl_callback)
    rospy.Subscriber("/MaxWeight", MarkerArray, particles_callback)
    rospy.Subscriber("/particles", MarkerArray, allParticles_callback)
    rospy.Subscriber("/odom",Odometry,odometry_callback)
    initialization()
    signal.signal(signal.SIGINT, save_data)

    rate = rospy.Rate(1)  
    while not rospy.is_shutdown():
        get_position()
        #allParticles_position()
        rate.sleep()


def save_data(signal, frame):
    global amcl_values, particles_values, time_values
    
    data = pd.DataFrame({'ACML:': amcl_values,'Time':amcl_time_values})
    data1 = pd.DataFrame({'Particles:': particles_values, 'Time':time_values})

    timestamp = time.strftime("%Y%m%d-%H%M%S")

    data.to_csv(f'~/amcl_csv/amcl_results_{timestamp}.csv', index=False)
    data1.to_csv(f'~/particle_csv/particles_results_{timestamp}.csv', index=False)
    
    sys.exit(0)


if __name__ == '__main__':
    main()
    
   