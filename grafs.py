#!/usr/bin/env python3
from visualization_msgs.msg import MarkerArray
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseWithCovarianceStamped
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

amcl_values = []
particles_values = []
resolution = 0
origin = [0, 0, 0]
width = 0
height = 0
map = None
fig = None
ax = None
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

def amcl_callback(msg):
    position = msg.pose.pose.position
    amcl_values.append(position)

def particles_callback(msg):
    positions = [particle.pose.position for particle in msg.markers]
    particles_values.extend(positions)

def get_position():
    global resolution, origin, width, height, map, scatter_amcl, scatter_particles, line_amcl, line_particles,scatter_particles2

    # Plot the AMCL positions
    amcl_x = [position.x for position in amcl_values]
    amcl_y = [position.y for position in amcl_values]

    amcl_x_pixels, amcl_y_pixels = conversao_pixeis(amcl_x, amcl_y) 

    # Plot the particle positions
    particles_x = [position.x for position in particles_values]
    particles_y = [position.y for position in particles_values]

    particles_x_pixels, particles_y_pixels = conversao_pixeis(particles_x, particles_y)

    # Update the scatter plot with the new data
    scatter_amcl.set_offsets(np.column_stack((amcl_x_pixels, amcl_y_pixels)))
    scatter_particles.set_offsets(np.column_stack((particles_x_pixels, particles_y_pixels)))

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
    global resolution, origin, width, height, map, fig, ax, scatter_amcl, scatter_particles,fig2,ax2,line_amcl,line_particles,scatter_particles2

    map, width, height = read_pgm("/home/ubuntu/catkin_ws/src/montecarlo/maps/elevador.pgm", byteorder='<')

    with open("/home/ubuntu/catkin_ws/src/montecarlo/maps/elevador.yaml", 'r') as file:
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
    scatter_amcl = ax.scatter([], [], color='red', s=5, label='AMCL Positions')
    scatter_particles = ax.scatter([], [], color='blue', s=5, label='Particle Positions')
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

    initialization()
    signal.signal(signal.SIGINT, save_data)

    rate = rospy.Rate(1)  # Set the rate (10 Hz in this example)
    while not rospy.is_shutdown():
        get_position()
        rate.sleep()


def save_data(signal, frame):
    global amcl_values, particles_values
    
    data = pd.DataFrame({'ACML:': amcl_values})
    data1 = pd.DataFrame({'Particles:': particles_values})

    timestamp = time.strftime("%Y%m%d-%H%M%S")

    data.to_csv(f'~/amcl_csv/amcl_results_{timestamp}.csv', index=False)
    data1.to_csv(f'~/particle_csv/particles_results_{timestamp}.csv', index=False)
    
    sys.exit(0)


if __name__ == '__main__':
    main()
    
   