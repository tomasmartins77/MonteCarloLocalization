#!/usr/bin/env python3

import csv
from scipy import interpolate
import os
import pandas as pd
import numpy as np
import resampy
import matplotlib.pyplot as plt

def interpolate_csv(folder_name, arg):
    data_frames = []
    max_len = 0

    for filename in os.listdir(folder_name):
        if filename.endswith(".csv"):
            # Read the CSV file
            with open(os.path.join(folder_name, filename), "r") as file:
                df = pd.read_csv(os.path.join(folder_name, filename))  # Use the full file path
                
                if len(df) > max_len:
                    max_len = len(df)
                data_frames.append(df)
    
    x_avg = [0.0] * arg
    y_avg = [0.0] * arg
    theta_avg = [0.0] * arg
    time_avg = [0.0] * arg

    for i,df in enumerate(data_frames):
        
        x = df['x']
        x = np.array(x)
        x = resampy.resample(x,len(x)/arg,  1)
        x_avg += x 
        x_avg = np.array(x_avg)
 
        y = df['y']
        y = np.array(y)
        y = resampy.resample(y,len(y)/arg,  1)
       
        y_avg += y
        y_avg = np.array(y_avg)

        theta = df['theta']
        theta = np.array(theta)
        theta = resampy.resample(theta,len(theta)/arg,  1)
        theta = np.array(theta)
        theta = np.unwrap(theta)
        theta_avg += theta
        theta_avg = np.array(theta_avg)

        time = df['Time']
        time = np.array(time)
        time = resampy.resample(time,len(time)/arg,  1)
        time -= time[0]
        time_avg += time
        
        
        aux = i

    aux +=1
    x_avg = x_avg/(aux)    
    y_avg = y_avg/(aux)
    time_avg = time_avg/(aux)
    theta_avg = theta_avg/(aux)
    
    return x_avg, y_avg, theta_avg, time_avg

 

def calculate_RMSE(x_amcl,x_particle,y_amcl,y_particle):
    return np.sqrt((x_amcl-x_particle)**2+(y_amcl-y_particle)**2)

# Example usage - 17
x_particle, y_particle, theta_particle, time_particle = interpolate_csv('particle_csv', 152 )
x_amcl, y_amcl, theta_amcl, time_amcl = interpolate_csv('amcl_csv', 152)
#x_dead, y_dead, theta_dead, time_dead = interpolate_csv('dead', 187)


def calculate_RMSE_theta(theta1, theta2):
    return np.sqrt((theta1-theta2)**2)

def unwrap2(theta_rad):
    for dead in theta_rad:
        while(dead > np.pi or dead < -np.pi):
                            if dead > np.pi:
                                dead -= 2*np.pi
                            elif dead < -np.pi:
                                dead += 2*np.pi  
    return theta_rad
                                
common_time = np.arange(
    max(time_amcl.min(), time_particle.min()),
    min(time_amcl.max(), time_particle.max()),
    1/152 *(min(time_amcl.max(), time_particle.max())-max(time_amcl.min(), time_particle.min()))  # Use a smaller time step, e.g., 0.1, for smoother interpolation
)

rmse_values_particle = calculate_RMSE(x_amcl,x_particle,y_amcl,y_particle)
#rmse_values_dead = calculate_RMSE(x_amcl,x_dead,y_amcl,y_dead)

fig = plt.figure()
#plt.plot(common_time, rmse_values_dead, label='Dead Reckoning')
plt.plot(common_time, rmse_values_particle,label='Position Estimate')
plt.legend()
plt.xlabel('Time (s)')
plt.ylabel('RMSE (m)')
plt.xlim(0,common_time.max())
plt.show()

#theta_dead = unwrap2(theta_dead)
#rmse_theta = calculate_RMSE_theta(theta_amcl,theta_dead)


#fig = plt.figure()
#plt.plot(common_time, rmse_theta)
#plt.show()