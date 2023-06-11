#!/usr/bin/env python3

import os
import csv
import matplotlib.pyplot as plt
import math
import numpy as np
import pandas as pd
import tf.transformations as tf
from scipy.interpolate import interp1d, griddata
from collections import defaultdict
from scipy.interpolate import UnivariateSpline
# Function to read the last row of a CSV file
def read_last_row(csv_file):
    with open(csv_file, 'r') as file:
        reader = csv.reader(file)
        rows = list(reader)
        if rows:
            last_row = rows[-1]
            return last_row
        else:
            return None

# Function to extract the values from the given format
""" def extract_values(row):
    values = row[0].split('\n')
    x = float(values[0].split(':')[1])
    y = float(values[1].split(':')[1])
    return x, y """

# Path to the first folder containing CSV files
folder1 = os.path.expanduser('~/amcl_csv')

# Path to the second folder containing CSV files
folder2 = os.path.expanduser('~/particle_csv')

folder3 = os.path.expanduser('~/dead')

# Create dictionaries to store the cumulative positions and count for each position
ground_truth_positions = defaultdict(lambda: [0, 0, 0])  # [sum_x, sum_y, count]
simulated_positions = defaultdict(lambda: [0, 0, 0])     # [sum_x, sum_y, count]
dead_positions = defaultdict(lambda: [0, 0, 0])     # [sum_x, sum_y, count]

# Variable to store the differences in x values
x_differences = []

# Variable to store the differences in y values
y_differences = []

diff = []
distance = []
""" def calculate_RSMD(x_truth, x_sim, y_truth, y_sim):
    diff_squared_i = (x_truth - x_sim)**2 + (y_truth - y_sim)**2
    return diff_squared_i

def calculate_error_percentage(x_truth, x_sim,y_truth,y_sim):
    abs_diff_x = abs(x_truth - x_sim)
    abs_diff_y = abs(y_truth - y_sim)
    percentage_error = (avg_distance / magnitude_ground_truth) * 100

# Process the ground truth CSV files
for filename in os.listdir(folder1):
    if filename.endswith(".csv"):
        with open(os.path.join(folder1, filename), "r") as file:
            reader = csv.reader(file)
            next(reader)  # Skip header row if present
            for row in reader:
                x = float(row[0])
                y = float(row[1])
                ground_truth_positions[filename[:-4]][0] += x
                ground_truth_positions[filename[:-4]][1] += y
        ground_truth_positions[filename[:-4]][2] += 1

# Process the simulated CSV files
for filename in os.listdir(folder2):
    if filename.endswith(".csv"):
        with open(os.path.join(folder2, filename), "r") as file:
            reader = csv.reader(file)
            next(reader)  # Skip header row if present
            for row in reader:
                x = float(row[0])
                y = float(row[1]) 
                simulated_positions[filename[:-4]][0] += x
                simulated_positions[filename[:-4]][1] += y

        simulated_positions[filename[:-4]][2] += 1

for filename in os.listdir(folder3):
    if filename.endswith(".csv"):
        with open(os.path.join(folder3, filename), "r") as file:
            reader = csv.reader(file)
            next(reader)  # Skip header row if present
            for row in reader:
                x = float(row[0])
                y = float(row[1])
                dead_positions[filename[:-4]][0] += x
                dead_positions[filename[:-4]][1] += y
        dead_positions[filename[:-4]][2] += 1

average_ground_truth_positions = {
    run: [total_x / count, total_y / count]
    for run, [total_x, total_y, count] in ground_truth_positions.items()
}
average_simulated_positions = {
    run: [total_x / count, total_y / count]
    for run, [total_x, total_y, count] in simulated_positions.items()
}
average_dead_positions = {
    run: [total_x / count, total_y / count]
    for run, [total_x, total_y, count] in dead_positions.items()
}
print(average_ground_truth_positions)  """


""" gt_time_diff_sum = pd.Timedelta(0)
particles_time_diff_sum = pd.Timedelta(0)
num_files = 0

# Process the ground truth CSV files
for filename in os.listdir(folder1):
    if filename.endswith(".csv"):
        with open(os.path.join(folder1, filename), "r") as file:
            reader = csv.reader(file)
            num_rows = sum(1 for _ in reader)  # Count the number of rows in the file

            file.seek(0)
            # Load the data from the current ground truth file into a dataframe
            ground_truth = pd.read_csv(file)
            ground_truth['Time'] = pd.to_datetime(ground_truth['Time'], errors='coerce')
            ground_truth.dropna(subset=['Time'], inplace=True)
            
            # Calculate gt_time_diff
            gt_time_diff = ground_truth['Time'].diff().mean()
            gt_time_diff_sum += gt_time_diff
            num_files += 1

# Process the particles CSV files
for filename in os.listdir(folder2):
    if filename.endswith(".csv"):
        with open(os.path.join(folder2, filename), "r") as file:
            reader = csv.reader(file)
            num_rows = sum(1 for _ in reader)  # Count the number of rows in the file

            file.seek(0)
            # Load the data from the current particles file into a dataframe
            particles = pd.read_csv(file)
            particles['Time'] = pd.to_datetime(particles['Time'], errors='coerce')
            particles.dropna(subset=['Time'], inplace=True)
            particles.set_index('Time', inplace=True)
            
            # Calculate particles_time_diff
            particles_time_diff = particles.index.to_series().diff().mean()
            particles_time_diff_sum += particles_time_diff

# Calculate the average of gt_time_diff and particles_time_diff
gt_time_diff_avg = gt_time_diff_sum / num_files
particles_time_diff_avg = particles_time_diff_sum / num_files

# Calculate the interpolation factor
if particles_time_diff_avg.total_seconds() != 0:
    interpolation_factor = gt_time_diff_avg.total_seconds() / particles_time_diff_avg.total_seconds()
else:
    interpolation_factor = 0  # Handle the zero division case

# Perform interpolation using the interpolation_factor
if particles_time_diff_avg.total_seconds() != 0:
    interpolated_particles = particles.resample(f'{particles_time_diff_avg.total_seconds():.6f}S').mean().interpolate(method='linear')
else:
    interpolated_particles = pd.DataFrame()  # Empty DataFrame when particles_time_diff_avg is zero

print(interpolated_particles) """





ground_truth_common_positions = []
simulated_common_positions = []
dead_common_positions = []
num_files = 0
data1 = []
file_paths = [os.path.join(folder1, filename) for filename in os.listdir(folder1) if filename.endswith(".csv")]
data_frames = []
interpolated_values=[]
max_row_length = 0
data_frames_particle = []

# Process the ground truth CSV files again, considering only the common rows
""" for file_path in file_paths:
    with open(file_path, "r") as file:
        reader = csv.reader(file)
        next(reader)  # Skip header row if present
        df = pd.DataFrame(reader, columns=["x", "y", "Time", "theta"])  # Adjust column names if needed
        # Convert relevant columns to numeric types
        df["x"] = pd.to_numeric(df["x"])
        df["y"] = pd.to_numeric(df["y"])
        df["Time"] = pd.to_numeric(df["Time"])
        df["theta"] = pd.to_numeric(df["theta"])
        
        data_frames.append(df)
        row_length = len(df)
        if row_length > max_row_length:
            max_row_length = row_length

# Step 2: Determine the desired size
desired_size = max_row_length

# Step 4: Interpolate the data frames
for df in data_frames:
    df_interpolated = df.interpolate(method='linear', limit_direction='both', limit_area='inside', limit=desired_size)

    # Step 5: Store the interpolated data
    #file_name = os.path.basename(file_path)  # Extract the file name from the time column or adjust as per your data
    #f_interpolated.to_csv('~/interpolated_' + file_name)
    interpolated_values.append(df_interpolated)

# If you want to combine the interpolated data into a single data frame:
combined_df = pd.concat(interpolated_values, axis=1)
combined_df.to_csv('~/combined_AMCL_interpolated_data.csv')
 """
max_row_length = 0
# Process the simulated CSV files again, considering only the common rows and skipping rows as needed
# Step 1: Read and load the CSV files
data_frames = []

for filename in os.listdir(folder2):
    if filename.endswith(".csv"):
        df = pd.read_csv(os.path.join(folder2, filename))
        data_frames.append(df)

# Step 2: Ensure both CSV files have the same length
desired_length = max(len(df) for df in data_frames)
for i in range(len(data_frames)):
    df = data_frames[i]
    if len(df) < desired_length:
        df = df.interpolate(method='linear', limit_direction='both', limit_area='inside', limit=desired_length)
    data_frames[i] = df[:desired_length]

# Step 3: Sort the data by time
for i in range(len(data_frames)):
    data_frames[i] = data_frames[i].sort_values('Time')

# Step 4: Interpolate missing data
interpolated_values = []
for i in range(len(data_frames[0])):
    row = {}
    for column in ['x', 'y', 'theta', 'Time']:
        value_sum = 0
        count = 0
        for df in data_frames:
            if column in df.columns:
                value = df[column].iloc[i]
                if not pd.isna(value):
                    value_sum += value
                    count += 1
        if count > 0:
            row[column] = value_sum / count
    interpolated_values.append(row)

# Step 5: Save the interpolated CSV file
interpolated_df = pd.DataFrame(interpolated_values)
interpolated_df.to_csv('~/interpolated_combined.csv', index=False)
                
for filename in os.listdir(folder3):
    if filename.endswith(".csv"):
        with open(os.path.join(folder3, filename), "r") as file:
            reader = csv.reader(file)
            next(reader)  # Skip header row if present
            for i, row in enumerate(reader):
                x = row[0]
                y = row[1] 
                dead_common_positions[i][0] += x
                dead_common_positions[i][1] += y


# Separate x and y positions into NumPy arrays
ground_truth_pos_x_np = np.array([pos[0] for pos in ground_truth_common_positions])
ground_truth_pos_y_np = np.array([pos[1] for pos in ground_truth_common_positions])

simulated_pos_x_np = np.array([pos[0] for pos in simulated_common_positions])
simulated_pos_y_np = np.array([pos[1] for pos in simulated_common_positions])

dead_pos_x_np = np.array([pos[0] for pos in dead_common_positions])
dead_pos_y_np = np.array([pos[1] for pos in dead_common_positions])

ground_truth_avg_pos_x_np = ground_truth_pos_x_np / num_files
ground_truth_avg_pos_y_np = ground_truth_pos_y_np / num_files

simulated_avg_pos_x_np = simulated_pos_x_np / num_files
simulated_avg_pos_y_np = simulated_pos_y_np / num_files 

dead_avg_pos_x_np = dead_pos_x_np / num_files
dead_avg_pos_y_np = dead_pos_y_np / num_files





""" rmse_avg = np.sqrt((ground_truth_avg_pos_x_np - simulated_avg_pos_x_np[:len(ground_truth_avg_pos_x_np)]) ** 2 +
                   (ground_truth_avg_pos_y_np - simulated_avg_pos_y_np[:len(ground_truth_avg_pos_y_np)]) ** 2)

rmse_avg_result =  np.empty(len(simulated_avg_pos_x_np))
rmse_avg_result[:len(ground_truth_avg_pos_x_np)] = rmse_avg
rmse_avg_result[len(ground_truth_avg_pos_x_np):] = rmse_avg[-1]

# Read CSV files from folder1 and folder2 simultaneously
for filename1, filename2 in zip(sorted(os.listdir(folder1)), sorted(os.listdir(folder2))):
    if filename1.endswith('.csv') and filename2.endswith('.csv'):
        file_path1 = os.path.join(folder1, filename1)
        file_path2 = os.path.join(folder2, filename2)

        last_row1 = read_last_row(file_path1)
        last_row2 = read_last_row(file_path2)
        if last_row1 and last_row2:
            x1 = last_row1[0]
            y1 = last_row1[1]
            x2 = last_row2[0]
            y2 = last_row2[1]
            diff.append(math.sqrt(calculate_RSMD(x1,x2,y1,y2)))
            distance.append(calculate_RSMD(x1,x2,y1,y2))
# Plotting 
avg_distance = np.sum(distance) / len(distance)
diff_np = np.array(diff)
result = (diff_np / len(diff_np)) * 100
average_value = np.mean(result)
file_indices = np.arange(1,len(diff_np)+1) """

""" plt.bar(file_indices, result)
plt.axhline(y=average_value, color='r', linestyle='--', label='Average')
# Set the y-axis limits to include space for the text annotation
plt.ylim(0, np.max(result) * 1.1)  # Adjust the multiplication factor as needed

# Add text annotation for the average value outside of the graph
plt.text(len(diff_np) - 1, np.max(result) * 1.05, f'Average: {average_value:.2f}%', ha='right', va='bottom')

plt.xlabel('Number of Runs')
plt.ylabel('Error [%]')
plt.title('quero morrer')
plt.show() """

#print("length",len(rmse_avg))
""" amcl_time_array=[]
particle_time_array=[]
for filename in os.listdir(folder1):
    if filename.endswith(".csv"):
        with open(os.path.join(folder1, filename), "r") as file:
            reader = csv.reader(file)
            next(reader)
            for row in reader:
                time = float(row[2])
                amcl_time_array.append(time)
for filename in os.listdir(folder2):
    if filename.endswith(".csv"):
        with open(os.path.join(folder2, filename), "r") as file:
            reader = csv.reader(file)
            next(reader)
            for row in reader:
                time = float(row[2])
                particle_time_array.append(time)

amcl_time_array = np.array(amcl_time_array)
particle_time_array = np.array(particle_time_array)

amcl_time_array -= amcl_time_array[0]
particle_time_array -= particle_time_array[0] """


# TESTE
#amcl_df = pd.read_csv('~/amcl_csv/amcl.csv')
#particle_df = pd.read_csv('~/particle_csv/particle.csv')
""" 
amcl_time = amcl_df['Time']
amcl_orient = amcl_df[['x', 'y', 'w', 'z']]
amcl_w_z = amcl_df[['w','z']]
particle_time = particle_df['Time']
particle_orient = particle_df[['x:', 'y', 'w', 'z']]
particle_w_z = particle_df[['w','z']] """

# --------------------
""" 
# Assuming you have two DataFrames: amcl_df and particle_df
amcl_time = amcl_df['Time']
amcl_w_z = amcl_df[['w', 'z']]
particle_time = particle_df['Time']
particle_w_z = particle_df[['w', 'z']]

# Perform circular interpolation for 'w' and 'z' separately
amcl_interp_w = interp1d(amcl_time, amcl_w_z['w'], kind='linear', fill_value='extrapolate')
amcl_interp_z = interp1d(amcl_time, amcl_w_z['z'], kind='linear', fill_value='extrapolate')
particle_interp_w = interp1d(particle_time, particle_w_z['w'], kind='linear', fill_value='extrapolate')
particle_interp_z = interp1d(particle_time, particle_w_z['z'], kind='linear', fill_value='extrapolate')

# Generate the desired time points for interpolation
common_time = np.arange(
    max(amcl_time.min(), particle_time.min()),
    min(amcl_time.max(), particle_time.max()),
    0.1  # Use a smaller time step, e.g., 0.1, for smoother interpolation
)
# Perform the interpolation
amcl_interp_w_values = amcl_interp_w(common_time)
amcl_interp_z_values = amcl_interp_z(common_time)
particle_interp_w_values = particle_interp_w(common_time)
particle_interp_z_values = particle_interp_z(common_time)

# Calculate the circular difference of angles
def circular_diff(a, b):
    diff = np.arctan2(np.sin(a - b), np.cos(a - b))
    return diff

# Calculate the circular difference for 'w' and 'z' angles
diff_w = circular_diff(amcl_interp_w_values, particle_interp_w_values)
diff_z = circular_diff(amcl_interp_z_values, particle_interp_z_values)
# Calculate the root mean square (RMSE) of the circular differences
rmse_angles = np.sqrt((np.square(diff_w)) + (np.square(diff_z)))
print(rmse_angles) """
# -------------------


amcl_interp_func = interp1d(amcl_df['Time'], amcl_df[['x', 'y','w','z']], axis=0, fill_value='extrapolate')
particle_interp_func = interp1d(particle_df['Time'], particle_df[['x:', 'y','w','z']], axis=0, fill_value='extrapolate')

common_time = np.arange(max(amcl_df['Time'].min(), particle_df['Time'].min()),
                        min(amcl_df['Time'].max(), particle_df['Time'].max()),
                        0.71)  # Use a smaller time step, e.g., 0.1, for smoother interpolation

amcl_interp_values = amcl_interp_func(common_time)
particle_interp_values = particle_interp_func(common_time)

amcl_interp_values = np.array(amcl_interp_values)
particle_interp_values = np.array(particle_interp_values)

amcl_interp_values_x =  amcl_interp_values[:,0]
amcl_interp_values_y =  amcl_interp_values[:,1]
amcl_interp_values_w =  amcl_interp_values[:,2]
amcl_interp_values_z =  amcl_interp_values[:,3]
particle_interp_values_x = particle_interp_values[:,0]
particle_interp_values_y = particle_interp_values[:,1]
particle_interp_values_w =  particle_interp_values[:,2]
particle_interp_values_z =  particle_interp_values[:,3]

amcl_theta = [tf.euler_from_quaternion([0, 0, amcl_interp_values_z[i], amcl_interp_values_w[i]])[2] for i in range(len(amcl_interp_values_w))]
amcl_theta = np.array(amcl_theta)
particle_theta = [tf.euler_from_quaternion([0, 0, particle_interp_values_z[i], particle_interp_values_w[i]])[2] for i in range(len(amcl_interp_values_w))]
particle_theta = np.array(particle_theta)

def circular_diff(a, b):
    diff = np.arctan2(np.sin(a - b), np.cos(a - b))
    return diff

def angular_rmse(predicted_angles, target_angles):
    # Compute angular differences
    angular_diff = np.unwrap(predicted_angles - target_angles)
    angular_diff = (angular_diff + np.pi) % (2 * np.pi) - np.pi
  

    # Square the angular differences
    squared_diff = np.square(angular_diff)

    # Calculate angular RMSE
    angular_rmse = np.sqrt(squared_diff)

    return angular_rmse


# Calculate the circular difference for 'w' and 'z' angles
diff_w = angular_rmse(amcl_theta, particle_theta)


diff_w_1 = circular_diff(amcl_interp_values_w, particle_interp_values_w)
diff_z_1 = circular_diff(amcl_interp_values_z, particle_interp_values_z)
# Calculate the root mean square (RMSE) of the circular differences
rmse_angles1 = np.sqrt((np.square(diff_w_1)) + (np.square(diff_z_1)))

rmse_angles = np.sqrt((amcl_theta-particle_theta)**2)
rmse_angles = [2*np.pi - angle if angle > np.pi else angle for angle in rmse_angles]
rmse_angles = np.array(rmse_angles) 
data = pd.DataFrame({'amcl_theta': amcl_theta,'particle_theta':particle_theta,'rmse_angles':rmse_angles,'oi?':rmse_angles1,'morte':diff_w})

#data.to_csv(f'~/amcl_csv/angles.csv', index=False)




rmse_avg = np.sqrt((amcl_interp_values_x - particle_interp_values_x) ** 2 +
                   (amcl_interp_values_y - particle_interp_values_y) ** 2)

common_time -= common_time[0]
plt.figure(figsize=(10, 6))
plt.plot(common_time,rmse_avg)
plt.xlabel('Time [s]')
plt.ylabel('RMSE [m]')
plt.xlim(0,common_time[len(common_time)-1])
plt.grid(True)
plt.show() 

plt.figure(figsize=(8, 4))
plt.plot(common_time, diff_w)
plt.xlabel('Time [s]')
plt.ylabel('RMSE [rad]')
plt.xlim(0,common_time[len(common_time)-1])
plt.grid(True)

plt.show()
""" 
total_time=max(amcl_time_array)
time_interval = total_time / len(rmse_avg)
times = np.linspace(0, total_time, len(rmse_avg))
# Create the x-axis values
x_values = range(1, 80)

# Plot the RMSE values
plt.plot(times, rmse_avg)

# Set labels and title
plt.xlabel('Total Time of Simulation (seconds)')
plt.ylabel('RMSE')
plt.title('RMSE vs. Total Time of Simulation')
total_time = 83
# Set x-axis ticks
#plt.xticks(x_values, [i * total_time / len(x_values) for i in x_values])

# Display the plot
plt.show() """