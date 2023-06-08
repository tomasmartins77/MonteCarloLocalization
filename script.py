#!/usr/bin/env python3

import os
import csv
import matplotlib.pyplot as plt
import math
import numpy as np
from collections import defaultdict
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
def extract_values(row):
    values = row[0].split('\n')
    x = float(values[0].split(':')[1])
    y = float(values[1].split(':')[1])
    return x, y

# Path to the first folder containing CSV files
folder1 = os.path.expanduser('~/amcl_csv')

# Path to the second folder containing CSV files
folder2 = os.path.expanduser('~/particle_csv')

# Create dictionaries to store the cumulative positions and count for each position
ground_truth_positions = defaultdict(lambda: [0, 0, 0])  # [sum_x, sum_y, count]
simulated_positions = defaultdict(lambda: [0, 0, 0])     # [sum_x, sum_y, count]

# Variable to store the differences in x values
x_differences = []

# Variable to store the differences in y values
y_differences = []

diff = []
distance = []
def calculate_RSMD(x_truth, x_sim, y_truth, y_sim):
    diff_squared_i = (x_truth - x_sim)**2 + (y_truth - y_sim)**2
    return diff_squared_i

def calculate_error_percentage(x_truth, x_sim,y_truth,y_sim):
    abs_diff_x = abs(x_truth - x_sim)
    abs_diff_y = abs(y_truth - y_sim)
    percentage_error = (avg_distance / magnitude_ground_truth) * 100

""" # Process the ground truth CSV files
for filename in os.listdir(folder1):
    if filename.endswith(".csv"):
        with open(os.path.join(folder1, filename), "r") as file:
            reader = csv.reader(file)
            next(reader)  # Skip header row if present
            for row in reader:
                x, y = extract_values(row)
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
                x, y = extract_values(row)
                simulated_positions[filename[:-4]][0] += x
                simulated_positions[filename[:-4]][1] += y
                simulated_positions[filename[:-4]][2] += 1

average_ground_truth_positions = {
    run: [total_x / count, total_y / count]
    for run, [total_x, total_y, count] in ground_truth_positions.items()
}
average_simulated_positions = {
    run: [total_x / count, total_y / count]
    for run, [total_x, total_y, count] in simulated_positions.items()
}
print(average_ground_truth_positions) """

# Find the number of common rows
num_common_rows = float('inf')  # Initialize with infinity

# Process the ground truth CSV files
for filename in os.listdir(folder1):
    if filename.endswith(".csv"):
        with open(os.path.join(folder1, filename), "r") as file:
            reader = csv.reader(file)
            num_rows = sum(1 for _ in reader)  # Count the number of rows in the file
            num_common_rows = min(num_common_rows, num_rows)

# Skip interval for the simulation data
simulation_skip_interval = 10  # Adjust as needed

# Initialize lists to store the cumulative positions for common rows
ground_truth_common_positions = [[0, 0] for _ in range(num_common_rows)]
simulated_common_positions = [[0, 0] for _ in range(num_common_rows)]

# Process the ground truth CSV files again, considering only the common rows
for filename in os.listdir(folder1):
    if filename.endswith(".csv"):
        with open(os.path.join(folder1, filename), "r") as file:
            reader = csv.reader(file)
            next(reader)  # Skip header row if present
            for i, row in enumerate(reader):
                if i >= num_common_rows:  # Skip extra rows
                    break
                x, y = extract_values(row)
                ground_truth_common_positions[i][0] += x
                ground_truth_common_positions[i][1] += y

# Process the simulated CSV files again, considering only the common rows and skipping rows as needed
for filename in os.listdir(folder2):
    if filename.endswith(".csv"):
        with open(os.path.join(folder2, filename), "r") as file:
            reader = csv.reader(file)
            next(reader)  # Skip header row if present
            skip_counter = 0  # Counter for skipping rows
            for i, row in enumerate(reader):
                if i >= num_common_rows:  # Skip extra rows
                    break
                if skip_counter == 0:  # Process the row
                    x, y = extract_values(row)
                    simulated_common_positions[i][0] += x
                    simulated_common_positions[i][1] += y
                skip_counter = (skip_counter + 1) % simulation_skip_interval  # Skip rows based on the interval

# Separate x and y positions into NumPy arrays
ground_truth_avg_pos_x_np = np.array([pos[0] for pos in ground_truth_common_positions])
ground_truth_avg_pos_y_np = np.array([pos[1] for pos in ground_truth_common_positions])

simulated_avg_pos_x_np = np.array([pos[0] for pos in simulated_common_positions])
simulated_avg_pos_y_np = np.array([pos[1] for pos in simulated_common_positions])


rmse_avg = np.sqrt((ground_truth_avg_pos_x_np - simulated_avg_pos_x_np[:len(ground_truth_avg_pos_x_np)]) ** 2 +
                   (ground_truth_avg_pos_y_np - simulated_avg_pos_y_np[:len(ground_truth_avg_pos_y_np)]) ** 2)

rmse_avg_result =  np.empty(len(simulated_avg_pos_x_np))
rmse_avg_result[:len(ground_truth_avg_pos_x_np)] = rmse_avg
rmse_avg_result[len(ground_truth_avg_pos_x_np):] = rmse_avg[-1]
print("TAMANHO ", len(rmse_avg))

# Read CSV files from folder1 and folder2 simultaneously
for filename1, filename2 in zip(sorted(os.listdir(folder1)), sorted(os.listdir(folder2))):
    if filename1.endswith('.csv') and filename2.endswith('.csv'):
        file_path1 = os.path.join(folder1, filename1)
        file_path2 = os.path.join(folder2, filename2)

        last_row1 = read_last_row(file_path1)
        last_row2 = read_last_row(file_path2)
        if last_row1 and last_row2:
            x1, y1 = extract_values(last_row1)
            x2, y2 = extract_values(last_row2)
            diff.append(math.sqrt(calculate_RSMD(x1,x2,y1,y2)))
            distance.append(calculate_RSMD(x1,x2,y1,y2))
# Plotting 
avg_distance = np.sum(distance) / len(distance)
diff_np = np.array(diff)
result = (diff_np / len(diff_np)) * 100
average_value = np.mean(result)
file_indices = np.arange(1,len(diff_np)+1)

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

# Create the x-axis values
x_values = range(1, len(rmse_avg) + 1)

# Plot the RMSE values
plt.plot(x_values, rmse_avg, marker='o')

# Set labels and title
plt.ylim(0,5)
plt.xlabel('Total Time of Simulation (seconds)')
plt.ylabel('RMSE')
plt.title('RMSE vs. Total Time of Simulation')
total_time = 83
# Set x-axis ticks
#plt.xticks(x_values, [i * total_time / len(x_values) for i in x_values])

# Display the plot
plt.show()