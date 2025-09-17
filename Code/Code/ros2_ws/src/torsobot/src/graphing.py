import matplotlib.pyplot as plt
import pandas as pd
import math
import numpy as np

# import json

# for reading yaml parameter files
import yaml


def rad2deg(rad_value):
    return rad_value * 180/math.pi


def cycles2rad(cycles):
    return cycles*2*np.pi


# directory path for the data_logs
dir_path = "/home/pi/torsobot/Code/Code/ros2_ws/data_logs"

# Get user input for filename
file_date = input("Enter date:  ")
file_time = input("Enter time:  ")

csv_file_name = "csv_" + \
    file_date + "_" + file_time + ".csv"
csv_file_path = dir_path + "/" + csv_file_name

# json_file_name = "torsobot_data_metadata_" + \
#     file_date + "_" + file_time + ".json"
# json_file_path = dir_path + "/" + json_file_name

# yaml path file
yaml_file_name = "metadata_" + \
    file_date + "_" + file_time + ".yaml"
yaml_file_path = dir_path + "/" + yaml_file_name

# for saving the figure as png
save_file_name = csv_file_name.split(".")[0] + ".png"
save_file_path = dir_path + "/graphs/" + save_file_name

# read yaml
with open(yaml_file_path, 'r') as yaml_file:
    metadata = yaml.safe_load(yaml_file)

desired_torso_pitch = metadata["/**"]["ros__parameters"]["desired_torso_pitch"]
wheel_max_torque = metadata["/**"]["ros__parameters"]["wheel_max_torque"]
control_max_integral = metadata["/**"]["ros__parameters"]["control_max_integral"]
kp = metadata["/**"]["ros__parameters"]["kp"]
ki = metadata["/**"]["ros__parameters"]["ki"]
kd = metadata["/**"]["ros__parameters"]["kd"]


# plot text
plot_text = "desired_torso_pitch:" + str(desired_torso_pitch) + "\nkp: " + str(kp) + ", ki: " + str(ki) + ", kd: " + \
    str(kd) + "\wheel_max_torque: " + str(wheel_max_torque) + \
    "\ncontrol_max_integral: " + str(control_max_integral)

# read csv
df = pd.read_csv(csv_file_path)

# --------------------------- Data----------------------------
# get data from the csv
# "timestamp,torso_pitch,torso_pitch_rate,wheel_pos,wheel_vel,wheel_torque,wheel_cmd_torque,mot_drv_mode";
timestamp = df["timestamp"]
torso_pitch = df["torso_pitch"]
torso_pitch_rate = df["torso_pitch_rate"]
wheel_pos = df["wheel_pos"]
wheel_vel = df["wheel_vel"]
wheel_torque = df["wheel_torque"]
wheel_cmd_torque = df["wheel_cmd_torque"]
mot_drv_mode = df["mot_drv_mode"]

# change data
timestamp = timestamp/(10**6)  # ns to ms

zero_time = timestamp[0]
timestamp = timestamp - zero_time

start_time = int(input("Start time in ms: "))
# get values where time is more than start_time
start_time = timestamp > start_time
start_idx = df[start_time].index[0]

# get values from start_idx to end
timestamp = timestamp[start_idx:-1]
torso_pitch = rad2deg(torso_pitch[start_idx:-1])
wheel_pos = wheel_pos[start_idx:-1]
wheel_vel = wheel_vel[start_idx:-1]
wheel_cmd_torque = wheel_cmd_torque[start_idx:-1]
wheel_torque = wheel_torque[start_idx:-1]

desired_torso_pitch = np.ones(len(timestamp)) * \
    desired_torso_pitch  # df["desired_pitch_data"]
# print(desired_torso_pitch)

# --------------------------- Plotting----------------------------

# plotting limits
max_trq_lim = 1.1 * max(max(wheel_torque), max(wheel_cmd_torque))
min_trq_lim = 1.1 * min(min(wheel_torque), min(wheel_cmd_torque))

fig, axs = plt.subplots(4, 1, figsize=(15, 8))
fig.suptitle(file_date + "---" + file_time)

# Subplot 1
axs[0].plot(timestamp, desired_torso_pitch,
            color="red", label="desired torso pitch")
axs[0].plot(timestamp, torso_pitch, label="actual torso pitch")

# draw horizontal line for pitch
# plt.axhline(desired_torso_pitch)

axs[0].minorticks_on()
axs[0].grid(which="both")

axs[0].set_ylabel("torso pitch (deg)")
# axs[0].set_ylim([-10, 380])
# axs[0].set_xlim(xmin=0)
axs[0].legend()
axs[0].set_title(plot_text, fontsize=10)
# axs[0].text(2800, 110, plot_text, ha='right',
#             bbox=dict(facecolor='yellow', alpha=0.75))

# Subplot 2
axs[1].plot(timestamp, wheel_torque, label="wheel_torque")
axs[1].plot(timestamp, wheel_cmd_torque,
            color="red", label="wheel_cmd_torque")
axs[1].minorticks_on()
axs[1].grid(which="both")
# axs[1].set_xlabel("time (ms)")
axs[1].set_ylabel("torque (N.m)")
axs[1].set_ylim([min_trq_lim, max_trq_lim])
axs[1].legend()

# Subplot 3
axs[2].plot(timestamp, wheel_pos, label="wheel_pos")
axs[2].minorticks_on()
axs[2].grid(which="both")
axs[2].axhline(np.pi/10, color="red")
axs[2].axhline(-np.pi/10, color="red")
# axs[2].set_xlabel("time (ms)")
axs[2].set_ylabel("wheel pos (rad)")
# axs[2].set_ylim([min_trq_lim, max_trq_lim])
axs[2].legend()

# Subplot 4
axs[3].plot(timestamp, wheel_vel, label="wheel_vel")
axs[3].minorticks_on()
axs[3].grid(which="both")
axs[3].set_xlabel("time (ms)")
axs[3].set_ylabel("wheel velocity (rad/s)")
# axs[3].set_ylim([min_trq_lim, max_trq_lim])
axs[3].legend()

plt.show()
plt.savefig(save_file_path, dpi=300)
