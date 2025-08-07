import matplotlib.pyplot as plt
import pandas as pd
import math
import numpy as np

# import json

# for reading yaml parameter files
import yaml


def rad2deg(rad_value):
    return rad_value * 180/math.pi


dir_path = "/home/pi/torsobot/Code/Code/ros2_ws/data_logs"

# Get user input for filename
file_date = input("Enter date:  ")
file_time = input("Enter time:  ")

csv_file_name = "torsobot_data_csv_" + \
    file_date + "_" + file_time + ".csv"
csv_file_path = dir_path + "/" + csv_file_name

# json_file_name = "torsobot_data_metadata_" + \
#     file_date + "_" + file_time + ".json"
# json_file_path = dir_path + "/" + json_file_name

# yaml path file
yaml_file_name = "torsobot_data_metadata_" + \
    file_date + "_" + file_time + ".yaml"
yaml_file_path = dir_path + "/" + yaml_file_name

# for saving the figure as png
save_file_name = csv_file_name.split(".")[0] + ".png"
save_file_path = dir_path + "/" + save_file_name

# read yaml
with open(yaml_file_path, 'r') as yaml_file:
    metadata = yaml.safe_load(yaml_file)

desired_torso_pitch = metadata["/**"]["ros__parameters"]["desired_torso_pitch"]
mot_max_torque = metadata["/**"]["ros__parameters"]["motor_max_torque"]
control_max_integral = metadata["/**"]["ros__parameters"]["control_max_integral"]
kp = metadata["/**"]["ros__parameters"]["kp"]
ki = metadata["/**"]["ros__parameters"]["ki"]
kd = metadata["/**"]["ros__parameters"]["kd"]

# print(desired_torso_pitch)

# # convert to radians
# desired_torso_pitch = rad2deg(desired_torso_pitch)
# print(desired_torso_pitch)

# plot text
plot_text = "desired_torso_pitch:" + str(desired_torso_pitch) + "\nkp: " + str(kp) + ", ki: " + str(ki) + ", kd: " + \
    str(kd) + "\nmax_motor_torque: " + str(mot_max_torque) + \
    "\ncontrol_max_integral: " + str(control_max_integral)

# read csv
df = pd.read_csv(csv_file_path)

# get data from the csv
time_value = df["timestamp"]
imu_pitch_data = df["IMU_pitch"]
motor_torque = df["motor_trq"]
motor_cmd_torque = df["motor_cmd_trq"]

# for the plotting
max_trq_lim = 1.1 * max(max(motor_torque), max(motor_cmd_torque))
min_trq_lim = 1.1 * min(min(motor_torque), min(motor_cmd_torque))

zero_time = time_value[0]
time_value = time_value - zero_time

desired_torso_pitch = np.ones(len(time_value)) * \
    desired_torso_pitch  # df["desired_pitch_data"]
# print(desired_torso_pitch)

# change data
time_value = time_value/(10**6)  # ns to ms
imu_pitch_data = rad2deg(imu_pitch_data)

fig, axs = plt.subplots(2, 1, figsize=(15, 8))
fig.suptitle(file_date + "---" + file_time)

# Subplot 1
axs[0].plot(time_value, desired_torso_pitch,
            color="red", label="desired pitch")
axs[0].plot(time_value, imu_pitch_data, label="imu_pitch")

# draw horizontal line for pitch
# plt.axhline(desired_torso_pitch)

axs[0].minorticks_on()
axs[0].grid(which="both")

axs[0].set_ylabel("IMU pitch (deg)")
axs[0].set_ylim([-10, 380])
axs[0].set_xlim(xmin=0)
axs[0].legend()
axs[0].set_title(plot_text, fontsize=10)
# axs[0].text(2800, 110, plot_text, ha='right',
#             bbox=dict(facecolor='yellow', alpha=0.75))

# Subplot 2
axs[1].plot(time_value, motor_torque, label="motor_torque")
axs[1].plot(time_value, motor_cmd_torque,
            color="red", label="motor_cmd_torque")
axs[1].minorticks_on()
axs[1].grid(which="both")
axs[1].set_xlabel("time (ms)")
axs[1].set_ylabel("torque (N.m)")
axs[1].set_ylim([min_trq_lim, max_trq_lim])
axs[1].legend()

plt.show()
plt.savefig(save_file_path, dpi=300)
