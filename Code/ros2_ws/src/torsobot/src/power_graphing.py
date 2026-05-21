import matplotlib.pyplot as plt
import pandas as pd
import math
import numpy as np
from pathlib import Path

from matplotlib.ticker import MultipleLocator

# import json

# for reading yaml parameter files
import yaml


def rad2deg(rad_value):
    return rad_value * 180 / math.pi


def cycles2rad(cycles):
    return cycles * 2 * np.pi


# directory path for the data_logs
script_path = Path(__file__).resolve()
# dir_path = "/home/pi/torsobot/Code/ros2_ws/data_logs"
dir_path = script_path.parents[3] / "data_logs"

# Get user input for filename
file_date = input("Enter date:  ")
file_time = input("Enter time:  ")

csv_file_name = file_date + "_" + file_time + ".csv"
csv_file_path = dir_path / "csvs" / csv_file_name

# json_file_name = "torsobot_data_metadata_" + \
#     file_date + "_" + file_time + ".json"
# json_file_path = dir_path + "/" + json_file_name

# yaml path file
yaml_file_name = file_date + "_" + file_time + ".yaml"
yaml_file_path = dir_path / "metadata" / yaml_file_name

# for saving the figure as png
save_file_name = csv_file_name.split(".")[0] + ".png"
save_file_path = dir_path / "graphs" / "power_plots" / save_file_name

# read yaml
with open(yaml_file_path, "r") as yaml_file:
    metadata = yaml.safe_load(yaml_file)

desired_torso_pitch_deg = metadata["/**"]["ros__parameters"]["desired_torso_pitch_deg"]
wheel_max_torque = metadata["/**"]["ros__parameters"]["wheel_max_torque"]
control_max_integral = metadata["/**"]["ros__parameters"]["control_max_integral"]
kp = metadata["/**"]["ros__parameters"]["kp"]
ki = metadata["/**"]["ros__parameters"]["ki"]
kd = metadata["/**"]["ros__parameters"]["kd"]
controller = metadata["/**"]["ros__parameters"]["controller"]


# plot text
plot_text = (
    "desired_torso_pitch_deg:"
    + str(desired_torso_pitch_deg)
    + "\ncontroller: "
    + str(controller)
    + "\nkp: "
    + str(kp)
    + ", ki: "
    + str(ki)
    + ", kd: "
    + str(kd)
    + "\ncontrol_max_integral: "
    + str(control_max_integral)
    + "\nwheel_max_torque: "
    + str(wheel_max_torque)
)

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
encoder_speed = df["encoder_speed"]

# change data
timestamp = timestamp / (10**6)  # ns to ms

zero_time = timestamp[0]
timestamp = timestamp - zero_time

start_time = int(input("Start time in ms: "))
# get values where time is more than start_time
start_time = timestamp > start_time
start_idx = df[start_time].index[0]

# get values from start_idx to end
timestamp = timestamp[start_idx:-1]
torso_pitch = rad2deg(torso_pitch[start_idx:-1])
torso_pitch_rate = torso_pitch_rate[start_idx:-1]
wheel_pos = wheel_pos[start_idx:-1]
wheel_vel = wheel_vel[start_idx:-1]
wheel_cmd_torque = wheel_cmd_torque[start_idx:-1]
wheel_torque = wheel_torque[start_idx:-1]
encoder_speed = encoder_speed[start_idx:-1]

desired_torso_pitch_deg = (
    np.ones(len(timestamp)) * desired_torso_pitch_deg
)  # df["desired_pitch_data"]
# print(desired_torso_pitch_deg)

# get power
power = wheel_torque * encoder_speed

# --------------------------- Plotting----------------------------
fig, axs = plt.subplots()
sc = axs.plot(timestamp, power)
axs.set_title(f"pitch: {desired_torso_pitch_deg[0]}")
axs.grid(which="both")

axs.minorticks_on()

plt.savefig(save_file_path, dpi=300)
plt.show()
