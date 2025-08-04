import matplotlib.pyplot as plt
import pandas as pd
import math
import numpy as np

import json


def rad2deg(rad_value):
    return rad_value * 180/math.pi


dir_path = "/home/pi/torsobot/Code/Code/ros2_ws/data_logs"
file_date = "2025-08-04"
file_time = "16-41-15"
csv_file_name = "torsobot_data_csv_" + \
    file_date + "_" + file_time + ".csv"
csv_file_path = dir_path + "/" + csv_file_name

json_file_name = "torsobot_data_metadata_" + \
    file_date + "_" + file_time + ".json"
json_file_path = dir_path + "/" + json_file_name

save_file_name = csv_file_name.split(".")[0] + ".png"
save_file_path = dir_path + "/" + save_file_name

# read json
with open(json_file_path, 'r') as json_file:
    metadata = json.load(json_file)
    desired_torso_pitch = metadata["desired_torso_pitch"]
    mot_max_torque = metadata["mot_max_torque"]
    kp = metadata["kp"]
    ki = metadata["ki"]
    kd = metadata["kd"]

# plot text
plot_text = "kp: " + str(kp) + "\nki: " + str(ki) + "\nkd: " + \
    str(kd) + "\nmax_motor_torque: " + str(mot_max_torque)

# read csv
df = pd.read_csv(csv_file_path)

# get data
time_value = df["timestamp"]
imu_pitch_data = df["IMU_pitch"]

zero_time = time_value[0]
time_value = time_value - zero_time

desired_torso_pitch = np.ones(len(time_value)) * \
    desired_torso_pitch  # df["desired_pitch_data"]

# change data
time_value = time_value/(10**6)  # ns to ms
imu_pitch_data = rad2deg(imu_pitch_data)

plt.plot(time_value, imu_pitch_data, label="imu_pitch")
plt.plot(time_value, desired_torso_pitch, label="desired pitch")

# draw horizontal line for pitch
# plt.axhline(desired_torso_pitch)

plt.minorticks_on()
plt.grid(which="both")

plt.xlabel("time (ms)")
plt.ylabel("IMU pitch (deg)")
plt.ylim([-180, 180])
plt.xlim(xmin=0)
plt.legend()
plt.title(file_date + "---" + file_time)
plt.text(2800, 110, plot_text, ha='right',
         bbox=dict(facecolor='yellow', alpha=0.75))

plt.show()
plt.savefig(save_file_path)
