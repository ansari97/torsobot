import matplotlib.pyplot as plt
import pandas as pd
import math
import numpy as np


def rad2deg(rad_value):
    return rad_value * 180/math.pi


dir_path = "/home/pi/torsobot/Code/Code/ros2_ws/data_logs"
csv_file_date = "2025-08-01"
csv_file_time = "12-04-33"
csv_file_name = "torsobot_data_csv_" + \
    csv_file_date + "_" + csv_file_time + ".csv"
csv_file_path = dir_path + "/" + csv_file_name

save_file_name = csv_file_name.split(".")[0] + ".png"
save_file_path = dir_path + "/" + save_file_name

df = pd.read_csv(csv_file_path)

# get data
time_value = df["timestamp"]
imu_pitch_data = df["IMU_pitch"]

desired_pitch_value = np.ones(len(time_value))*10  # df["desired_pitch_data"]

# change data
time_value = time_value/(10**6)  # ns to ms
imu_pitch_data = rad2deg(imu_pitch_data)

plt.plot(time_value, imu_pitch_data, label="imu_pitch")
plt.plot(time_value, desired_pitch_value, label="desired pitch")
plt.xlabel("time (ms)")
plt.ylabel("IMU pitch (deg)")
plt.ylim([-180, 180])
plt.legend()


plt.show()
plt.savefig(save_file_path)
