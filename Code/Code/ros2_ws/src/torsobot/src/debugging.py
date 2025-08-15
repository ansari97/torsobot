import pandas as pd
import matplotlib.pyplot as plt
import math

import numpy as np

gear_ratio = (38 / 16) * (48 / 16)
num_spokes = 10

wheel_pos_init = -math.pi/num_spokes

directory_path = "/home/pi/torsobot/Code/Code/ros2_ws/data_logs/"
date_time = "2025-08-15_16-31-59"
csv_file_name = "torsobot_data_csv_" + date_time + ".csv"
save_file = "debugging_" + date_time + ".png"

df = pd.read_csv(directory_path + csv_file_name)

timestamp = df["timestamp"]
torso_pitch = df["torso_pitch"]
mot_pos = df["mot_pos"]
mot_vel = df["mot_vel"]

mot_pos_init = 353.156
wheel_vel_wrt_torso = (mot_pos - mot_pos_init)/gear_ratio

wheel_vel_wrt_torso = np.fmod(wheel_vel_wrt_torso, 2 * math.pi / num_spokes)

torso_pitch_init = 3.12944
torso_pitch_check = torso_pitch - torso_pitch_init

wheel_pos = wheel_vel_wrt_torso - torso_pitch_check + wheel_pos_init

#   // wrap around (-PI/n, PI/n]
wheel_pos = np.fmod(wheel_pos, 2 * math.pi / num_spokes)

# if (wheel_pos <= -math.pi / num_spokes):
#     wheel_pos += 2 * math.pi / num_spokes
# elif (wheel_pos > math.pi / num_spokes):
#     wheel_pos -= 2 * math.pi / num_spokes

wheel_pos = np.where(
    wheel_pos <= -math.pi / num_spokes,
    wheel_pos + 2 * math.pi / num_spokes,
    np.where(
        wheel_pos > math.pi / num_spokes,
        wheel_pos - 2 * math.pi / num_spokes,
        wheel_pos
    )
)


timestamp = timestamp - timestamp[0]
timestamp = timestamp/(1000*1000)
print(timestamp[0:5])

fig, axs = plt.subplots(4, 1, figsize=(15, 8))

axs[0].plot(timestamp, torso_pitch)
axs[0].minorticks_on()
axs[0].grid(which="both")

axs[1].plot(timestamp, mot_pos)
axs[1].minorticks_on()
axs[1].grid(which="both")

axs[2].plot(timestamp, mot_vel)
axs[2].minorticks_on()
axs[2].grid(which="both")

# axs[3].plot(timestamp, wheel_vel_wrt_torso, label="wheel_vel_wrt_torso")
# axs[3].plot(timestamp, torso_pitch_check, label="torso_pitch_check")
axs[3].plot(timestamp, wheel_pos, label="wheel_pos")
axs[3].legend()
axs[3].minorticks_on()
axs[3].grid(which="both")

plt.show()
plt.savefig(directory_path + "debugging/" + save_file, dpi=400)
