import pandas as pd
import matplotlib.pyplot as plt
import math

import numpy as np

gear_ratio = (38 / 16) * (48 / 16)
num_spokes = 10

wheel_pos_init = -math.pi / num_spokes

# directory_path = "/home/pi/torsobot/Code/Code/ros2_ws/data_logs/"
directory_path = "/home/aansari/torsobot/Code/Code/ros2_ws/data_logs/"
date_time = "2025-11-03_11-25-42"
csv_file_name = "csv_" + date_time + ".csv"
save_file = "debugging_" + date_time + ".png"

df = pd.read_csv(directory_path + csv_file_name)

timestamp = df["timestamp"].values
torso_pitch_raw = df["torso_pitch"].values
torso_pitch_rate = df["torso_pitch_rate"].values
mot_pos_raw = df["mot_pos"].values
mot_vel = df["mot_vel"].values


# timestamp is in nanosecs
timestamp = timestamp - timestamp[0]
timestamp = timestamp / (1000 * 1000)  # millisecs
# print(timestamp[0:5])

mot_pos_init = -21.6492

mot_pos_now = mot_pos_raw - mot_pos_init

wheel_pos_wrt_torso = mot_pos_now / gear_ratio

#  Numerically differentiate data
# 1. torso_pitch rate from torso_pitch

# wheel_pos_wrt_torso = np.fmod(wheel_pos_wrt_torso, 2 * math.pi / num_spokes)

torso_pitch_init = 3.1553  # change for each run
torso_pitch_check = torso_pitch_raw - torso_pitch_init

# initialize wheel_pos at initial standing position of the wheel at two spokes
wheel_pos_raw = wheel_pos_wrt_torso + torso_pitch_check + wheel_pos_init

# print(wheel_pos.shape)

wheel_vel_calc = np.zeros_like(wheel_pos_raw)

# print(wheel_vel_calc.shape)

# print(wheel_pos_raw[2:])

wheel_vel_calc[1:-1] = (
    (wheel_pos_raw[2:] - wheel_pos_raw[:-2]) / (timestamp[2:] - timestamp[:-2]) * 1000
)

wheel_vel_calc[0] = (
    (wheel_pos_raw[1] - wheel_pos_raw[0]) / (timestamp[1] - timestamp[0]) * 1000
)
wheel_vel_calc[-1] = (
    (wheel_pos_raw[-1] - wheel_pos_raw[-2]) / (timestamp[-1] - timestamp[-2]) * 1000
)


#   // wrap around (-PI/n, PI/n]
wheel_pos_raw = np.fmod(wheel_pos_raw, 2 * math.pi / num_spokes)

# if (wheel_pos <= -math.pi / num_spokes):
#     wheel_pos += 2 * math.pi / num_spokes
# elif (wheel_pos > math.pi / num_spokes):
#     wheel_pos -= 2 * math.pi / num_spokes

wheel_pos_wrapped = np.where(
    wheel_pos_raw < -math.pi / num_spokes,
    wheel_pos_raw + 2 * math.pi / num_spokes,
    np.where(
        wheel_pos_raw >= math.pi / num_spokes,
        wheel_pos_raw - 2 * math.pi / num_spokes,
        wheel_pos_raw,
    ),
)

# from raw sensor values
wheel_vel = mot_vel / gear_ratio - torso_pitch_rate

print(wheel_vel_calc)
print(wheel_vel)

# >>> graphing >>>

fig, axs = plt.subplots(5, 1, figsize=(15, 8))

axs[0].plot(timestamp, torso_pitch_raw)
axs[0].minorticks_on()
axs[0].grid(which="both")
axs[0].set_ylabel("torso_pitch_raw")

# axs[1].plot(timestamp, torso_pitch_rate)
# axs[1].minorticks_on()
# axs[1].grid(which="both")
# axs[1].set_ylabel("torso_pitch_rate")

# axs[2].plot(timestamp, mot_pos)
# axs[2].minorticks_on()
# axs[2].grid(which="both")
# axs[2].set_ylabel("mot_pos")

# axs[3].plot(timestamp, mot_vel)
# axs[3].minorticks_on()
# axs[3].grid(which="both")
# axs[3].set_ylabel("mot_vel")

axs[1].plot(timestamp, torso_pitch_check)
axs[1].minorticks_on()
axs[1].grid(which="both")
axs[1].set_ylabel("torso_pitch_check")
# axs[1].set_ylim([-1, 3.1])

axs[2].plot(timestamp, mot_pos_raw)
axs[2].minorticks_on()
axs[2].grid(which="both")
axs[2].set_ylabel("mot_pos_raw")

axs[3].plot(timestamp, mot_pos_now)
axs[3].set_ylabel("mot_pos_now")
axs[3].minorticks_on()
axs[3].grid(which="both")

axs[4].plot(timestamp, wheel_pos_raw)
axs[4].set_ylabel("wheel_pos_raw")
axs[4].minorticks_on()
axs[4].grid(which="both")


# axs[3].plot(timestamp, wheel_vel_wrt_torso, label="wheel_vel_wrt_torso")
# axs[3].plot(timestamp, torso_pitch_check, label="torso_pitch_check")
# axs[3].plot(timestamp, wheel_pos, label="wheel_pos")
# axs[3].legend()
# axs[3].axhline(np.pi/10, color="red")
# axs[3].axhline(-np.pi/10, color="red")
# # >>>>>>> 4b0afdb (more runs)
# axs[3].minorticks_on()
# # axs[3].plot(timestamp, wheel_vel_calc)
# axs[3].grid(which="both")
# axs[3].set_ylabel("wheel_vel_calc")
# axs[3].set_ylim([-2, 3.2])

# axs[3].plot(timestamp, wheel_pos_wrt_torso, label="wheel_pos_wrt_torso")
# axs[3].plot(timestamp, torso_pitch_check, label="torso_pitch_check")
# axs[3].plot(timestamp, wheel_pos, label="wheel_pos")
# axs[3].legend()
# axs[3].minorticks_on()
# axs[3].grid(which="both")

plt.show()
plt.savefig(directory_path + "debugging/" + save_file, dpi=400)
