%%% for plotting csvs

clc;
clear;
close all;

%% Get files

% Enter filename
filename = input("CSV filename: ", 's');

% csv directory and file name
csv_directory  = "..\..\..\..\Code\ros2_ws\data_logs\csvs\";
csv_filename = csv_directory + filename + ".csv";

% yaml directory and file name
yaml_directory  = "..\..\..\..\Code\ros2_ws\data_logs\metadata\";
yaml_filename = yaml_directory + filename + ".yaml";

disp(["Reading file from: ", csv_filename])

%% get data from yaml
yaml_data = yaml.loadFile(yaml_filename);

desired_torso_pitch_deg = yaml_data.x___.ros__parameters.desired_torso_pitch_deg;
controller = yaml_data.x___.ros__parameters.controller;
kp = yaml_data.x___.ros__parameters.kp;
ki = yaml_data.x___.ros__parameters.ki;
kd = yaml_data.x___.ros__parameters.kd;
wheel_max_torque = yaml_data.x___.ros__parameters.wheel_max_torque;
control_max_integral = yaml_data.x___.ros__parameters.control_max_integral;

%% get data from csv
csv_data = readtable(csv_filename);

time = csv_data.timestamp;
time = time - time(1); % get relative time from the first value
time = time/(1e6); % convert to ms

torso_pitch = csv_data.torso_pitch;
torso_pitch_rate = csv_data.torso_pitch_rate;
wheel_pos = csv_data.wheel_pos;
wheel_vel = csv_data.wheel_vel;
wheel_torque = csv_data.wheel_torque;
wheel_cmd_torque = csv_data.wheel_cmd_torque;
encoder_ang = csv_data.encoder_ang;
encoder_speed = csv_data.encoder_speed;

%% time data plotting
torso_pitch = rad2deg(torso_pitch); % convert to deg
fig1 = figure(1);
title_text = "Pitch: " + desired_torso_pitch_deg + "^o";
sgtitle(title_text);

subplot(5, 1, 1);
plot(time, wheel_torque);hold on;
plot(time, wheel_cmd_torque, 'r'); hold off;
legend("wheel torque", "wheel cmd torque", Location="northeast");
ylabel("wheel torque (Nm)")
grid on; 
grid minor;

subplot(5, 1, 2);
plot(time, torso_pitch); hold on;
plot(time, desired_torso_pitch_deg*ones(length(time)), 'r'); hold off;
legend("torso pitch", "desired torso pitch deg", Location="northeast");
ylabel("torso pitch (deg)");
grid on; 
grid minor;

subplot(5, 1, 3);
plot(time, torso_pitch_rate);
ylabel("torso pitch rate (rad/s)");
grid on; 
grid minor;

subplot(5, 1, 4);
plot(time, wheel_pos);
yline(-pi/10, 'r');
yline(pi/10, 'r');
ylabel("wheel pos (rad)");
grid on; 
grid minor;

subplot(5, 1, 5);
plot(time, wheel_vel);
ylabel("wheel vel (rad/s)");
xlabel("Time (ms)");
grid on; 
grid minor;

% save figures
save_directory  = "..\..\..\..\Code\ros2_ws\data_logs\graphs\time_plots\";
save_filename = save_directory + filename;

exportgraphics(fig1, save_filename + "_time.png")

%% scatter plot
fig2 = figure(2);
c = linspace(0, 1, length(time));
scatter(wheel_pos, wheel_vel, 50, c, 'filled')
hold on;
grid on; 
grid minor;
xline(-pi/10, 'r');
xline(pi/10, 'r');
colorbar;
xlabel("wheel pos (rad)");
ylabel("wheel vel (rad/s)");
hold off;

% save figures
save_directory  = "..\..\..\..\Code\ros2_ws\data_logs\graphs\phase_plots\";
save_filename = save_directory + filename;

exportgraphics(fig2, save_filename + "_phase.png")
