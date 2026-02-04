%%% parameters.m
% This file contains the robot and simulation parameters

%% Change these variables
slope_angle = 0; % slope angle in degrees, positive slope is downwards from left to right

%% Robot Parameters
% All in SI units, m, kg, s
% wheel
L = 412.5e-3;   % spoke length in m
M = 2*1.3;  % mass in kg
Iw = 136917590e-9;  % moment of inertia about com/center of the wheel in kgm^2
n = 10; % number of spokes

% torso
l_t = 253.18e-3; % total length of torso from axis of revolution to top
l = sqrt(83.6^2+6.12^2)/1000; % distance of torso coM to wheel center
m = 1577.83e-3; % torso mass in kg
It = 9324204e-9;%m*l^2; % torso moment of inertia about the y axis coincident with the coM of the torso

% struct of robot parameters
robot_param.L = L;
robot_param.M = M;
robot_param.Iw = Iw;
robot_param.n = n;
robot_param.l_t = l_t;
robot_param.l = l;
robot_param.m = m;
robot_param.It = It;

% collision angle
collision_angle = pi/n;

g = 9.81;   % gravitational acceleration in m/s^2

%% Controller parameters
% for PID controller
desired_settling_period = 0.1;
PID_controller.kp = (4.7/(desired_settling_period*0.8))^2;
PID_controller.ki = 0.00; % removed I = 0.05 term and added gravity compesnation for the torso angle
PID_controller.kd = 2*1*(4.7/(desired_settling_period*0.8));
PID_controller.control_max_integral = 4.0; % for the error sum (error integral term)

% make generic controller
controller_param = PID_controller;

controller_param.phi_desired = deg2rad(90);
controller_param.theta_dot_desired = 2;
controller_param.max_torque = 5.0; % at the wheel
controller_param.gear_ratio = 48/16*38/16;