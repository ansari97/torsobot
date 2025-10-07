%%% main.m
% This file contains the main code for defining the dynamics of the
% torsobot moving on a slope
% The normal vector to the slope defines the reference axis
% positive angles are defined anticlockwise from the slope normal
%
% Ahmed Alam Ansari
%
% Updated:
% 9-22-2025

close all; % close all open figures
clear; % clear the workspace
clc; % clear command window

%% Change these variables
slope_angle = 0; % slope angle in degrees, positive slope is downwards from left to right

%% Initial conditions
theta_init = 0; % initial theta (wheel angle)
phi_init = deg2rad(180 - slope_angle); % initial phi (torso angle from the reference axis)
theta_dot_init = 0.5; % initial theta rate
phi_dot_init = 0; % initial phi rate

%% Plotting and saving options
phase_plot = true;
motion_plot = true; % for visualizing the robot motion
% make_movie = true;
save_movie = false;
frames_per_sec = 5;
save_var = true;

%% Controller parameters
% for PID controller
PID_controller.kp = 0.65;
PID_controller.ki = 0.00; % removed I = 0.05 term and added gravity compesnation for the torso angle
PID_controller.kd = 0.08;
PID_controller.control_max_integral = 4.0; % for the error sum (error integral term)

% make generic controller
controller_param = PID_controller;

controller_param.phi_desired = deg2rad(90);
controller_param.theta_dot_desired = 2;
controller_param.max_torque = 5.0; % at the wheel
controller_param.gear_ratio = 48/16*38/16;

%% Solver setup
time_interval = [0 5]; % time interval for the ODE solution
solver_type = 'ode45';
solver_max_step = 0.1; % max time step; 0.02 is reasonable
frame_skip = 10; % number of frames to skip for animation;
% skipping too many frames creates a seemingly disconneted animation

%% Do not change
% robot_param = {L, M, Iw, n, l, m, It};

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

% This block corrects the initial conditions
% going up and collision angle incorrectly set to just before colliding
% up
if theta_dot_init < 0 && theta_init == -collision_angle
    theta_init = collision_angle;

    % going down and collision angle incorrectly set to just before colliding
    % down
elseif theta_dot_init > 0 && theta_init == collision_angle
    theta_init = -collision_angle;
end

% resetting angle value to be within range
if abs(theta_init) > collision_angle
    % if going up
    if theta_dot_init < 0
        theta_init = collision_angle;
        % if going down
    else
        theta_init = -collision_angle;
    end
end

e_sum = 0; % for PID controller

init_con = [theta_init, phi_init, theta_dot_init, phi_dot_init, e_sum];

stop_vel = 0.02; % stop simulation if wheel angular velocity is less than this value

%% Do not change
% solver_param = {init_con, stop_vel, time_interval, solver_type, solver_max_step};
% struct for solver_param
solver_param.init_con = init_con;
solver_param.stop_vel = stop_vel;
solver_param.time_interval = time_interval;
solver_param.solver_type = solver_type;
solver_param.solver_max_step = solver_max_step;

if ~motion_plot
    % make_movie = false;
    save_movie = false;
end

% Run solver and plot
[sol, event_sol, frames] = robotSimulation(slope_angle, robot_param, solver_param, controller_param, phase_plot, motion_plot, frame_skip);

%% Energy graphs
theta = sol(2, :);
phi = sol(3, :);
theta_dot = sol(4, :);
phi_dot = sol(5, :);
KE = 1/2*(M*L^2 + Iw + m*L^2)*theta_dot.^2 + 1/2*(m*l^2 + It)*phi_dot.^2 + m*l*L*theta_dot.*phi_dot.*cos((theta - phi));
PE = g*(m+M)*L*cos(theta + deg2rad(slope_angle)) + m*g*l*cos(phi + deg2rad(slope_angle));

E = KE + PE;

figure;
subplot(3,1,1);
plot(sol(1, :), KE);
hold on;
title("Kinetic Energy vs time");
xlabel("time(s)");
ylabel("Kinetic Energy");
hold off;

subplot(3,1,2);
plot(sol(1, :), PE);
hold on;
title("Potential Energy vs time");
xlabel("time(s)");
ylabel("Potential Energy");
hold off;

subplot(3,1,3);
plot(sol(1, :), E);
hold on;
title("Total Energy vs time");
xlabel("time(s)");
ylabel("Total Energy");
hold off;

%% save variables into a .mat file for later use
datetime_filename = string(datetime("today", Format="uuuu-MM-dd")) + "_" + string(datetime("now", Format = "HH-mm-ss")) + ".mp4";

if save_var
    save("./mat_files/var_" + datetime_filename + ".mat", 'sol', 'frames');
end

%% video code
% make a video
if save_movie
    % t = sol(1,:);
    % len = length(t);
    % time_max = max(t); % end time for the solution; might be different from
    % % required simulation time if simulation ends prematurely
    % time_min = min(t); % start time of the solution (should be 0)
    %
    % t_movie = time_min:solver_max_step*10:time_max; % time for the movie frame
    % % t_ind = zeros(length(t_movie))
    % for i = 1:length(t_movie)
    %     ind = find(abs(t - t_movie(i)) < solver_max_step/10);
    %
    %     if ~isempty(ind) > 0
    %         if length(ind)>1
    %             ind = round(mean(ind));
    %         end
    %         t_ind(i) = ind;
    %     elseif isempty(ind)
    %         % do nothing
    %     end
    % end


    % frame = frame(t_ind);

    % play once
    % f = figure;
    % movie(f, frames, 1, frames_per_sec);
    % close(f);
    % drawnow;

    filename = ".\videos\" + datetime_filename;
    v = VideoWriter(filename, "MPEG-4");
    open(v);
    writeVideo(v, frames);
    close(v);
    disp("Saving video file to" + filename);

end
