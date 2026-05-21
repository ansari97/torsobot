%%% main.m
% This file contains the main code for defining the dynamics of the
% torsobot moving on a slope
% The normal vector to the slope defines the reference axis
% positive angles are defined anticlockwise from the slope normal
%
% Ahmed Alam Ansari
%
% Updated:
% 3-17-2026

close all; % close all open figures
clear; % clear the workspace
clear collisionResponse; % clears the value of persistent counter variable
clc; % clear command window

addpath('C:\Users\ahmed\Torsobot\Modelling\torsobot_dynamics\matlab\plotting');

% seed for random noise
rng(40);

% get the parameter file
parameters;
% pause();

%% Initial conditions
theta_init = -pi/n + 0.001; % initial theta (wheel angle)
phi_init = pi;%deg2rad(0 - slope_angle); % initial phi (torso angle from the reference axis)
theta_dot_init = 1.5; % initial theta rate
phi_dot_init = 0; % initial phi rate


%% Plotting and saving options
phase_plot = true;
motion_plot = false; % for visualizing the robot motion
% make_movie = true;
save_movie = false;
frames_per_sec = 5;
save_var = true;

%% Solver setup
time_interval = [0 5]; % time interval for the ODE solution
solver_type = 'ode45';
solver_max_step = 01; % max time step; 0.02 is reasonable
frame_skip = 1; % number of frames to skip for animation;
% skipping too many frames creates a seemingly disconneted animation

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

init_con = [theta_init, phi_init, theta_dot_init, phi_dot_init]; %, e_sum];

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

save_filename = num2str(rad2deg(controller_param.phi_desired)) + "_sol_data";
save("mat_files\" + save_filename + ".mat", "sol");

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

    num_frames = length(frames);

    % Generate the global colormap ('map') from the very first frame
    [~, map] = rgb2ind(frames(1).cdata, 256, 'nodither');
    
    % Preallocate the 4D array for speed (Height x Width x 1 x NumFrames)
    [h, w, ~] = size(frames(1).cdata);
    im = zeros(h, w, 1, num_frames, 'uint8'); 
    
    % Convert all frames into the 4D indexed array
    for k = 1:num_frames 
        % We use your existing 'frames' variable instead of getframe
        im(:,:,1,k) = rgb2ind(frames(k).cdata, map, 'nodither');
    end
    
    % Dump the entire 4D array to a GIF in one shot
    gif_filename = filename + ".gif";
    imwrite(im, map, gif_filename, 'DelayTime', 1/30, 'LoopCount', inf);
    disp("Saving GIF file to " + gif_filename);

end
