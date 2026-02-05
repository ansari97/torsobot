% This file contains the main code for defining the dynamics of a
% 2D rimless wheel moving on a slope
% The normal vector to the slope defines the reference axis
% positive angles are defined anticlockwise from the slope normal

close all
clear
clc

%% Change these variables
time_interval = [0 20]; % time interval for the ODE solution

slope_angle = 5; % degrees
l = 0.45;   % spoke length in m
m = 1.4;  % mass in kg
I = 0.15;  % moment of inertia about center of mass/center of wheel in kgm^2
n = 10; % spokes

% initial conditions
init_ang = 0; % initial angle
init_vel = 1; % initial angular velocity


%% Do not change
g = 9.81;
A = (I + m*l^2)/(m*g*l);
collision_angle = pi/n;

if init_vel < 0 && init_ang == -collision_angle
    init_ang = collision_angle;
elseif init_vel > 0 && init_ang == collision_angle
    init_ang = -collision_angle;
end

if abs(init_ang) > collision_angle
    if init_vel < 0
        init_ang = collision_angle;
    else
        init_ang = -collision_angle;
    end
end

init_con = [init_ang, init_vel];

stop_vel = 0.02; % stop simulation if wheel angular velocity is less than this value

% Solver setup
solver_type = 'ode45';
solver_max_step = 0.1;

% Plotting options
phase_plot = false;
fig_plot = false;
make_movie = true;

% Run solver and plot
[y_sol, sol, event_sol, frame] = wheelSimulation(slope_angle, l, m, I, n, init_con, stop_vel, time_interval, solver_type, solver_max_step, phase_plot, fig_plot, A);


%% Energy analysis

theta = sol(2, :);
theta_dot = sol(3, :);
T = 1/2*(m*l^2+I)*theta_dot.^2;
V = m*g*l*cos(theta+deg2rad(slope_angle));
% E is not constant between collisions for non-dimensionalized 
E = T+V;

figure;
figure;
subplot(3,1,1);
plot(sol(1, :), T);
hold on;
title("Kinetic Energy vs time");
xlabel("time(s)");
ylabel("Kinetic Energy");
hold off;

subplot(3,1,2);
plot(sol(1, :), V);
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

%% make a video
if fig_plot && make_movie
    t = sol(1,:);
    len = length(t);
    time_max = max(t); % end time for the solution
    time_min = min(t); % start time of the solution (should be 0)

    t_movie = time_min:solver_max_step*10:time_max; % time for the movie frame
    % t_ind = zeros(length(t_movie))
    for i = 1:length(t_movie)
        ind = find(abs(t - t_movie(i)) < solver_max_step/10);

        if ~isempty(ind) > 0
            if length(ind)>1
                ind = round(mean(ind));
            end
            t_ind(i) = ind;
        elseif isempty(ind)
            % do nothing
        end
    end


    frame = frame(t_ind);
    movie(frame, 1, 10);
end
