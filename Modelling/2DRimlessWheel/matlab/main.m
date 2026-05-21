% This file contains the main code for defining the dynamics of a
% 2D rimless wheel moving on a slope
% The normal vector to the slope defines the reference axis
% positive angles are defined anticlockwise from the slope normal

close all
clear
clc

%% Change these variables
time_interval = [0 10]; % time interval for the ODE solution

slope_angle = 2.5; % degrees
l = 0.4125;   % spoke length in m
m = 1.4;  % mass in kg
I = 0.15;  % moment of inertia about center of mass/center of wheel in kgm^2
n = 10; % spokes

% initial conditions
init_ang = -pi/n+0.001; % initial angle
init_vel = 2; % initial angular velocity


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
solver_max_step = 0.5;

% Plotting options
phase_plot = false;
fig_plot = false;
make_movie = false;

% Run solver and plot
[y_sol, sol, event_sol, frame] = wheelSimulation(slope_angle, l, m, I, n, init_con, stop_vel, time_interval, solver_type, solver_max_step, phase_plot, fig_plot, A);

filename = "./mat_files/solution_data"
save(filename, "y_sol", "sol", "event_sol", "frame", '-v7.3')

%% Energy analysis
t = sol(1,:);
theta = sol(2, :);
theta_dot = sol(3, :);
T = 1/2*(m*l^2+I)*theta_dot.^2;
V = m*g*l*cos(theta+deg2rad(slope_angle));
% E is not constant between collisions for non-dimensionalized 
E = T+V;

% figure;
% figure;
% subplot(3,1,1);
% plot(sol(1, :), T);
% hold on;
% title("Kinetic Energy vs time");
% xlabel("time(s)");
% ylabel("Kinetic Energy");
% hold off;
% 
% subplot(3,1,2);
% plot(sol(1, :), V);
% hold on;
% title("Potential Energy vs time");
% xlabel("time(s)");
% ylabel("Potential Energy");
% hold off;
% 
% subplot(3,1,3);
% plot(sol(1, :), E);
% hold on;
% title("Total Energy vs time");
% xlabel("time(s)");
% ylabel("Total Energy");
% hold off;
% 
% %% make a video
% if fig_plot && make_movie
%     t = sol(1,:);
%     len = length(t);
%     time_max = max(t); % end time for the solution
%     time_min = min(t); % start time of the solution (should be 0)
% 
%     t_movie = time_min:solver_max_step*10:time_max; % time for the movie frame
%     % t_ind = zeros(length(t_movie))
%     for i = 1:length(t_movie)
%         ind = find(abs(t - t_movie(i)) < solver_max_step/10);
% 
%         if ~isempty(ind) > 0
%             if length(ind)>1
%                 ind = round(mean(ind));
%             end
%             t_ind(i) = ind;
%         elseif isempty(ind)
%             % do nothing
%         end
%     end
% 
% 
%     frame = frame(t_ind);
%     movie(frame, 1, 10);
% end
% 
% 
% %% average velocity over step
% pos_diff  = diff(theta);
% impact_idx = find(abs(pos_diff)>0.5);
% 
% 
% t = sol(1,:);
% 
% start_idx = impact_idx(end-1);
% end_idx = impact_idx(end);
% 
% t_start = t(start_idx);
% t_end = t(end_idx);
% 
% mean_speed = trapz(t(start_idx: end_idx), theta_dot(start_idx: end_idx))/(t_end-t_start)
% 
% step_length = sqrt(2*l^2 - 2*l*l*cos(2*pi/n));
% 
% COM_speed = l/(t_end-t_start)
% Fr = COM_speed^2/(g*l)
% Fr_root = sqrt(Fr)

color_torso_pos = [0.00, 0.45, 0.74];  % Dark Blue
color_torso_vel = [0.30, 0.75, 0.93];  % Light Blue
color_wheel_pos = [0.85, 0.33, 0.10];  % Dark Orange/Rust
color_wheel_vel = [0.93, 0.69, 0.13];  % Golden Orange
color_target    = [0.80, 0.20, 0.20];  % Muted Red for target ylines

fig2 = figure(2);
fig1.Units = 'pixels';
fig1.Position = [100, 100, 800, 800];   % wide figure

hold('on');

% static elements (drawn once, don't move)
xline([-pi/10, pi/10], "--", 'Color', color_target, 'LineWidth', 1.5);
xlim(0.33*[-1, 1]);
ylim([min(theta_dot)-0.05, max(theta_dot)+0.05]);       % fix ylim so it doesn't jump
xlabel("Wheel Position (rad)");
ylabel('Wheel Velocity (rad/s)');
grid('on');

% scatter handle — starts empty, grows each frame
h_scatter = scatter(theta, theta_dot, 40, t, 'o', 'filled', ...
    'MarkerEdgeColor', [0.15 0.15 0.15], ...
    'MarkerFaceAlpha', 0.8, ...
    'LineWidth', 0.5);

colormap('parula');
ax2.FontSize = 14;
xlabel("Wheel Position (rad)", 'FontSize', 16);
ylabel('Wheel Velocity (rad/s)', 'FontSize', 16);
clim([min(t), max(t)]);           % fix color range so colors are stable
cb = colorbar();
cb.Label.String = 'Time (s)';
cb.Label.FontSize = 12;