function [sol, event_sol, frames]= robotSimulation(slope_angle, robot_param, solver_param, controller_param, phase_plot, motion_plot, frame_skip)
%%% robotSimulation   Main function for simulating the torsobot
%   [sol, event_sol, frame] = robotSimulation(slope_angle, L, M, Iw, n, init_con, stop_vel, time_interval, solver, solver_max_step, phase_plot, fig_plot)
%   
%   Inputs:
%   slope_angle is the angle of the slope in degrees, the slope is always
%       declining from right to left
% 
%   robot_param contains the physical attributes of the robot
% 
%   solver_param contains the initial conditions, wheel stopping velocity,
%   time_interval for integration, solver_type and solver_max_step
% 
%   phase_plot is a boolean variable to plot the phase plots
% 
%   fig_plot is a boolean variable to plot the wheel animation
% 
%   Output:
%   sol is a 5 x p matrix containing time and state values as rows
%   event_sol is the time and state vectors when the collision event occurs
%   frame contains the animation frames

%% Define slope
%  Slope, received as argument in degrees to the function
slope_angle = deg2rad(slope_angle);  % angle in radians

%% Define intermediate variables
L = robot_param.L;
M = robot_param.M;
Iw = robot_param.Iw;
n = robot_param.n;
l_t = robot_param.l_t;
l = robot_param.l;
m = robot_param.m;
It = robot_param.It;

spoke_angle = 2*pi/n; % angle between two spokes

init_con = solver_param.init_con;
stop_vel = solver_param.stop_vel;
time_interval = solver_param.time_interval;
solver_type = solver_param.solver_type;
solver_max_step = solver_param.solver_max_step;

%% Collision event
% general case
% for downhill motion, collision angle is pi/n,
% for uphill it is -pi/n
collision_angle = pi/n;

%% Dynamics setup
% Define ODE event
% for some reason "both" direction does not work for the collision event
E = odeEvent(EventFcn=@collisionEvent, ...
    Direction="ascending", ...
    Response="callback",...
    CallbackFcn=@collisionResponse);

% create ode object
ode_param.slope_angle = slope_angle;
ode_param.robot_param = robot_param;
ode_param.controller_param = controller_param;
ode_param.collision_angle = collision_angle;
ode_param.stop_vel = stop_vel;

F = ode(ODEFcn = @continuousDynamics, InitialValue = init_con, EventDefinition = E, Solver= solver_type, Parameters=ode_param);
% set solver options
F.SolverOptions.MaxStep = solver_max_step;
F.RelativeTolerance = 1e-9; % Tighten from default 1e-3
F.AbsoluteTolerance = 1e-12; % Tighten from default 1e-6

% Solve ODE
y_sol = solve(F, time_interval(1), time_interval(2), Refine=8);

%% Solution values
state = y_sol.Solution;

% Wrap angles from -pi to pi
state(1, :) = wrapToPi(state(1,:));
state(2, :) = wrapTo2Pi(state(2,:));

% matrix of solution values 
% time as row 1, theta as 2, phi as 3, theta_dot as 4, phi_dot as 5
t = y_sol.Time;
sol = [t; state(1, :); state(2, :); state(3, :); state(4, :)];

% matrix of event values (time as row 1, angle as 2, ang_vel as 3)
if ~isempty(y_sol.EventTime)
    event_sol = [y_sol.EventTime; y_sol.EventSolution(1,:); y_sol.EventSolution(2,:); y_sol.EventSolution(3,:); y_sol.EventSolution(4,:)];
else
    % error('No collision occurs during the time interval used. Increase the time interval for the simulation.')
end

disp(strcat('Simulation time: ', num2str(max(t)), ' s'));

%% Plotting
% plot the robot state wrt time

pad_ylim = @(y, frac) [min(y) - frac*(max(y)-min(y)), ...
                       max(y) + frac*(max(y)-min(y))];

time = t;
wheel_pos = state(1, :);
torso_pitch = state(2, :);
wheel_vel = state(3, :);
torso_pitch_rate = state(4, :);
desired_torso_pitch_deg = rad2deg(controller_param.phi_desired);

color_torso_pos = [0.00, 0.45, 0.74];  % Dark Blue
color_torso_vel = [0.30, 0.75, 0.93];  % Light Blue
color_wheel_pos = [0.85, 0.33, 0.10];  % Dark Orange/Rust
color_wheel_vel = [0.93, 0.69, 0.13];  % Golden Orange
color_target    = [0.80, 0.20, 0.20];  % Muted Red for target ylines

% plot in deg
torso_pitch_deg = rad2deg(torso_pitch); 

fig2 = figure(2);
tiledlayout(4, 1, 'TileSpacing', 'loose', 'Padding', 'compact');

ax_a = nexttile;
plot(time, torso_pitch_deg, 'Color', color_torso_pos, 'LineWidth', 2);
hold on;
yline(desired_torso_pitch_deg, "--", 'Color', color_target, 'LineWidth', 1.5); 

xlim([time(1), time(end)]);
ylim(pad_ylim(torso_pitch_deg, 0.10));
% ylim([0.80*min(torso_pitch), 1.20*max(torso_pitch)])

ylabel('Torso Pitch (deg)');

legend('Actual Pitch', 'Target Pitch', 'Location', 'northeast');

grid on;

text(0.02, 0.85, '(a)', 'Units', 'normalized', ...
    'FontSize', 10, 'FontName', 'Helvetica', ...
    'BackgroundColor', 'w', 'EdgeColor', 'none'); % 'w' = white background, 'k' = black border

hold off;

nexttile;
plot(time, torso_pitch_rate, 'Color', color_torso_vel, 'LineWidth', 2);
hold on;

xlim([time(1), time(end)]);
ylim(pad_ylim(torso_pitch_rate, 0.10));
% ylim([1.20*min(torso_pitch_rate), 1.20*max(torso_pitch_rate)])

ylabel('Torso Pitch Rate (rad/s)');

grid on;

text(0.02, 0.85, '(b)', 'Units', 'normalized', ...
    'FontSize', 10, 'FontName', 'Helvetica', ...
    'BackgroundColor', 'w', 'EdgeColor', 'none'); % 'w' = white background, 'k' = black border

hold off;

nexttile;
plot(time, wheel_pos, 'Color', color_wheel_pos, 'LineWidth', 2); 
hold on;
yline(-pi/10, '--', 'Color', color_target, 'LineWidth', 1.5);
yline(pi/10, '--', 'Color', color_target, 'LineWidth', 1.5);

xlim([time(1), time(end)]);
ylim([-0.33, 0.33])

ylabel("Wheel Position (rad)");

grid on;

text(0.02, 0.85, '(c)', 'Units', 'normalized', ...
    'FontSize', 10, 'FontName', 'Helvetica', ...
    'BackgroundColor', 'w', 'EdgeColor', 'none'); % 'w' = white background, 'k' = black border

hold off;


nexttile;
plot(time, wheel_vel, 'Color', color_wheel_vel, 'LineWidth', 2); 
hold on;

xlim([time(1), time(end)]);
ylim(pad_ylim(wheel_vel, 0.10));
% ylim([0.80*min(wheel_vel), 1.20*max(wheel_vel)])

ylabel("Wheel Velocity (rad/s)");
xlabel("Time (s)");

grid on; 

text(0.02, 0.85, '(d)', 'Units', 'normalized', ...
    'FontSize', 10, 'FontName', 'Helvetica', ...
    'BackgroundColor', 'w', 'EdgeColor', 'none'); % 'w' = white background, 'k' = black border

hold off;

format_my_plot(10, 7.5)

main_pos = ax_a.Position;   % [x y w h] of tile (a) in figure coords

% inset placed top-right inside tile (a)
inset_x = main_pos(1) + 0.25 * main_pos(3);
inset_y = main_pos(2) + 0.35 * main_pos(4);
inset_w = 0.50 * main_pos(3);
inset_h = 0.40 * main_pos(4);

ax_inset = axes('Position', [inset_x, inset_y, inset_w, inset_h]);

% data in the zoom window
mask = time >= 1.5 & time <= 4.5;
plot(ax_inset, time(mask), torso_pitch_deg(mask), ...
     'Color', color_torso_pos, 'LineWidth', 1.5);
hold(ax_inset, 'on');
yline(ax_inset, desired_torso_pitch_deg, '--', ...
      'Color', color_target, 'LineWidth', 1);

xlim(ax_inset, [1.5, 4.5]);
ylim(ax_inset, pad_ylim(torso_pitch_deg(mask), 0.10));

% no labels, just a clean box
% set(ax_inset, 'XTick', [], 'YTick', []);
grid(ax_inset, 'on')
box(ax_inset, 'on');
ax_inset.LineWidth = 0.8;

% save figures
filename = "presentation_state_vs_time_" +  num2str(desired_torso_pitch_deg) + ".png";

save_directory  = ".\matlab_plots\";
save_filename = save_directory + filename;

exportgraphics(gcf, save_filename, 'Resolution', 600);

%% external calculations for torque
T = zeros(1, length(t));
e = zeros(1, length(t));
for i = 1:1:length(t)
    % This is no longer the torque because this returns the auxiliary input u
    % [T(i), e(i)] = PDTorqueController(t(i), state(:, i), controller_param);
    [T(i), e(i)] = PDGCTorqueController(t(i), state(:, i), ode_param);
end

%% calculations for motor power at the wheel
color_wheel_torque = [0.00, 0.45, 0.74];  % Dark Blue
color_wheel_power = [0.93, 0.69, 0.13];  % Golden Orange

wheel_rel_vel = state(3, :) - state(4, :);
mot_pow = T.*wheel_rel_vel;
f4 = figure(4);

tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(time, T, 'Color', color_wheel_torque, 'LineWidth', 2); 
hold on;

xlim([time(1), time(end)]);
ylim(pad_ylim(T, 0.10));


ylabel('Torque (Nm)');

grid on;

text(0.02, 0.85, '(a)', 'Units', 'normalized', ...
    'FontSize', 10, 'FontName', 'Helvetica', ...
    'BackgroundColor', 'w', 'EdgeColor', 'none'); % 'w' = white background, 'k' = black border

hold off;

ax_a = nexttile;
plot(time, mot_pow, 'Color', color_wheel_power, 'LineWidth', 2); 
hold on;

xlim([time(1), time(end)]);
ylim(pad_ylim(mot_pow, 0.10));

xlabel("Time (s)");
ylabel('Actuator Power (W)');

grid on;

text(0.02, 0.85, '(b)', 'Units', 'normalized', ...
    'FontSize', 10, 'FontName', 'Helvetica', ...
    'BackgroundColor', 'w', 'EdgeColor', 'none'); % 'w' = white background, 'k' = black border

hold off;

format_my_plot(10, 5);

main_pos = ax_a.Position;   % [x y w h] of tile (a) in figure coords

% inset placed top-right inside tile (a)
inset_x = main_pos(1) + 0.25 * main_pos(3);
inset_y = main_pos(2) + 0.35 * main_pos(4);
inset_w = 0.50 * main_pos(3);
inset_h = 0.40 * main_pos(4);

ax_inset = axes('Position', [inset_x, inset_y, inset_w, inset_h]);

% data in the zoom window
mask = time >= 1.5   & time <= 4.5;
plot(ax_inset, time(mask), mot_pow(mask), ...
     'Color', color_wheel_power, 'LineWidth', 1.5);
hold(ax_inset, 'on');


xlim(ax_inset, [1.5, 4.5]);
ylim(ax_inset, pad_ylim(mot_pow(mask), 0.10));

% no labels, just a clean box
% set(ax_inset, 'XTick', [], 'YTick', []);
grid(ax_inset, 'on')
box(ax_inset, 'on');
ax_inset.LineWidth = 0.8;

% save figures
filename = "presentation_actuator_effort_" +  num2str(desired_torso_pitch_deg) + ".png";

save_directory  = ".\matlab_plots\";
save_filename = save_directory + filename;

exportgraphics(gcf, save_filename, 'Resolution', 600);

% plot the wheel trajectory

if motion_plot == true
    frames = wheelTrajPlot(slope_angle, robot_param, sol, frame_skip);
else
    frames = 0;
end

if phase_plot == true
    phasePlot(sol, controller_param, collision_angle, solver_max_step, 2)
end
end
