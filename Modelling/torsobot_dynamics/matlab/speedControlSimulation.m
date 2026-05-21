%%% implements speed control

cd 'C:\Users\ahmed\Torsobot\Modelling\torsobot_dynamics\matlab'

close all; % close all open figures
clear; % clear the workspace
clear collisionResponse; % clears the value of persistent counter variable
clc; % clear command window

addpath("./plotting");

% seed for random noise
rng(40);

%% get parameters
parameters;

%% Define slope
%  Slope, received as argument in degrees to the function
slope_angle = deg2rad(slope_angle);  % angle in radians

%% Initial conditions
theta_init = -collision_angle + 0.01; % initial theta (wheel angle)
phi_init = pi;%deg2rad(0 - slope_angle); % initial phi (torso angle from the reference axis)
theta_dot_init = 3; % initial theta rate
phi_dot_init = 0; % initial phi rate

%% Solver setup
time_interval = [0 100]; % time interval for the ODE solution
solver_type = 'ode45';
solver_max_step = 0.1; % max time step; 0.02 is reasonable
frame_skip = 100; % number of frames to skip for animation;
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

max_num_steps = 80;

E = odeEvent(EventFcn=@collisionEvent, ...
    Direction="ascending", ...
    Response="stop",...
    CallbackFcn=@collisionResponse);

% create ode object
ode_param.slope_angle = slope_angle;
ode_param.robot_param = robot_param;
ode_param.controller_param = controller_param;
ode_param.collision_angle = collision_angle;
ode_param.stop_vel = stop_vel;

% start at this time
t = time_interval(1);

state = [];
actual_time = [];

last_event_time = 0;

step_mean_vel = [];

% velocity controller
desired_wheel_vel = 1.5; % rad/s

kp_vel = 0.50;
ki_vel = 0.05;
vel_error_sum = 0;

desired_torso_pitch_vector = zeros(1, max_num_steps-1);
mean_vel_vector = zeros(1, max_num_steps);
vel_error_vector = zeros(1, max_num_steps-1);

prev_num_steps_for_vel_error = 10;

step_ctr = 1;

% get inverse function for theta given vel
mean_step_vel_func = load("./plotting/mat_files/mean_step_vel_func");

v = mean_step_vel_func.v;
phi = mean_step_vel_func.phi;
mean_step_vel_func = mean_step_vel_func.mean_step_vel_fit;

% get inverse function; get theta from vel
theta_func = solve(mean_step_vel_func == v, phi)
theta_func = theta_func(1)
theta_solution = double(subs(theta_func, v, desired_wheel_vel))

prev_mean_vel = 0; mean_vel = desired_wheel_vel; % will be overwritten in the first step

ode_param.controller_param.phi_desired = deg2rad(theta_solution); % in radians

limit_cycle_control = false;
limit_cycle_reached = false;

i_star = -1;

for i=1:max_num_steps

    if i==1
    desired_torso_pitch = deg2rad(theta_solution);
    
    else

        % get velocity error
        vel_error_vector(i) = desired_wheel_vel - mean_vel_vector(i-1);
       
        if i_star == -1 && i > prev_num_steps_for_vel_error && all(abs(vel_error_vector(i-prev_num_steps_for_vel_error:i))/desired_wheel_vel < (1/100))
            i_star = i-1; % save index where wheel velocity is within 1% of desired wheel velocity
        end

        if ~limit_cycle_control

             % get velocity error
            vel_error = desired_wheel_vel - mean_vel_vector(i-1);
            vel_error_sum = vel_error_sum + vel_error;
            
            %implement controller and get desired torso pitch
            
            kp_term = kp_vel*vel_error;
            ki_term = ki_vel*vel_error_sum;
                    
            desired_torso_pitch = deg2rad(theta_solution) + kp_term + ki_term;
            desired_torso_pitch = clip(desired_torso_pitch, 0, pi/2);
            

        else % if limit cycle control
            if i>2 && limit_cycle_reached

                 % get velocity error
                vel_error = desired_wheel_vel - mean_vel_vector(i-1);
                vel_error_sum = vel_error_sum + vel_error;
                
                %implement controller and get desired torso pitch
                
                kp_term = kp_vel*vel_error;
                ki_term = ki_vel*vel_error_sum;
                               
                desired_torso_pitch = deg2rad(theta_solution) + kp_term + ki_term;
                desired_torso_pitch = clip(desired_torso_pitch, 0, pi/2);
            end
        end
    end

    ode_param.controller_param.phi_desired = desired_torso_pitch; % in radians
    desired_torso_pitch_vector(i) = desired_torso_pitch;

    % Solve ODE
    F = ode(ODEFcn = @continuousDynamics, InitialValue = init_con, EventDefinition = E, Solver= solver_type, Parameters=ode_param);
    % set solver options
    F.SolverOptions.MaxStep = solver_max_step;
    F.RelativeTolerance = 1e-9; % Tighten from default 1e-3
    F.AbsoluteTolerance = 1e-12; % Tighten from default 1e-6

    y_sol = solve(F, t, t+100, Refine=8);

    % y_before = y_sol.Solution

    % pause()
    y_before = y_sol.EventSolution;

    % t and i does not do anything
    [stop,  y_after, p] = collisionResponse(t, y_before, i, ode_param);

    
    % t = y_sol.EventTime
    init_con = y_after;
    
    % calculate mean velocities
    step_duration = y_sol.EventTime;

    % mean_vel = mean(y_sol.Solution(3, :));
    mean_vel = trapz(y_sol.Time, y_sol.Solution(3, :))/step_duration;

    mean_vel_vector(i) = mean_vel;

    
    % step_mean_vel = [step_mean_vel, mean_vel];

    % save values
    actual_time = [actual_time, y_sol.Time + last_event_time];

    last_event_time = last_event_time + y_sol.EventTime;

    state = [state, y_sol.Solution(1:4, :)];

    % check limit cycle
    mean_vel_diff = (mean_vel - prev_mean_vel)/mean_vel
    limit_cycle_reached = abs(mean_vel_diff) < (0.5/100);
   
    prev_mean_vel = mean_vel;
    
    step_ctr = i;
    if actual_time(end)>time_interval(2)
        break;
    end
end

step_index = 1:step_ctr;

sol = [actual_time; state];

% plot(actual_time, state(1, :))
% figure;
% plot(actual_time, state(3, :));
% % plot(sol(1, :), sol(4, :), "ro")
% 
% figure;
% plot(actual_time, state(2, :));
% 
% figure;
% plot(1:step_ctr, rad2deg(desired_torso_pitch_vector(1:step_ctr)));
% 
% figure;
% plot(1:step_ctr, mean_vel_vector(1:step_ctr));



%% colors
color_torso_pos = [0.00, 0.45, 0.74];  % Dark Blue
color_torso_vel = [0.30, 0.75, 0.93];  % Light Blue
color_wheel_pos = [0.85, 0.33, 0.10];  % Dark Orange/Rust
color_wheel_vel = [0.93, 0.69, 0.13];  % Golden Orange
color_target    = [0.80, 0.20, 0.20];  % Muted Red for target ylines

color_vel_pre   = [0.64, 0.08, 0.18];  % Deep Crimson for pre-collision velocity

color_actuator_COT = [0.49, 0.18, 0.56];  % Plum/Purple (Clipped Work / COT)
color_work_net     = [0.47, 0.67, 0.19];  % Forest Green (Net Unclipped Work)
color_work_neg     = [0.60, 0.60, 0.60];  % Neutral Grey (Negative Work)

%% plot
fig1 = figure(1);
tl = tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

ax1 = nexttile(tl); 
plot(ax1, step_index, mean_vel_vector, "-", 'Color', color_wheel_vel, 'LineWidth', 2);
hold(ax1, 'on');
yline(ax1, desired_wheel_vel, "--", 'Color', color_target, 'LineWidth', 1.25)
plot(ax1, i_star, mean_vel_vector(i_star), "p",  "MarkerSize", 10 , "MarkerFaceColor", color_target) 

xlabel(ax1, "Step Index")
ylabel(ax1, "Mean Wheel Velocity (rad/s)")

ylim(ax1, [min(mean_vel_vector) - 0.5, max(mean_vel_vector) + 0.5]);
xlim(ax1, [step_index(1), step_index(end)]);

legend(ax1, "Actual", ...
    "Target",...
    "Within 1% of Target",...
    'Location', "best");

text(ax1, 0.02, 0.80, '(a)', 'Units', 'normalized', ...
    'FontSize', 10, 'FontName', 'Helvetica', ...
    'BackgroundColor', 'w', 'EdgeColor', 'none'); % 'w' = white background, 'k' = black border

grid(ax1, "on");

zoom_x = [i_star - prev_num_steps_for_vel_error, i_star + prev_num_steps_for_vel_error]; 
zoom_y = [min([desired_wheel_vel, mean_vel_vector(i_star - prev_num_steps_for_vel_error:i_star + prev_num_steps_for_vel_error)]) - 0.02, max([desired_wheel_vel, mean_vel_vector(i_star - prev_num_steps_for_vel_error:i_star + prev_num_steps_for_vel_error)]) + 0.02];

inset_ax = axes('Parent', fig1, 'Position', [0.15, 0.50, 0.3, 0.15]); 
plot(inset_ax, step_index, mean_vel_vector, "-", 'Color', color_wheel_vel, 'LineWidth', 1.5);
hold(inset_ax, 'on');
yline(inset_ax, desired_wheel_vel, "--", 'Color', color_target, 'LineWidth', 1)
plot(inset_ax, i_star, mean_vel_vector(i_star), "p", "MarkerSize", 10 , "MarkerFaceColor", color_target) 

xlim(inset_ax, zoom_x);
ylim(inset_ax, zoom_y);

grid(inset_ax, 'on');

hold(inset_ax, 'off');
hold(ax1, 'off');

% next tile
ax2 = nexttile(tl);
plot(ax2, step_index, rad2deg(desired_torso_pitch_vector), "-",'Color', color_torso_pos, 'LineWidth', 2);
hold(ax2, "on");
yline(ax2, theta_solution, "--", 'Color', color_target, 'LineWidth', 1.25)
plot(ax2, i_star, rad2deg(desired_torso_pitch_vector(i_star)), "p", "MarkerSize", 10, "MarkerFaceColor", color_target)

xlabel(ax2, "Step Index")
ylabel(ax2, "Desired Torso Pitch (Deg)")

ylim(ax2, [min(rad2deg(desired_torso_pitch_vector)) - 5, max(rad2deg(desired_torso_pitch_vector)) + 5]);
xlim(ax2, [step_index(1), step_index(end)]);

grid(ax2, "on");

current_ylim = get(ax2, 'YLim');

legend(ax2, "Actual", ...
    "Target",...
    'Location', "best");

text(ax2, 0.02, 0.80, '(b)', 'Units', 'normalized', ...
    'FontSize', 10, 'FontName', 'Helvetica', ...
    'BackgroundColor', 'w', 'EdgeColor', 'none'); % 'w' = white background, 'k' = black border

hold(ax2, "off");

format_my_plot(10, 5);

additional_text = "";
export_filename = "./matlab_plots/speed_controller_desired_vel_" + num2str(desired_wheel_vel) + "_" + num2str(limit_cycle_control) + additional_text + ".png";
exportgraphics(gcf, export_filename, 'Resolution', 600);

%%
% frames = wheelTrajPlot(slope_angle, robot_param, sol, frame_skip);

%%
diff = mean_vel_vector - desired_wheel_vel;

undershoot = min(diff)

undershoot_percent = undershoot/abs(desired_wheel_vel - mean_vel_vector(1))*100

[rise_time_90, rise_time_90_idx] = min(abs(0.9*(desired_wheel_vel - mean_vel_vector(1)) - mean_vel_vector))

[rise_time_90, rise_time_90_idx] = min((mean_vel_vector(4) + 0.9*(desired_wheel_vel - mean_vel_vector(1))))

v0       = mean_vel_vector(1);
v_ref    = desired_wheel_vel;
dv       = v_ref - v0;                          % negative for your downward step
target90 = v0 + 0.9*dv;

if dv < 0
    rise_idx = find(mean_vel_vector <= target90, 1, 'first');
else
    rise_idx = find(mean_vel_vector >= target90, 1, 'first');
end

% optional: interpolate to a fractional step for a cleaner number
k_before = rise_idx - 1;
frac     = (mean_vel_vector(k_before) - target90) / ...
           (mean_vel_vector(k_before) - mean_vel_vector(rise_idx));
rise_time_90_steps = k_before + frac;           % e.g., 3.4 steps