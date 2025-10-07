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
figure;
% theta
subplot(5, 1, 1);
plot(t, state(1, :));
hold on;
yline([0 -collision_angle collision_angle]);

axis([time_interval -collision_angle*1.5, collision_angle*1.5]);
title('Wheel Angle from normal');
xlabel('time (s)');
ylabel('wheel angle (rad)');
hold off;

% theta_dot
subplot(5, 1, 2);
plot(t, state(3, :));
hold on;
yline(0);

xlim(time_interval);
title('Wheel Angular velocity');
xlabel('time (s)');
ylabel('theta_{dot} (rad/s)');
hold off;

% phi
subplot(5, 1, 3);
plot(t, state(2, :));
hold on;
yline(0);
plot(t, ones(size(t))*controller_param.phi_desired, "r-");
xlim(time_interval);
title('Torso Angle from normal');
xlabel('time (s)');
ylabel('torso angle (rad)');
hold off;

% phi_dot
subplot(5, 1, 4);
plot(t, state(4, :));
hold on;
yline(0);

xlim(time_interval);
title('Torso Angular velocity');
xlabel('time (s)');
ylabel('phi_{dot} (rad/s)');
hold off;

% external calculations for torque
for i = 1:1:length(t)
    [T(i), e(i)] = torqueController(t(i), state(:, i), controller_param);
end

% calculations for motor power at the wheel
mot_pow = T.*state(3, :);
figure;

subplot(2, 1, 1);
plot(t, T);
hold on;
yline(0);

xlim(time_interval);
title('Torque at joint');
xlabel('time (s)');
ylabel('Torque (Nm)');
hold off;

subplot(2, 1, 2);
plot(t, mot_pow);
hold on;
yline(0);

xlim(time_interval);
title('Motor power');
xlabel('time (s)');
ylabel('Motor Power (W)');
hold off;

% plot the phase plot with plot_type=2, plot_type 1 is under construction
if phase_plot == true
    phasePlot(sol, controller_param, collision_angle, solver_max_step, 2);
end

% plot the wheel trajectory

if motion_plot == true
    frames = wheelTrajPlot(slope_angle, robot_param, sol, frame_skip);
else
    frames = 0;
end
end
