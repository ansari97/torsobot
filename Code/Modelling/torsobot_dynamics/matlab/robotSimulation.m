function [sol, event_sol, frame]= robotSimulation(slope_angle, robot_param, solver_param, phase_plot, fig_plot, frame_skip)
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
%  Slope, received as argument to the function
slope_angle = deg2rad(slope_angle);  % angle in radians

%% Define intermediate variables
[L, M, Iw, n, l, m, It] = robot_param{:};
spoke_angle = 2*pi/n; % angle between two spokes

[init_con, stop_vel, time_interval, solver_type, solver_max_step] = solver_param{:};

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
F = ode(ODEFcn = @continuousDynamics, InitialValue = init_con, EventDefinition = E, Solver= solver_type, Parameters={slope_angle, robot_param, collision_angle, stop_vel});
% set solver options
F.SolverOptions.MaxStep = solver_max_step;

% Solve ODE
y_sol = solve(F, time_interval(1), time_interval(2), Refine=8);

%% Solution values
state = y_sol.Solution;

% Wrap angles from -pi to pi
state(1, :) = wrapToPi(state(1,:));
state(2, :) = wrapToPi(state(2,:));

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
subplot(4, 1, 1);
plot(t, state(1, :));
hold on;
yline([0 -collision_angle collision_angle]);

axis([time_interval -collision_angle*1.5, collision_angle*1.5]);
title('Wheel Angle from normal');
xlabel('time (s)');
ylabel('wheel angle (rad)');
hold off;

% theta_dot
subplot(4, 1, 2);
plot(t, state(3, :));
hold on;
yline(0);

xlim(time_interval);
title('Wheel Angular velocity');
xlabel('time (s)');
ylabel('theta\_dot (rad/s)');
hold off;

% phi
subplot(4, 1, 3);
plot(t, state(2, :));
hold on;
yline(0);

xlim(time_interval);
title('Torso Angle from normal');
xlabel('time (s)');
ylabel('torso angle (rad)');
hold off;

% phi
subplot(4, 1, 4);
plot(t, state(4, :));
hold on;
yline(0);

xlim(time_interval);
title('Torso Angular velocity');
xlabel('time (s)');
ylabel('phi\_dot (rad/s)');
hold off;

% plot the phase plot with plot_type=2, plot_type 1 is under construction
if phase_plot == true
    phasePlot(state(1, :), state(3, :), collision_angle, solver_max_step, 2);
end

% plot the wheel trajectory

if fig_plot == true
    frame = wheelTrajPlot(slope_angle, robot_param, sol, frame_skip);
else
    frame = 0;
end
end
