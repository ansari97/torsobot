function [y_sol, sol, event_sol, frame]= wheelSimulation(slope_angle, l, m, I, n, init_con, stop_vel, time_interval, solver_type, solver_max_step, phase_plot, fig_plot)
% wheelSimulation   Main function for simulating the 2D rimless wheel
%   [y_sol, sol, event_sol, frame]= wheelSimulation(slope_angle, l, m, I, n, init_con, stop_vel, time_interval, solver, solver_max_step, phase_plot, fig_plot)
%   
%   slope_angle is the angle of the slope in degrees, the slope is always
%       declining from right to left
% 
%   l is the spoke length in m
% 
%   m is the mass of the wheel in kg
% 
%   I in the mass moment of inertia in kg.m^2
% 
%   n is the number of spokes
% 
%   init_con is the vector of initial conditions in the form 
%       [initial angle, initial velocity]
% 
%   stop_vel is the minimum speed to stop the ode solution
% 
%   time_interval is the array of time in the form
%       [initial time, end time]
% 
%   solver_type is the ODE solver
% 
%   solver_max_step is the max step for the ODE solver
% 
%   phase_plot is a boolean to select
% 
%   fig_plot is a boolean to select

%% Define slope
%  Slope
% slope_angle = 1;  % degrees
slope_angle = deg2rad(slope_angle);  % angle in radians

%% Define parameters of the wheel
%  Wheel
% l = 0.5;   % spoke length in m
% m = 0.5;  % mass in kg
% I = 0.01;  % moment of inertia about center of mass/center of wheel in kgm^2
% n = 7; % spokes

spoke_angle = 2*pi/n; % angle between two spokes

J = I/(2*m*l^2); % radius of gyration

lam = 1/(2*J+1); % lambda, used as an intermediate variable

%% Collision event
% general case
% for downhill motion, collision angle is pi/n,
% for uphill it is -pi/n
collision_angle = pi/n;

% Define velocity loss coefficient
vel_coeff = (I + m*l^2*cos(spoke_angle))/(I + m*l^2);

%%  initial conditions
% init_ang = -pi/n; % initial angle
% init_vel = 0.5; % initial angular velocity
% init_con = [init_ang, init_vel];
% stop_vel = 0.00001; % minimum value; if velocity of wheel is less than this value, stop simulation

%% Differential equation for the swing
% dydt = @(t,y) [y(2); sin(y(1) + slope_angle)];

% time_interval = [0 6]; % time interval for the ODE solution
% collision_stop = false;
% num_collisions = 5;

% Simpe ODE45 solver
% [t, y] = ode45(dydt, time_interval, init_con);

% Define ODE event
E = odeEvent(EventFcn=@collisionEvent, ...
    Direction="ascending", ...
    Response="callback",...
    CallbackFcn=@collisionResponse);

% create ode object
F = ode(ODEFcn = @diffFunc, InitialValue = init_con, EventDefinition = E, Solver= solver_type, Parameters=[slope_angle, n, collision_angle, vel_coeff, stop_vel]);
% set solver options
F.SolverOptions.MaxStep = solver_max_step;

% Solve ODE
y_sol = solve(F, time_interval(1), time_interval(2), Refine=8);

%% Solution values
state = y_sol.Solution;

% matrix of solution values (time as row 1, angle as 2, ang_vel as 3)
t = y_sol.Time;
sol = [t; state(1, :); state(2, :)];

% matrix of event values (time as row 1, angle as 2, ang_vel as 3)
if ~isempty(y_sol.EventTime)
    event_sol = [y_sol.EventTime; y_sol.EventSolution(1,:); y_sol.EventSolution(2,:)];
else
    error('No collision occurs during the time interval used. Increase the time interval for the simulation.')
end

disp(strcat('Simulation time: ', num2str(max(t)), ' s'));

%% Plotting
% plot the angle and velocity wrt time
figure;
subplot(2, 1, 1);
plot(t, state(1, :));
hold on;
yline([0 -collision_angle collision_angle]);

axis([time_interval -collision_angle*1.5, collision_angle*1.5]);
title('Wheel Angle from normal');
xlabel('time (s)');
ylabel('angle (rad)');
hold off;

subplot(2, 1, 2);
plot(t, state(2, :));
hold on;
yline(0);

xlim(time_interval);
title('Angular velocity');
xlabel('time (s)');
ylabel('angular velocity (rad/s)');
hold off;

% plot the phase plot with plot_type=2, plot_type 1 is under construction
if phase_plot == true
    phasePlot(state(1, :), state(2, :), collision_angle, solver_max_step, 2);
end

% plot the wheel trajectory
if fig_plot == true
    frame = wheelTrajPlot(slope_angle, l, n, sol, event_sol);
else
    frame = 0;
end
end
