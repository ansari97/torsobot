clear;
clc;
close all;

% get the parameters
parameters;

% Initial conditions
init_theta = -pi/n; % initial theta
init_phi = 0; % initial phi
init_theta_dot = 4; % initial theta rate
init_phi_dot = 0; % initial phi rate

% Solver parameters
time_interval = [0 15]; % time interval for the ODE solution
solver_type = 'ode45';
solver_max_step = 0.1; % max time step; 0.02 is reasonable

e_sum = 0; % for PID controller

init_con = [init_theta, init_phi, init_theta_dot, init_phi_dot]; %error_sum

stop_vel = 0.02; % stop simulation if wheel angular velocity is less than this value

% struct for solver_param

solver_param.stop_vel = stop_vel;
solver_param.time_interval = time_interval;
solver_param.solver_type = solver_type;
solver_param.solver_max_step = solver_max_step;
solver_param.init_con = init_con;
    
% parameters
p.slope_angle = slope_angle;
p.robot_param = robot_param;
p.collision_angle = collision_angle;
p.solver_param = solver_param;
p.controller_param = controller_param;

y_after_0 =   init_con'
y_after_1 = takeOneStep(y_after_0, p)
y_after_2 = takeOneStep(y_after_1, p)
y_after_3 = takeOneStep(y_after_2, p)

% Initial guess; theta is always -collision angle; rest are randomized
theta = -collision_angle;

num_rand = 20;

a = -pi;
b = pi;
phi =  a + (b-a).*rand(num_rand,1);
% phi = 0*ones(num_rand,1)

a = 1;
b = 5;
theta_dot =  a + (b-a).*rand(num_rand,1);

a = 0;
b = 5;
phi_dot =  a + (b-a).*rand(num_rand,1);
phi_dot = zeros(num_rand,1);

% create a randomized vector of initial conditions
j = 10;
init_con = zeros([4, j]);
sol = zeros([5, j]);

init_con(1,:) = ones([1, j])*theta;

for i=1:j
    % i
    ind = randi(num_rand, [1, 3]);
    init_con(2, i) = phi(ind(1));
    init_con(3, i) = theta_dot(ind(2));
    init_con(4, i) = phi_dot(ind(3));

    % disp(init_con(:, i));

    % init_conditions
    init_con_for_sim = init_con(:, i);
    solver_param.init_con = init_con_for_sim;
    
    % function to solve
    p.solver_param = solver_param;
    limCyc = @(y) (takeOneStep(y, p) - y);
    
    % try to solve
    % try
        [y_val,fval,exitflag,output] = fsolve(limCyc, init_con(:, i));
        exitflag
        if exitflag >= 1
            % y_val
            sol(1, i) = exitflag;
            sol(2:5, i) = y_val;
        end
    % catch
    %     disp("error")
    % end
end

k = find(sol(1, :) == 1);

final_sol = sol(:, k);
size(final_sol);
