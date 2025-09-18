clear;
clc;
close all;

slope_angle = 3; % slope angle in degrees, positive slope is downwards
slope_angle = deg2rad(slope_angle);

L = 400e-3;   % spoke length in m
M = 400e-3;  % mass in kg
Iw = 32e-3;  % moment of inertia about center of mass/center of wheel in kgm^2
n = 12; % number of spokes

l = 400e-3; % distance of torso coM to wheel center
m = 500e-3; % torso mass in kg
It = 80e-3; % torso moment of inertia about the y axis coincident with the coM of the torso

robot_param = {L, M, Iw, n, l, m, It};
collision_angle = pi/n;

% Initial conditions
init_theta = -pi/n; % initial theta
init_phi = 0; % initial phi
init_theta_dot = 4; % initial theta rate
init_phi_dot = 0; % initial phi rate

init_con = [init_theta, init_phi, init_theta_dot, init_phi_dot];

stop_vel = 0.02; % stop simulation if wheel angular velocity is less than this value
time_interval = [0 10]; % time interval for the ODE solution
solver_type = 'ode45';
solver_max_step = 0.005;

solver_param = {stop_vel, time_interval, solver_type, solver_max_step};

p = {slope_angle, robot_param, collision_angle, solver_param};

y_after_0 =   init_con'
y_after_1 = takeOneStep(y_after_0, p)

n = 10;
theta = -collision_angle;

a = -pi;
b = pi;
phi =  a + (b-a).*rand(n,1);

a = 0;
b = 5;
theta_dot =  a + (b-a).*rand(n,1);

a = 0;
b = 5;
phi_dot =  a + (b-a).*rand(n,1);

j = 10;
init_con = zeros([4, j]);
sol = zeros([5, j]);

init_con(1,:) = ones([1, j])*theta;

limCyc = @(y) (takeOneStep(y, p) - y);

for i=1:j
    % i
    ind = randi(n, [1, 3]);
    init_con(2, i) = phi(ind(1));
    init_con(3, i) = theta_dot(ind(2));
    init_con(4, i) = phi_dot(ind(3));

    % disp(init_con(:, i));

    try
        [y_val,fval,exitflag,output] = fsolve(limCyc, init_con(:, i));
        if exitflag >= 1
            % y_val
            sol(1, i) = exitflag;
            sol(2:5, i) = y_val;
        end
    catch
        %
    end
end

k = find(sol(1, :) == 1);

final_sol = sol(:, k);
size(final_sol);
