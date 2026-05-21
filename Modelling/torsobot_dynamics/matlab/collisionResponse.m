function [stop,  y, p] = collisionResponse(t, y, i, p)
%%% collisionResponse function maps the state right before collision to 
% right after collision
% y = [theta, phi, theta_dot, phi_dot]; the state vector
% p is the vector of parameters received as argument to the ODE function

% define counter variable as syatic
persistent collision_ctr;

% initialize on first run
if isempty(collision_ctr)
    collision_ctr = 0;
end

collision_ctr = collision_ctr + 1;


%% Define collision dynamics here
% Angular momentum and linear momentum for the whole body is conserved
% See Maple files for details

% unpack variables
stop_vel = p.stop_vel;

robot_param = p.robot_param;
L = robot_param.L;
M = robot_param.M;
Iw = robot_param.Iw;
n = robot_param.n;
l_t = robot_param.l_t;
l = robot_param.l;
m = robot_param.m;
It = robot_param.It;

% R(q)*q = s(q)
% left q or 'q+' is the state right after the collision
% right q or 'q-' is the state right before the collision
% R and s are dependent on 'q-'


S = It + m*l^2;
R = m*l*L*cos(y(1) - y(2));
W = m*l*L*cos(y(1) + y(2));
s1 = S*y(4) + R*y(3);

Q = Iw + L^2*M + L^2*m + m*l*L*cos(y(1) + y(2));
P = It + l^2*m + m*l*L*cos(y(1) + y(2));

% right side vector
s2 = (Iw + L^2*(m + M)*cos(2*y(1)) + m*l*L*cos(y(1) - y(2)))*y(3) + P*y(4);

add_random_noise = false;
step_length = sqrt(2*L^2 - 2*L*L*cos(2*pi/n));
angle = (pi - pi/n)/2;

% inject random noise to wheel velocity
if add_random_noise && mod(collision_ctr, 20) == 0
    lower_bound = -0.5;
    upper_bound = -lower_bound;
    linear_impulse = lower_bound + (upper_bound - lower_bound) * rand();

    angular_impulse = step_length * linear_impulse * sin(angle);

    s2 = s2 + angular_impulse;
    disp("adding noise...")
end

% left side matrix
R = [1, 0, 0, 0;
    0, 1, 0, 0;
    0, 0, W, S;
    0, 0, Q, P];

s = [-y(1), y(2), s1, s2]';

y = R\s; %y(5)];  % error_sum does not change during collision since phi (y(2)) is continuous at collision


%


% if collision_ctr == 12
%     y(3) = 1.5;
%     disp("pusihing forward...");
% 
% else


stop = false;
% stop after n steps
if collision_ctr == 40
    stop = true;
end

% if theta velocity is less than a threshold value, stop solving

if abs(y(3)) < stop_vel
    stop = true;
    disp("wheel almost stopped rocking!")
end

% stop if torso touches the ground
% if l>L && abs(L*cos(y(1)) + l*cos(y(2))) < 0.001
%     stop = true;
%     disp("Torso collided with the ground!")
% end
end