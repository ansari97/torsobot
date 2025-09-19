function [stop,  y, p] = collisionResponse(t, y, i, p)
%%% collisionResponse function maps the state right before collision to 
% right after collision
% y = [theta, phi, theta_dot, phi_dot]; the state vector
% p is the vector of parameters received as argument to the ODE function

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

% left side matrix
R = [1, 0, 0, 0;
    0, 1, 0, 0;
    0, 0, W, S;
    0, 0, Q, P];

s = [-y(1), y(2), s1, s2]';

y = [R\s; y(5)];  % error_sum does not change during collision since phi (y(2)) is continuous at collision

% if theta velocity is less than a threshold value, stop solving
stop = false;
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