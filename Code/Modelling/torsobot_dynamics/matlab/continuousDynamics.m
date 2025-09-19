function dydt = continuousDynamics(t,y,p)
%%% stanceDynamics defines the stance dynamics for the torsobot
% p is the parameter vector passed to the ode function

%% %% Define stance dynamics here
% equations of motion for a double pendulum
% See Maple files for details

% unpack variables
robot_param = p.robot_param;
L = robot_param.L;
M = robot_param.M;
Iw = robot_param.Iw;
n = robot_param.n;
l_t = robot_param.l_t;
l = robot_param.l;
m = robot_param.m;
It = robot_param.It;

g = 9.81; % acceleration due to gravity

slope_angle = p.slope_angle; % slope angle in radians

% Dynamics
R = m*l*L*cos(y(1) - y(2));
S = l^2*m + It;
V = Iw + L^2*(m + M);

f1 = m*l*L*y(3)^2*sin(y(1) - y(2)) + m*g*l*sin(slope_angle + y(2));
f2 = -m*l*L*y(4)^2*sin(y(1) - y(2)) + (m + M)*g*L*sin(slope_angle + y(1));

N = [1, 0, 0, 0;
    0, 1, 0, 0;
    0, 0, R, S;
    0, 0, V, R];

f = [y(3), y(4), f1, f2]';

H = [0, 0, -1, 1]';

%% Torque controller
controller_param = p.controller_param;
[T, e] = torqueController(t, y, controller_param);

%% Derivative
dydt = N\(H*T + f);
dydt(5) = e;
end