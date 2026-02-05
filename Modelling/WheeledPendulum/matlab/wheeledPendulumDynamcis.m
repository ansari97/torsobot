function qdot = wheeledPendulumDynamcis(t, q, param, K)
% diffFun is the system of differential equations describing the wheeled
% pendulum
%

% Full state feedback controller
T = -K*q;

% Unpack parameters
l = param(1);
m = param(2);
I_r = param(3);
r = param(4);
M = param(5);
I_w = param(6);

% gravitational acceleration;
g = 9.81;

A = I_w/r + r*(M + m);
B = m*r*l*cos(q(2));
C = m*l*cos(q(2));
D = I_r + m*l^2;

M = [1, 0, 0, 0;
    0, 1, 0, 0;
    0, 0, -A, B;
    0, 0, -C, D];

f1 = m*r*l*sin(q(2)) * q(4)^2;
f2 = m*g*l*sin(q(2));

f = [q(3); 
    q(4); 
    f1; 
    f2];

H = [0; 0; 1; -1];

qdot = M\(H*T + f);

end
