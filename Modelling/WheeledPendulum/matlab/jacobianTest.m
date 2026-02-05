
T = 0;

L = 300e-3; % total length of the rod in m
l = L/2; % length from pivot to center of mass, m
m = 500e-3; % torso mass in kg
I_r = 1/12*m*L^2; % for a slender rod about the rotation axis passing through the CoM
% I_r = 100;
r = 100e-3; % wheel radius in m
M = 250e-3; % wheel mass in kg
I_w = 1/2*M*r^2; % for a solid disc of mass M and radius r; could be different if wheel geometry is different

% gravitational acceleration;
g = 9.81;

syms x(t) theta(t) t


x_dot = diff(x);
theta_dot = diff(theta);

q = {x; theta; x_dot; theta_dot}

A = I_w/r + r*(M + m);
B = m*r*l*cos(q{2});
C = m*l*cos(q{2});
D = I_r + m*l^2;

M = [1, 0, 0, 0;
    0, 1, 0, 0;
    0, 0, -A, B;
    0, 0, -C, D];

H = [0; 0; 1; 1];

f1 = m*r*l*sin(q{2}) * q{4}^2;
f2 = m*g*l*sin(q{2});

f = [q{3}; 
    q{4}; 
    f1; 
    f2];

qdot = M\(H*T + f);
