function [T, e] = PDGCTorqueController(t, y, ode_param)
%%% PD gravity compensation torque controller returns the T on the motor
%%% (not the wheel)
% for each timestep based on state and controller parameters

slope_angle = ode_param.slope_angle;

% unpack robot parameters
robot_param = ode_param.robot_param;
L = robot_param.L;
M = robot_param.M;
Iw = robot_param.Iw;
n = robot_param.n;
l_t = robot_param.l_t;
l = robot_param.l;
m = robot_param.m;
It = robot_param.It;

% unpack controller parameters
controller_param = ode_param.controller_param;
phi_desired = controller_param.phi_desired; % radians
kp = controller_param.kp;
ki = controller_param.ki;
kd = controller_param.kd;
max_torque = controller_param.max_torque;
control_max_integral = controller_param.control_max_integral;

gear_ratio = controller_param.gear_ratio;

% get variables from state and calculate error
phi = wrapTo2Pi(y(2)); % changed from wrapToPi to wrapto2Pi

% compute error, rate of change of error and error buildup
e = phi_desired - phi;
dedt = -y(4);
% e_sum = y(5);

% wrap e around (-pi, pi]
if e > pi
    e = e - 2*pi;
elseif e < -pi
    e = e + 2*pi;
end

% clamp e_sum between -control_max_integral and control_max_integral
% e_sum = min(control_max_integral, max(e_sum, -control_max_integral));

% compute torque
T = kp*e + kd*dedt; % + ki*e_sum % at the motor
T = T*gear_ratio; % at the wheel

% compute gravity compensation term
gc = m*9.81*l*sin(phi + slope_angle);

T = T - gc;

T = -T;

% clamp e_sum between -max_torque and max_torque
% T = min(max_torque, max(T, -max_torque));

end