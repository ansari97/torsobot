function [T, e] = PDTorqueController(t, y, controller_param)
%%% torqueController returns the T for each timestep based on state and
%%% controller parameters

% unpack controller parameters
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
T = kp*e + kd*dedt; % + ki*e_sum 
% T = -T; % for positive e, we need a negative torque; the +torque on torso
% acts in clcwise direction; commented out; negative outside of this
% function

% torque is output at the wheel
% T = T*gear_ratio;

% clamp e_sum between -max_torque and max_torque
% T = min(max_torque, max(T, -max_torque));

end