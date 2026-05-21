clc
clear

close all

cd 'C:\Users\ahmed\Torsobot\Modelling\torsobot_dynamics\matlab'
addpath 'C:\Users\ahmed\Torsobot\Modelling\torsobot_dynamics\matlab\plotting'

parameters;

phi_abs = 25:0.25:155
slope_angle = 0;


phi_des_deg = phi_abs - slope_angle;
phi_des = deg2rad(phi_des_deg);

slope_angle = deg2rad(slope_angle);

theta_c = pi/n;

grav = g;

step_length = sqrt(2*L^2 - 2*L*L*cos(2*pi/n));
COT_denominator = ((m+M)*grav*step_length);

%% Simplified analyses assuming that torso has zero vel post-collision
h = Iw + (m+M)*L^2 + m*l*L*cos(theta_c + phi_des);
j = Iw + (m+M)*L^2*cos(2*theta_c) + m*l*L*cos(theta_c - phi_des);

f = j./h;

actuator_work = 2*theta_c*m*grav*l*sin(phi_des + slope_angle);
grav_work = (m+M)*grav*step_length*sin(slope_angle);
total_work = actuator_work + grav_work;

A = 0.5*(M*L^2 + Iw);
B = 0.5*(m*L^2);

C = A+B;

theta_dot_before_simple = sqrt(total_work./(C*(1-f.^2)));
% 
% plot(phi_des, theta_dot_before)

% max
[max_vel, max_vel_idx] = max(theta_dot_before_simple)
max_vel_phi_abs_simple = phi_abs(max_vel_idx)

theta_dot_before_simple(1)
theta_dot_before_simple(end)

% torque and ground impulse calculation
theta_before = pi/10;
theta_after = -theta_before;

actuator_impulse_work = zeros(1, length(theta_dot_before_simple));
ground_impulse_work = zeros(1, length(theta_dot_before_simple));

for i=1:length(theta_dot_before_simple)
    theta_dot_before = theta_dot_before_simple(i);
    theta_dot_after = f(i)*theta_dot_before;
    
    v_torso_after = L*theta_dot_after*[cos(theta_after), 0, -sin(theta_after)];
    v_torso_before = L*theta_dot_before*[cos(theta_before),0,  -sin(theta_before)];
    
    r_torso = l*[sin(phi_des(i)), 0, cos(phi_des(i))];
    
    %angular momentum
    torso_ang_momentum_after = cross(r_torso, m*v_torso_after);
    torso_ang_momentum_before = cross(r_torso, m*v_torso_before);
    
    torque_impulse = torso_ang_momentum_after(2) - torso_ang_momentum_before(2);
    actuator_impulse_work(i) = -torque_impulse/2 * (theta_dot_after+theta_dot_before);

    % linear momentum
    v_wheel_before = L*theta_dot_before*[cos(theta_before), 0, -sin(theta_before)];
    v_wheel_after = L*theta_dot_after*[cos(theta_after), 0, -sin(theta_after)];

    linear_momentum_before = m*(v_torso_before) + M*(v_wheel_before);
    linear_momentum_after = m*(v_torso_after) + M*(v_wheel_after);

    ground_impulse = linear_momentum_after - linear_momentum_before;

    v_spoke_before = [0,0,-step_length*theta_dot_before];
    ground_impulse_work(i) = dot(ground_impulse, v_spoke_before/2);


end

min(actuator_impulse_work);
min(ground_impulse_work);

KE_lost = theta_dot_before_simple.^2.*C.*(1-f.^2);

all(abs(actuator_impulse_work + ground_impulse_work + KE_lost) < 1e-5)

[min_ground_impulse, min_ground_impulse_idx] = min(ground_impulse_work);
min_ground_impulse_phi_des_deg = phi_des_deg(min_ground_impulse_idx)

[max_ground_impulse, max_ground_impulse_idx] = max(ground_impulse_work);
max_ground_impulse_phi_des_deg = phi_des_deg(max_ground_impulse_idx)

[min_KE_lost, min_KE_lost_idx] = min(KE_lost);
min_KE_lost_phi_des_deg = phi_des_deg(min_KE_lost_idx)

[max_KE_lost, max_KE_lost_idx] = max(KE_lost);
max_KE_lost_phi_des_deg = phi_des_deg(max_KE_lost_idx)


% COT analysis
actuator_total_work = actuator_impulse_work + actuator_work;
COT_approx_simple = actuator_total_work/COT_denominator;

% hold on
% xline(pi/2)
% plot(phi_des, COT_approx)

% max
[max_COT, max_COT_idx] = max(COT_approx_simple)
max_COT_phi_abs_simple = phi_abs(max_COT_idx)

COT_approx_simple(1)
COT_approx_simple(end)

filename = "analytical_baseline_simple_slope_" + num2str(rad2deg(slope_angle))
save("./mat_files/" + filename + ".mat", 'phi_des_deg', 'theta_dot_before_simple', 'COT_approx_simple', 'max_COT', 'max_vel', 'max_COT_phi_abs_simple', 'max_vel_phi_abs_simple');


%% Analyses assuming torso has some non-zero vel post-collision
% 3 eqns, 3 unknowns, energy eqn is non-linear

P = It + l^2*m + m*l*L*cos(theta_c + phi_des);
Q = Iw + L^2*M + L^2*m + m*l*L*cos(theta_c + phi_des);
R = m*l*L*cos(theta_c - phi_des);
S = l^2*m + It;
W = m*l*L*cos(theta_c + phi_des);

G = (Iw + L^2*(m + M)*cos(2*theta_c) + m*l*L*cos(theta_c - phi_des));

a = (P.*R - S.*G)./(P.*W - S.*Q);
b = (W.*G - R.*Q)./(P.*W - S.*Q);

A = 0.5*(M*L^2 + Iw);
B = 0.5*(m*L^2);
C = A*a.^2;
D = B*a.^2;
E = 0.5*(m*l^2 + It)*b.^2;
F = m*l*L*a.*b.*cos(theta_c-phi_des);

coeff = A + B - C - (D + E + F);

theta_dot_before_complex = sqrt(total_work./coeff);

% max
[max_vel, max_vel_idx] = max(theta_dot_before_complex)
max_vel_phi_abs_complex = phi_abs(max_vel_idx)

theta_dot_before_complex(1)
theta_dot_before_complex(end)

theta_dot_after = a.*theta_dot_before_complex
phi_dot_after = b.*theta_dot_before_complex

rel_vel_after = theta_dot_after - phi_dot_after

% COT analysis
E_loss = total_work;



COT_approx_complex = E_loss/COT_denominator;

% max
[max_COT, max_COT_idx] = max(COT_approx_complex)
max_COT_phi_abs_complex = phi_abs(max_COT_idx)

% COT_approx(1)
% COT_approx(end)

filename = "analytical_baseline_complex_slope_" + num2str(rad2deg(slope_angle))
save("./mat_files/" + filename + ".mat", 'phi_des_deg', 'theta_dot_before_complex', 'COT_approx_complex', 'max_COT', 'max_vel', 'max_COT_phi_abs_complex', 'max_vel_phi_abs_complex');

%% plotting

fig1 = figure(1);

color_mean_vel   = [0.00, 0.45, 0.74];  % Dark Blue
color_mean_vel_before = [0.64, 0.08, 0.18];  % Deep Maroon/Red
color_mean_cot   = [0.47, 0.67, 0.19];  % Forest Green
color_mean_cot_clipped   = [0.85, 0.33, 0.10];  % Forest Green
color_mean_cot_negative = [0.49, 0.18, 0.56];  % Plum/Purple

tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile
plot(phi_abs, theta_dot_before_simple, "-",'Color', color_mean_vel, 'LineWidth', 2);
hold on;
plot(phi_abs, theta_dot_before_complex, "-",'Color', color_mean_vel_before, 'LineWidth', 2);
ylabel("Limit Cycle Wheel Velocity before Collision (rad/s)")
ylim([-inf, max([theta_dot_before_simple, theta_dot_before_complex])+0.1])
xlim([phi_abs(1), phi_abs(end)]); 

xline(90, ":", 'Color', [0.3, 0.3, 0.3, 0.5], 'LineWidth', 1.25)
xline(max_vel_phi_abs_simple, "--",'Color', color_mean_vel, 'LineWidth', 2)
xline(max_vel_phi_abs_complex, "--",'Color', color_mean_vel_before, 'LineWidth', 2)

legend("simple", ...
    "complex",...
    'Location', "best");

text(0.02, 0.80, '(a)', 'Units', 'normalized', ...
    'FontSize', 10, 'FontName', 'Helvetica', ...
    'BackgroundColor', 'w', 'EdgeColor', 'none'); % 'w' = white background, 'k' = black border

hold off;

nexttile
plot(phi_abs, COT_approx_simple, "-",'Color', color_mean_cot, 'LineWidth', 2);
hold on;
plot(phi_abs, COT_approx_complex, "-",'Color', color_mean_cot_clipped, 'LineWidth', 2);
ylabel("Limit Cycle COT")
ylim auto
xlim([phi_abs(1), phi_abs(end)]); 

xline(90, ":", 'Color', [0.3, 0.3, 0.3, 0.5], 'LineWidth', 1.25)
xline(max_COT_phi_abs_simple, "--",'Color', color_mean_cot, 'LineWidth', 2)

legend("simple ", ...
    "complex",...
    'Location', "best");

text(0.02, 0.80, '(b)', 'Units', 'normalized', ...
    'FontSize', 10, 'FontName', 'Helvetica', ...
    'BackgroundColor', 'w', 'EdgeColor', 'none'); % 'w' = white background, 'k' = black border

hold off;

xlabel("Torso Pitch (deg)")
format_my_plot(6.5, 7.5);

% export_filename = "./matlab_plots/limit_cycle_pred_against_pitch_slope_" + num2str(rad2deg(slope_angle)) + ".png";
% exportgraphics(gcf, export_filename, 'Resolution', 600);

% fig2 = figure(2);
% 
% plot(phi_abs, rel_vel_after, "-", 'Color', color_mean_cot_negative, 'LineWidth', 2); hold on;
% ylim auto
% xlim([phi_abs(1), phi_abs(end)]); 
% grid on
% 
% ylabel("Relative Post-Collision Wheel Velocity (rad/s)")
% xlabel("Torso Pitch (deg)")
% 
% hold off;
% format_my_plot(6.5, 7.5);
% 
% export_filename = "./matlab_plots/wheel_rel_vel_against_pitch_slope_" + num2str(rad2deg(slope_angle)) + ".png";
% exportgraphics(gcf, export_filename, 'Resolution', 600);
% 
% 

% plot(phi_abs, phi_dot_after, "-"); hold off;

figure;

plot(phi_des_deg, ground_impulse_work);





