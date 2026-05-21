%%% for plotting trends with torso angle
% get filtered data from each torso pitch
% get mean of last n steps from each run
% get mean step vel
% get mid-stance vel

cd 'C:\Users\ahmed\Torsobot\Modelling\torsobot_dynamics\matlab\plotting'

clc;
clear;
close all;

addpath("../");
parameters

analytical = load("../mat_files/analytical_baseline_simple_slope_0.mat");

%% Get files
pitch = 25:1:155;
test_slope = 0;

controller_name = "PDGC_controller";

data_cell=cell(1, length(pitch));

limit_cycle_vel_before_collision = zeros(1, length(pitch));
limit_cycle_mean_vel = zeros(1, length(pitch));
limit_cycle_pow = zeros(1, length(pitch));
limit_cycle_work = zeros(1, length(pitch));
limit_cycle_work_clipped = zeros(1, length(pitch));
percentage_threshold_idx = zeros(1, length(pitch));
% mid_stance_vel = zeros(1, length(pitch));

for i=1:length(pitch)
   file_name = controller_name + "_" + "slope" + num2str(test_slope) + "pitch" + num2str(pitch(i)) + "_sol_data";
    data_cell{i} = load("../mat_files/" + file_name);

    sol = data_cell{i}.sol;
    time = sol(1,:);
    torso_pitch = sol(3,:);
    torso_pitch_rate = sol(5,:);
    wheel_pos = sol(2,:);
    wheel_vel = sol(4,:);

    mot_pow = data_cell{i}.power;

    threshold = 0.5;
    wheel_pos_diff = diff(wheel_pos);
    pre_impact_idx = find(abs(wheel_pos_diff) > threshold);

    max_num_steps = 40;
    pre_impact_idx = pre_impact_idx(1:max_num_steps+1);

    time = time(pre_impact_idx(1):pre_impact_idx(end));
    time = time - time(1);
    torso_pitch = torso_pitch(pre_impact_idx(1):pre_impact_idx(end)); 
    torso_pitch_rate = torso_pitch_rate(pre_impact_idx(1):pre_impact_idx(end));
    wheel_pos = wheel_pos(pre_impact_idx(1):pre_impact_idx(end));
    wheel_vel = wheel_vel(pre_impact_idx(1):pre_impact_idx(end));
    mot_pow = mot_pow(pre_impact_idx(1):pre_impact_idx(end));

    mot_pow_clipped = clip(mot_pow, 0, inf);
    min(mot_pow)

    pre_impact_idx = pre_impact_idx - pre_impact_idx(1) + 1;
    % wheel_vel(2871)

    mean_step_vel = zeros(1, max_num_steps);
    step_vel_before_collision = zeros(1, max_num_steps);
    mean_step_pow = zeros(1, max_num_steps);
    mean_step_COT = zeros(1, max_num_steps);
    mean_step_work = zeros(1, max_num_steps);
    mean_step_work_clipped = zeros(1, max_num_steps);

    for j = 1:max_num_steps
         pre_collision_idx = pre_impact_idx(j);
        idx_start = pre_impact_idx(j)+1; % right after collision
        idx_end   = pre_impact_idx(j+1);
    
        step_time = time(idx_end) - time(idx_start);
        
        step_vel_before_collision(j) = wheel_vel(pre_collision_idx);
        mean_step_vel(j) = trapz(time(idx_start:idx_end), wheel_vel(idx_start:idx_end))/step_time;
        
        
        mean_step_work(j) = trapz(time(idx_start:idx_end), mot_pow(idx_start:idx_end));
        mean_step_pow(j) = mean_step_work(j)/step_time;

        mean_step_work_clipped(j) = trapz(time(idx_start:idx_end), mot_pow_clipped(idx_start:idx_end));

    end

    save_steps = 2;
    limit_cycle_vel_before_collision(i) = mean(step_vel_before_collision(end-save_steps:end));
    limit_cycle_mean_vel(i) = mean(mean_step_vel(end-save_steps:end));
    limit_cycle_pow(i) = mean(mean_step_pow(end-save_steps:end));
    limit_cycle_work(i) = mean(mean_step_work(end-save_steps:end));
    limit_cycle_work_clipped(i) = mean(mean_step_work_clipped(end-save_steps:end));

    % get index where limit cycle is reached
    mean_step_vel_diff = diff(mean_step_vel);
    percentage_diff = mean_step_vel_diff./mean_step_vel(1:end-1)*100;

    percentage_threshold = 0.1; 

    [~, percentage_threshold_idx(i)] = min(abs(abs(percentage_diff) - percentage_threshold));

end

% get limit cycle COT
step_length = sqrt(2*L^2 - 2*L*L*cos(2*pi/n));
gravity_work = ((m+M)*g*step_length)

limit_cycle_work_diff = limit_cycle_work_clipped - limit_cycle_work;

limit_cycle_COT = limit_cycle_work/gravity_work;
limit_cycle_COT_clipped = limit_cycle_work_clipped/gravity_work;
limit_cycle_COT_diff = limit_cycle_work_diff/gravity_work;

% get where max occurs
[max_vel_before_collision, max_vel_before_collision_idx] = max(limit_cycle_vel_before_collision)
max_vel_before_collision_pitch = pitch(max_vel_before_collision_idx)

[max_vel, max_vel_idx] = max(limit_cycle_mean_vel)
max_vel_pitch = pitch(max_vel_idx)

[max_work, max_work_idx] = max(limit_cycle_work)
max_work_pitch = pitch(max_work_idx)

[max_work_clipped, max_work_clipped_idx] = max(limit_cycle_work_clipped)
max_work_clipped_pitch = pitch(max_work_clipped_idx)

[max_work_diff, max_work_diff_idx] = max(limit_cycle_work_diff)
max_work_diff_pitch = pitch(max_work_diff_idx)

[max_COT, max_COT_idx] = max(limit_cycle_COT)
max_COT_pitch = pitch(max_COT_idx)

[max_COT_clipped, max_COT_clipped_idx] = max(limit_cycle_COT_clipped)
max_COT_clipped_pitch = pitch(max_COT_clipped_idx)

[max_COT_diff, max_COT_diff_idx] = max(limit_cycle_COT_diff)
max_COT_diff_pitch = pitch(max_COT_diff_idx)

% %% median filter
% mean_step_vel_filt = medfilt1(mean_step_vel , 3);
% mid_stance_vel_filt = medfilt1(mid_stance_vel , 3);
% mean_step_power_filt = medfilt1(mean_step_power , 3);

%% curve fit for velocity
syms phi v

% mean vel
fit_func = @(p, pitch) p(1) .* sind(p(2) .* pitch + p(3)) + p(4);
p0 = [1.1, 1.5, 0, 1];

options = optimoptions('lsqcurvefit', 'Display', 'iter', 'FunctionTolerance', 1e-6, 'MaxIterations',10000);
optimal_params = lsqcurvefit(fit_func, p0, pitch, limit_cycle_mean_vel, [], [], options);

A = optimal_params(1);
B = optimal_params(2);
C = optimal_params(3);
D = optimal_params(4);

mean_step_vel_fit = fit_func(optimal_params, phi);

save("./mat_files/mean_step_vel_func", "phi", "v", "mean_step_vel_fit");

% get inverse function; get theta from vel
theta_func = solve(mean_step_vel_fit == v, phi);
theta_func = theta_func(1);
double(subs(theta_func, v, 2.0));

%% save values
save_filename = "./mat_files/" + "trends_across_simulated_slope_" + test_slope +".mat";

save(save_filename, "pitch", "limit_cycle_mean_vel", "limit_cycle_work_clipped")

%% colors
color_torso_pos = [0.00, 0.45, 0.74];  % Dark Blue
color_torso_vel = [0.30, 0.75, 0.93];  % Light Blue
color_wheel_pos = [0.85, 0.33, 0.10];  % Dark Orange/Rust
color_wheel_vel = [0.93, 0.69, 0.13];  % Golden Orange
color_target    = [0.80, 0.20, 0.20];  % Muted Red for target ylines

color_vel_pre   = [0.64, 0.08, 0.18];  % Deep Crimson for pre-collision velocity

color_actuator_COT = [0.49, 0.18, 0.56];  % Plum/Purple (Clipped Work / COT)
color_work_net     = [0.47, 0.67, 0.19];  % Forest Green (Net Unclipped Work)
color_work_neg     = [0.60, 0.60, 0.60];  % Neutral Grey (Negative Work)

%% plot
fig1 = figure(1);
tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile
plot(pitch, limit_cycle_mean_vel, "-",'Color', color_wheel_vel, 'LineWidth', 2);
hold on;
% plot(pitch, limit_cycle_vel_before_collision, "-",'Color', color_vel_pre, 'LineWidth', 2);
ylabel("Limit Cycle Wheel Velocity (rad/s)")
xlabel("Target Torso Pitch angle (deg)")
ylim([-inf, max(limit_cycle_mean_vel)+0.1])
xlim([pitch(1), pitch(end)]);

current_ylim = get(gca, 'YLim')

grid on

xline(90, ":", 'Color', [0.3, 0.3, 0.3, 0.5], 'LineWidth', 1.25)
plot([max_vel_pitch, max_vel_pitch], [0, max_vel], "-.",'Color', color_wheel_vel, 'LineWidth', 1.5)
% plot([max_vel_before_collision_pitch, max_vel_before_collision_pitch], [0, max_vel_before_collision], "-.",'Color', color_vel_pre, 'LineWidth', 1.5)

% legend("Mean Velocity", ...
%     "Pre-collision Velocity",...
%     'Location', "southeast");

text(0.02, 0.80, '(a)', 'Units', 'normalized', ...
    'FontSize', 10, 'FontName', 'Helvetica', ...
    'BackgroundColor', 'w', 'EdgeColor', 'none'); % 'w' = white background, 'k' = black border

hold off;

nexttile
plot(pitch, limit_cycle_work, "-",'Color', color_work_net, 'LineWidth', 2);
hold on;
plot(pitch, limit_cycle_work_clipped, "-",'Color', color_actuator_COT, 'LineWidth', 2);
plot(pitch, limit_cycle_work_diff, "-",'Color', color_work_neg, 'LineWidth', 2);
ylabel("Limit Cycle Work (J)")
ylim auto
xlim([pitch(1), pitch(end)]);
grid on

current_ylim = get(gca, 'YLim');

xline(90, ":", 'Color', [0.3, 0.3, 0.3, 0.5], 'LineWidth', 1.25)
plot([max_work_pitch, max_work_pitch], [current_ylim(1), max_work], "-.",'Color', color_work_net, 'LineWidth', 1.5)
plot([max_work_clipped_pitch, max_work_clipped_pitch], [current_ylim(1), max_work_clipped], "-.",'Color', color_actuator_COT, 'LineWidth', 1.5)
plot([max_work_diff_pitch, max_work_diff_pitch], [current_ylim(1), max_work_diff], "-.",'Color', color_work_neg, 'LineWidth', 1.5)

% xline(max_COT_pitch, "-.",'Color', color_mean_cot, 'LineWidth', 1.5)
% xline(max_COT_clipped_pitch, "-.",'Color', color_mean_cot_clipped, 'LineWidth', 1.5)
% xline(max_COT_diff_pitch, "-.",'Color', color_mean_cot_negative, 'LineWidth', 1.5)


legend("Unclipped", ...
    "Clipped",...
    "Negative",...
    'Location', "southeast");

text(0.02, 0.80, '(b)', 'Units', 'normalized', ...
    'FontSize', 10, 'FontName', 'Helvetica', ...
    'BackgroundColor', 'w', 'EdgeColor', 'none'); % 'w' = white background, 'k' = black border

hold off;

xlabel("Target Torso Pitch angle (deg)")
format_my_plot(12, 6);

export_filename = "../matlab_plots/limit_cycle_against_pitch_slope_" + num2str(test_slope) + ".png";
exportgraphics(gcf, export_filename, 'Resolution', 600);


%% comparison between predicted from analysis and simulated

fig2 = figure(2);
tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile
plot(analytical.phi_des_deg, analytical.theta_dot_before_simple, "-",'Color', color_vel_pre, 'LineWidth', 2);
hold on;
plot(pitch, limit_cycle_vel_before_collision, "--",'Color', color_vel_pre, 'LineWidth', 2);
ylabel("Limit Cycle Pre-collision Wheel Velocity (rad/s)")
ylim([-inf, max([limit_cycle_vel_before_collision, analytical.theta_dot_before_simple])+0.1])
xlim([pitch(1), pitch(end)]);

current_ylim = get(gca, 'YLim')

grid on

xline(90, ":", 'Color', [0.3, 0.3, 0.3, 0.5], 'LineWidth', 1.25)
plot([analytical.max_vel_phi_abs_simple, analytical.max_vel_phi_abs_simple], [0, analytical.max_vel], "-",'Color', color_vel_pre, 'LineWidth', 1.5)
plot([max_vel_before_collision_pitch, max_vel_before_collision_pitch], [0, max_vel_before_collision], "-.",'Color', color_vel_pre, 'LineWidth', 1.5)

legend("Predicted", ...
    "Simulated",...
    'Location', "southeast");

text(0.02, 0.80, '(a)', 'Units', 'normalized', ...
    'FontSize', 10, 'FontName', 'Helvetica', ...
    'BackgroundColor', 'w', 'EdgeColor', 'none'); % 'w' = white background, 'k' = black border

hold off;

nexttile
plot(analytical.phi_des_deg, analytical.COT_approx_simple*gravity_work, "-",'Color', color_work_net, 'LineWidth', 2);
hold on;
plot(pitch, limit_cycle_work, "--",'Color', color_work_net, 'LineWidth', 2);
plot(pitch, limit_cycle_work_clipped, "--",'Color', color_actuator_COT, 'LineWidth', 2);
plot(pitch, limit_cycle_work_diff, "--",'Color', color_work_neg, 'LineWidth', 2);
ylabel("Limit Cycle Work (J)")
ylim auto
xlim([pitch(1), pitch(end)]);
grid on

current_ylim = get(gca, 'YLim');

xline(90, ":", 'Color', [0.3, 0.3, 0.3, 0.5], 'LineWidth', 1.25)
plot([analytical.max_COT_phi_abs_simple, analytical.max_COT_phi_abs_simple], [current_ylim(1), analytical.max_COT*gravity_work], "-",'Color', color_work_net, 'LineWidth', 1.5)
plot([max_work_pitch, max_work_pitch], [current_ylim(1), max_work], "-.",'Color', color_work_net, 'LineWidth', 1.5)
plot([max_work_clipped_pitch, max_work_clipped_pitch], [current_ylim(1), max_work_clipped], "-.",'Color', color_actuator_COT, 'LineWidth', 1.5)
plot([max_work_diff_pitch, max_work_diff_pitch], [current_ylim(1), max_work_diff], "-.",'Color', color_work_neg, 'LineWidth', 1.5)

% xline(max_COT_pitch, "-.",'Color', color_mean_cot, 'LineWidth', 1.5)
% xline(max_COT_clipped_pitch, "-.",'Color', color_mean_cot_clipped, 'LineWidth', 1.5)
% xline(max_COT_diff_pitch, "-.",'Color', color_mean_cot_negative, 'LineWidth', 1.5)


legend("Predicted - Unclipped", ...
    "Simulated - Unclipped", ...
    "Simulated - Clipped",...
    "Simulated - Negative",...
    'Location', "southeast");

text(0.02, 0.80, '(b)', 'Units', 'normalized', ...
    'FontSize', 10, 'FontName', 'Helvetica', ...
    'BackgroundColor', 'w', 'EdgeColor', 'none'); % 'w' = white background, 'k' = black border

hold off;

xlabel("Target Torso Pitch angle (deg)")
format_my_plot(6.5, 7.5);

export_filename = "../matlab_plots/comparison_limit_cycle_against_pitch_slope_" + num2str(test_slope) + ".png";
exportgraphics(gcf, export_filename, 'Resolution', 600);