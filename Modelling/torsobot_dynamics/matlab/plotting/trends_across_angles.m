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

%% Get files
pitch = [30 40 45 50 60 65 70 75 80 85 90 95 100 105 110 115 120 130 140 150];
data_cell=cell(1, length(pitch));

mean_step_vel = zeros(1, length(pitch));
mean_step_work = zeros(1, length(pitch));
mean_step_work_clipped = zeros(1, length(pitch));
precol_step_vel = zeros(1, length(pitch));

for i=1:length(pitch)
    file_name = pitch(i) + "_end_steps_data.mat";
    data_cell{i} = load("./mat_files/physical/" + file_name);

    mean_step_vel(i) = data_cell{i}.mean_step_vel_save;
    mean_step_work(i) = data_cell{i}.mean_step_work_save;
    mean_step_work_clipped(i) = data_cell{i}.mean_step_work_clipped_save;
    precol_step_vel(i) = data_cell{i}.precol_step_vel_save;
end

mean_step_work_neg = mean_step_work_clipped - mean_step_work;

% %% median filter
% mean_step_vel_filt = medfilt1(mean_step_vel , 3);
% mid_stance_vel_filt = medfilt1(mean_step_work , 3);
% mean_step_power_filt = medfilt1(mean_step_work_clipped , 3);

%% curve fit for velocity
syms phi

% mean vel
fit_func = @(p, pitch) p(1) .* sind(p(2) .* pitch + p(3)) + p(4);
p0 = [1.0, 1.0, 0.0, mean(mean_step_vel)];

options = optimoptions('lsqcurvefit', 'Display', 'iter', 'FunctionTolerance', 1e-6);
[optimal_params, resnorm] = lsqcurvefit(fit_func, p0, pitch, mean_step_vel, [], [], options);

A = optimal_params(1);
B = optimal_params(2);
C = optimal_params(3);
D = optimal_params(4);

vel_fit_max_pitch = (90-C)/B;
vel_max = fit_func(optimal_params, vel_fit_max_pitch);

mean_step_vel_fit = fit_func(optimal_params, phi);

SST = sum((mean_step_vel - mean(mean_step_vel)).^2);
R_squared_vel_fit = 1 - (resnorm / SST);

% unclipped work
fit_func = @(p, pitch) p(1) .* sind(p(2) .* pitch + p(3)) + p(4);
p0 = [1.0, 1.0, 0.0, mean(mean_step_work)];

options = optimoptions('lsqcurvefit', 'Display', 'iter', 'FunctionTolerance', 1e-6);
[optimal_params, resnorm] = lsqcurvefit(fit_func, p0, pitch, mean_step_work, [], [], options);

A = optimal_params(1);
B = optimal_params(2);
C = optimal_params(3);
D = optimal_params(4);

mean_step_work_fit = fit_func(optimal_params, phi);

SST = sum((mean_step_work - mean(mean_step_work)).^2);
R_squared_work_fit = 1 - (resnorm / SST);

% clipped work
fit_func = @(p, pitch) p(1) .* sind(p(2) .* pitch + p(3)) + p(4);
p0 = [1.0, 1.0, 0.0, mean(mean_step_work_clipped)];

options = optimoptions('lsqcurvefit', 'Display', 'iter', 'FunctionTolerance', 1e-6);
[optimal_params, resnorm] = lsqcurvefit(fit_func, p0, pitch, mean_step_work_clipped, [], [], options);

A = optimal_params(1);
B = optimal_params(2);
C = optimal_params(3);
D = optimal_params(4);

c_work_fit_max_pitch = (90-C)/B;
c_work_max = fit_func(optimal_params, c_work_fit_max_pitch);

mean_step_work_clipped_fit = fit_func(optimal_params, phi);

SST = sum((mean_step_work_clipped - mean(mean_step_work_clipped)).^2);
R_squared_work_clipped_fit = 1 - (resnorm / SST);


% negative work
fit_func = @(p, pitch) p(1) .* sind(p(2) .* pitch + p(3)) + p(4);
p0 = [1.0, 1.0, 0.0, mean(mean_step_work_neg)];

options = optimoptions('lsqcurvefit', 'Display', 'iter', 'FunctionTolerance', 1e-6);
[optimal_params, resnorm] = lsqcurvefit(fit_func, p0, pitch, mean_step_work_neg, [], [], options);

A = optimal_params(1);
B = optimal_params(2);
C = optimal_params(3);
D = optimal_params(4);

mean_step_work_neg_fit = fit_func(optimal_params, phi);

SST = sum((mean_step_work_neg - mean(mean_step_work_neg)).^2);
R_squared_work_neg_fit = 1 - (resnorm / SST);

%% plot
color_torso_pos = [0.00, 0.45, 0.74];  % Dark Blue
color_torso_vel = [0.30, 0.75, 0.93];  % Light Blue
color_wheel_pos = [0.85, 0.33, 0.10];  % Dark Orange/Rust
color_wheel_vel = [0.93, 0.69, 0.13];  % Golden Orange
color_target    = [0.80, 0.20, 0.20];  % Muted Red for target ylines

color_vel_pre   = [0.64, 0.08, 0.18];  % Deep Crimson for pre-collision velocity

color_actuator_COT = [0.49, 0.18, 0.56];  % Plum/Purple (Clipped Work / COT)
color_work_net     = [0.47, 0.67, 0.19];  % Forest Green (Net Unclipped Work)
color_work_neg     = [0.60, 0.60, 0.60];  % Neutral Grey (Negative Work)

% plot in deg

fig1 = figure(1);
tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(pitch,mean_step_vel, "o", 'Color', color_wheel_vel, 'LineWidth', 2); 
hold on;
fplot(phi, mean_step_vel_fit, [pitch(1), pitch(end)],'--', 'Color', color_wheel_vel, 'LineWidth', 2); 
% plot(pitch, precol_step_vel, '-o', 'Color', color_vel_pre, 'LineWidth', 2); 

xlim([pitch(1), pitch(end)]);
ylim auto;
% ylim([0.80*min(torso_pitch), 1.20*max(torso_pitch)])

xlabel("Torso Pitch (deg)");
ylabel('Limit Cycle Wheel Velocity (rad/s)');

legend('Mean Velocity Data', 'Curve Fit', 'Location', 'best');

grid on;

text(0.02, 0.85, '(a)', 'Units', 'normalized', ...
    'FontSize', 10, 'FontName', 'Helvetica', ...
    'BackgroundColor', 'w', 'EdgeColor', 'none'); % 'w' = white background, 'k' = black border

hold off;

nexttile;
plot(pitch, mean_step_work, 'o', 'Color', color_work_net, 'LineWidth', 2); 
hold on;
plot(pitch, mean_step_work_clipped, 'o', 'Color', color_actuator_COT, 'LineWidth', 2); 
plot(pitch, mean_step_work_neg, 'o', 'Color', color_work_neg, 'LineWidth', 2); 

plot(NaN, NaN, '--k', 'LineWidth', 2); 

fplot(phi, mean_step_work_fit, [pitch(1), pitch(end)],'--', 'Color', color_work_net, 'LineWidth', 2); 
fplot(phi, mean_step_work_clipped_fit, [pitch(1), pitch(end)],'--', 'Color', color_actuator_COT, 'LineWidth', 2); 
fplot(phi, mean_step_work_neg_fit, [pitch(1), pitch(end)],'--', 'Color', color_work_neg, 'LineWidth', 2); 

xlim([pitch(1), pitch(end)]);
ylim auto;
% ylim([0.80*min(wheel_vel), 1.20*max(wheel_vel)])

ylabel("Limit Cycle Work (J)");
xlabel("Torso Pitch (deg)");

legend('Unclipped', 'Cipped', 'Negative', 'Fits', 'Location', 'best');
grid on; 

text(0.02, 0.85, '(b)', 'Units', 'normalized', ...
    'FontSize', 10, 'FontName', 'Helvetica', ...
    'BackgroundColor', 'w', 'EdgeColor', 'none'); % 'w' = white background, 'k' = black border

hold off;

format_my_plot(10, 5)

% save figures
filename = "Across_angles.png";

save_directory  = "..\matlab_plots\Physical\";
save_filename = save_directory + filename;

exportgraphics(gcf, save_filename, 'Resolution', 600);


%% comparison between simulations and experiments
sim = load("./mat_files/trends_across_simulated_slope_0.mat");
sim_pitch = sim.pitch;
sim_limit_cycle_mean_vel = sim.limit_cycle_mean_vel;
sim_limit_cycle_work_clipped = sim.limit_cycle_work_clipped;

[sim_max_vel, sim_max_vel_idx] = max(sim_limit_cycle_mean_vel);
sim_max_vel_pitch = sim_pitch(sim_max_vel_idx);

[sim_max_work, sim_max_work_idx] = max(sim_limit_cycle_work_clipped);
sim_max_work_pitch = sim_pitch(sim_max_work_idx);


%% comparison

fig2 = figure(2);

tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(pitch, mean_step_vel, "o", 'Color', color_wheel_vel, 'LineWidth', 2); 
hold on;
fplot(phi, mean_step_vel_fit, [pitch(1), pitch(end)],'--', 'Color', color_wheel_vel, 'LineWidth', 2); 
plot(sim_pitch, sim_limit_cycle_mean_vel, '-', 'Color', color_wheel_vel, 'LineWidth', 2); 

xline(90, ":", 'Color', [0.3, 0.3, 0.3, 0.5], 'LineWidth', 1.25)
plot([vel_fit_max_pitch, vel_fit_max_pitch], [0, vel_max], "-.",'Color', color_wheel_vel, 'LineWidth', 1.5)
plot([sim_max_vel_pitch, sim_max_vel_pitch], [0, sim_max_vel], "-",'Color', color_wheel_vel, 'LineWidth', 1.5)

xlim([pitch(1), pitch(end)]);
ylim auto;
% ylim([0.80*min(torso_pitch), 1.20*max(torso_pitch)])

xlabel("Torso Pitch (deg)");
ylabel('Limit Cycle Mean Wheel Velocity (rad/s)');

legend('Experiment Datapoints', 'Experiment Fit', 'Simulated',  'Location', 'best');

grid on;

text(0.02, 0.85, '(a)', 'Units', 'normalized', ...
    'FontSize', 10, 'FontName', 'Helvetica', ...
    'BackgroundColor', 'w', 'EdgeColor', 'none'); % 'w' = white background, 'k' = black border

hold off;

nexttile;
plot(pitch, mean_step_work_clipped, 'o', 'Color', color_actuator_COT, 'LineWidth', 2); 
hold on;
fplot(phi, mean_step_work_clipped_fit, [pitch(1), pitch(end)],'--', 'Color', color_actuator_COT, 'LineWidth', 2); 

plot(sim_pitch, sim_limit_cycle_work_clipped,'-', 'Color', color_actuator_COT, 'LineWidth', 2); 

xline(90, ":", 'Color', [0.3, 0.3, 0.3, 0.5], 'LineWidth', 1.25)
plot([c_work_fit_max_pitch, c_work_fit_max_pitch], [0, c_work_max], "-.",'Color', color_actuator_COT, 'LineWidth', 1.5)
plot([sim_max_work_pitch, sim_max_work_pitch], [0, sim_max_work], "-",'Color', color_actuator_COT, 'LineWidth', 1.5)

xlim([pitch(1), pitch(end)]);
ylim auto;
% ylim([0.80*min(wheel_vel), 1.20*max(wheel_vel)])

ylabel("Limit Cycle Work (J)");
xlabel("Torso Pitch (deg)");

legend('Experiment Datapoints', 'Experiment Fit', 'Simulated',  'Location', 'best');
grid on; 

text(0.02, 0.85, '(b)', 'Units', 'normalized', ...
    'FontSize', 10, 'FontName', 'Helvetica', ...
    'BackgroundColor', 'w', 'EdgeColor', 'none'); % 'w' = white background, 'k' = black border

hold off;

format_my_plot(10, 5)

% save figures
filename = "Across_angles_comparison_expvsim.png";

save_directory  = "..\matlab_plots\";
save_filename = save_directory + filename;

exportgraphics(gcf, save_filename, 'Resolution', 600);
