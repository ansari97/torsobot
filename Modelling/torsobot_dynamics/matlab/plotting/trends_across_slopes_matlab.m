%%% for plotting trends with slope angle

cd 'C:\Users\ahmed\Torsobot\Modelling\torsobot_dynamics\matlab\plotting'

clc;
clear;
close all;

addpath("../");
parameters

analytical = load("../mat_files/analytical_baseline_simple_pitch_30.mat");

%% Get files
abs_phi_desired = 30; % degrees from the vertical axis
test_slopes = 0:0.5:80;

data_cell=cell(1, length(test_slopes));

% when limit cycle has been reached
limit_cycle_vel_before_collision = zeros(1, length(test_slopes));
limit_cycle_mean_vel = zeros(1, length(test_slopes));
limit_cycle_pow = zeros(1, length(test_slopes));
limit_cycle_work = zeros(1, length(test_slopes));
limit_cycle_work_clipped = zeros(1, length(test_slopes));
percentage_threshold_idx = zeros(1, length(test_slopes));
% mid_stance_vel = zeros(1, length(test_slopes));

controller_name = "PDGC_controller";

for i=1:length(test_slopes)
    file_name = controller_name + "_" + "slope" + num2str(test_slopes(i)) + "pitch" + num2str(abs_phi_desired) + "_sol_data";
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
max_vel_before_collision_slope = test_slopes(max_vel_before_collision_idx)

[max_vel, max_vel_idx] = max(limit_cycle_mean_vel)
max_vel_slope = test_slopes(max_vel_idx)

[max_COT, max_COT_idx] = max(limit_cycle_COT)
max_COT_slope = test_slopes(max_COT_idx)

[max_COT_clipped, max_COT_clipped_idx] = max(limit_cycle_COT_clipped)
max_COT_clipped_slope = test_slopes(max_COT_clipped_idx)

[max_COT_diff, max_COT_diff_idx] = max(limit_cycle_COT_diff)
max_COT_diff_slope = test_slopes(max_COT_diff_idx)



% %% median filter
% mean_step_vel_filt = medfilt1(mean_step_vel , 3);
% mid_stance_vel_filt = medfilt1(mid_stance_vel , 3);
% mean_step_power_filt = medfilt1(mean_step_power , 3);

%% quadratic fit
% fit_coeff = polyfit(pitch, limit_cycle_vel, 2);
% 
% mean_step_vel_fit = polyval(fit_coeff, pitch);
% 
% syms theta v real
% mean_step_vel_func = fit_coeff(1)*theta^2 + fit_coeff(2)*theta + fit_coeff(3)
% 
% save("./mat_files/mean_step_vel_func", "theta", "v", "mean_step_vel_func");
% 
% % get inverse function; get theta from vel
% theta_func = solve(mean_step_vel_func == v, theta)
% theta_func = theta_func(1)
% double(subs(theta_func, v, 2.0))


%% plot

fig1 = figure(1);

color_mean_vel   = [0.00, 0.45, 0.74];  % Dark Blue
color_mean_vel_before = [0.64, 0.08, 0.18];  % Deep Maroon/Red
color_mean_cot   = [0.47, 0.67, 0.19];  % Forest Green
color_mean_cot_clipped   = [0.85, 0.33, 0.10];  % Forest Green
color_mean_cot_negative = [0.49, 0.18, 0.56];  % Plum/Purple

tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile
plot(test_slopes, limit_cycle_mean_vel, "-",'Color', color_mean_vel, 'LineWidth', 2);
hold on;
plot(test_slopes, limit_cycle_vel_before_collision, "-",'Color', color_mean_vel_before, 'LineWidth', 2);
ylabel("Limit Cycle Velocity (rad/s)")
ylim([-inf, max(limit_cycle_vel_before_collision)+0.1])
xlim([test_slopes(1), test_slopes(end)]);

current_ylim = get(gca, 'YLim')

xline(90, ":", 'Color', [0.3, 0.3, 0.3, 0.5], 'LineWidth', 1.25)
plot([max_vel_slope, max_vel_slope], [0, max_vel], "-.",'Color', color_mean_vel, 'LineWidth', 1.5)
plot([max_vel_before_collision_slope, max_vel_before_collision_slope], [0, max_vel_before_collision], "-.",'Color', color_mean_vel_before, 'LineWidth', 1.5)

legend("Mean Wheel Velocity", ...
    "Wheel Velocity before Collision",...
    'Location', "best");

text(0.02, 0.80, '(a)', 'Units', 'normalized', ...
    'FontSize', 10, 'FontName', 'Helvetica', ...
    'BackgroundColor', 'w', 'EdgeColor', 'none'); % 'w' = white background, 'k' = black border

hold off;

nexttile
plot(test_slopes, limit_cycle_COT, "-",'Color', color_mean_cot, 'LineWidth', 2);
hold on;
plot(test_slopes, limit_cycle_COT_clipped, "-",'Color', color_mean_cot_clipped, 'LineWidth', 2);
plot(test_slopes, limit_cycle_COT_diff, "-",'Color', color_mean_cot_negative, 'LineWidth', 2);
ylabel("Limit Cycle COT")
ylim auto
xlim([test_slopes(1), test_slopes(end)]);
grid on

current_ylim = get(gca, 'YLim');

xline(90, ":", 'Color', [0.3, 0.3, 0.3, 0.5], 'LineWidth', 1.25)
plot([max_COT_slope, max_COT_slope], [current_ylim(1), max_COT], "-.",'Color', color_mean_cot, 'LineWidth', 1.5)
plot([max_COT_clipped_slope, max_COT_clipped_slope], [current_ylim(1), max_COT_clipped], "-.",'Color', color_mean_cot_clipped, 'LineWidth', 1.5)
plot([max_COT_diff_slope, max_COT_diff_slope], [current_ylim(1), max_COT_diff], "-.",'Color', color_mean_cot_negative, 'LineWidth', 1.5)

% xline(max_COT_pitch, "-.",'Color', color_mean_cot, 'LineWidth', 1.5)
% xline(max_COT_clipped_pitch, "-.",'Color', color_mean_cot_clipped, 'LineWidth', 1.5)
% xline(max_COT_diff_pitch, "-.",'Color', color_mean_cot_negative, 'LineWidth', 1.5)


legend("Unclipped", ...
    "Clipped",...
    "Negative",...
    'Location', "best");

text(0.02, 0.80, '(b)', 'Units', 'normalized', ...
    'FontSize', 10, 'FontName', 'Helvetica', ...
    'BackgroundColor', 'w', 'EdgeColor', 'none'); % 'w' = white background, 'k' = black border

hold off;

xlabel("Slope angle (deg)")
format_my_plot(6.5, 7.5);

export_filename = "../matlab_plots/limit_cycle_against_slope_pitch_" + num2str(abs_phi_desired) + ".png";
exportgraphics(gcf, export_filename, 'Resolution', 600);

%% comparison
fig2 = figure(2);

color_mean_vel   = [0.00, 0.45, 0.74];  % Dark Blue
color_mean_vel_before = [0.64, 0.08, 0.18];  % Deep Maroon/Red
color_mean_cot   = [0.47, 0.67, 0.19];  % Forest Green
color_mean_cot_clipped   = [0.85, 0.33, 0.10];  % burnt Orange

tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile
plot(analytical.slope_angle_deg, analytical.theta_dot_before_simple, "-",'Color', color_mean_vel_before, 'LineWidth', 2);
hold on;
plot(test_slopes, limit_cycle_vel_before_collision, "--",'Color', color_mean_vel_before, 'LineWidth', 2);


ylabel("Limit Cycle Wheel Velocity before Collision (rad/s)")

ylim([-inf, max([limit_cycle_vel_before_collision, analytical.theta_dot_before_simple])+0.1])
xlim([test_slopes(1), test_slopes(end)]); 

current_ylim = get(gca, 'YLim');

xline(90, ":", 'Color', [0.3, 0.3, 0.3, 0.5], 'LineWidth', 1.25)
plot([max_vel_before_collision_slope, max_vel_before_collision_slope], [0, max_vel_before_collision], "-.",'Color', color_mean_vel_before, 'LineWidth', 1.5)
plot([analytical.max_vel_slope_simple, analytical.max_vel_slope_simple], [0, analytical.max_vel], "-",'Color', color_mean_vel_before, 'LineWidth', 1.5)

legend("Predicted", ...
    "Simulated",...
    'Location', "best");

text(0.02, 0.80, '(a)', 'Units', 'normalized', ...
    'FontSize', 10, 'FontName', 'Helvetica', ...
    'BackgroundColor', 'w', 'EdgeColor', 'none'); % 'w' = white background, 'k' = black border

hold off;

nexttile
plot(analytical.slope_angle_deg, analytical.COT_approx_simple, "-",'Color', color_mean_cot, 'LineWidth', 2);
hold on;
plot(test_slopes, limit_cycle_COT, "--",'Color', color_mean_cot, 'LineWidth', 2);
plot(test_slopes, limit_cycle_COT_clipped, "--",'Color', color_mean_cot_clipped, 'LineWidth', 2);
plot(test_slopes, limit_cycle_COT_diff, "--",'Color', color_mean_cot_negative, 'LineWidth', 2);


ylabel("Limit Cycle COT")
ylim auto
xlim([test_slopes(1), test_slopes(end)]);

xline(90, ":", 'Color', [0.3, 0.3, 0.3, 0.5], 'LineWidth', 1.25)
plot([max_COT_slope, max_COT_slope], [0, max_COT], "-.",'Color', color_mean_cot, 'LineWidth', 1.5)
plot([max_COT_clipped_slope, max_COT_clipped_slope], [0, max_COT_clipped], "-.",'Color', color_mean_cot_clipped, 'LineWidth', 1.5)
plot([analytical.max_COT_slope_simple, analytical.max_COT_slope_simple], [0, analytical.max_COT], "-",'Color', color_mean_cot, 'LineWidth', 1.5)
plot([max_COT_diff_slope, max_COT_diff_slope], [0, max_COT_diff], "-.",'Color', color_mean_cot_negative, 'LineWidth', 1.5)

legend("Predicted", ...
    "Simulated (Unclipped)",...
    "Simulated (Clipped)",...
    "Simulated (Negative)",...
    'Location', "southeast");

text(0.02, 0.80, '(b)', 'Units', 'normalized', ...
    'FontSize', 10, 'FontName', 'Helvetica', ...
    'BackgroundColor', 'w', 'EdgeColor', 'none'); % 'w' = white background, 'k' = black border

hold off;

xlabel("Slope Angle (deg)")
format_my_plot(6.5, 7.5);

export_filename = "../matlab_plots/limit_cycle_against_pitch_slope_" + num2str(slope_angle) + "_pred_vs_sim" + ".png";
exportgraphics(gcf, export_filename, 'Resolution', 600);

%% clipped vs not clipped power
% fig5 = figure(5);
% 
% plot(test_slopes, limit_cycle_COT_clipped, "-",'Color', color_mean_cot, 'LineWidth', 2);hold on;
% plot(test_slopes, limit_cycle_COT - limit_cycle_COT_clipped, "-",'Color', color_mean_vel, 'LineWidth', 2); 
% plot(test_slopes, limit_cycle_COT, "-",'Color', color_mean_power, 'LineWidth', 2); 
% 
% 
% % ylim([min([limit_cycle_COT, limit_cycle_COT_clipped]) - 0.051, min([limit_cycle_COT, limit_cycle_COT_clipped]) + 0.1]); 
% 
% xlabel("Downward Slope (deg)")
% ylabel("Limit Cycle Mean COT")
% 
% legend("Limit Cycle Mean COT (positive work)", ...
%     "Limit Cycle Mean COT (negative work)",...
%     "Limit Cycle Mean COT", ...
%     'Location', "best");
% 
% hold off;
% 
% format_my_plot(6.5);
% 
% export_filename = "../matlab_plots/limit_cycle_COT_againt_slope_combined_" + ".png";
% exportgraphics(gcf, export_filename, 'Resolution', 600);

%% limit cycle step index vs torso pitch
% fig3 = figure(3);
% 
% plot(test_slopes, percentage_threshold_idx, "-",'Color', color_mean_vel, 'LineWidth', 2);
% hold on;
% xlabel("Downward Slope (deg)")
% ylabel("Limit Cycle Step Index")
% xlim([test_slopes(1), test_slopes(end)]); 
% ylim([min(percentage_threshold_idx) - 1, max(percentage_threshold_idx) + 1]); 
% hold off;
% format_my_plot(6.5);
% 
% export_filename = "../matlab_plots/limit_cycle_step_idx_slope_" + ".png";
% exportgraphics(gcf, export_filename, 'Resolution', 600);


%% plot step data for a specific slope
% pitch = 45;
% 
% file_name = pitch + "_sol_data.mat";
% data_struct = load("../mat_files/" + file_name);
% 
% % Cost of transport
% % com length from the center of wheel
% 
% sol = data_struct.sol;
% time = sol(1,:);
% torso_pitch = sol(3,:);
% torso_pitch_rate = sol(5,:);
% wheel_pos = sol(2,:);
% wheel_vel = sol(4,:);
% 
% mot_pow = data_struct.power;
% 
% threshold = 0.5;
% wheel_pos_diff = diff(wheel_pos);
% pre_impact_idx = find(abs(wheel_pos_diff) > threshold);
% 
% max_num_steps = 30;
% pre_impact_idx = pre_impact_idx(1:max_num_steps+1);
% 
% time = time(pre_impact_idx(1):pre_impact_idx(end));
% time = time - time(1);
% torso_pitch = torso_pitch(pre_impact_idx(1):pre_impact_idx(end)); 
% torso_pitch_rate = torso_pitch_rate(pre_impact_idx(1):pre_impact_idx(end));
% wheel_pos = wheel_pos(pre_impact_idx(1):pre_impact_idx(end));
% wheel_vel = wheel_vel(pre_impact_idx(1):pre_impact_idx(end));
% mot_pow = mot_pow(pre_impact_idx(1):pre_impact_idx(end));
% 
% pre_impact_idx = pre_impact_idx - pre_impact_idx(1) + 1;
% % wheel_vel(2871)
% 
% state = sol(2:end, :);
% COM_len = (m*l + M*0)/(m+M);
% COM_vel = [L.*wheel_vel.*cos(wheel_pos) + COM_len.*torso_pitch_rate.*cos(torso_pitch); 
%     -(L.*wheel_vel.*sin(wheel_pos) + COM_len.*torso_pitch_rate.*sin(torso_pitch))];
% COM_speed = vecnorm(COM_vel);
% 
% % clamp motor power to always be positive or 0
% mot_pow = clip(mot_pow, 0, inf);
% 
% COT = mot_pow./((M+m)*g*COM_speed);
% 
% mean_step_vel = zeros(1, max_num_steps);
% mean_step_pow = zeros(1, max_num_steps);
% mean_step_COT = zeros(1, max_num_steps);
% mean_step_work = zeros(1, max_num_steps);
% step_duration = zeros(1, max_num_steps);
% 
% for j = 1:max_num_steps
%     j;
%     idx_start = pre_impact_idx(j);
%     idx_end   = pre_impact_idx(j+1);
% 
%     step_time = time(idx_end) - time(idx_start);
%     step_duration(j) = step_time;
% 
%     mean_step_vel(j) = trapz(time(idx_start:idx_end), wheel_vel(idx_start:idx_end))/step_time;
%     mean_step_pow(j) = trapz(time(idx_start:idx_end), mot_pow(idx_start:idx_end))/step_time;
% 
%     % mean_step_vel(j) = mean(wheel_vel(pre_impact_idx(j):pre_impact_idx(j+1))); 
%     % mean_step_pow(j) = mean(mot_pow(pre_impact_idx(j):pre_impact_idx(j+1))); 
% 
%     mean_step_work(j) = mean_step_pow(j) * step_time;
%     step_length = sqrt(2*L^2 - 2*L*L*cos(2*pi/n));
% 
%     % mean_step_COT(j) = mean(COT(pre_impact_idx(j):pre_impact_idx(j+1))); 
% 
%     mean_step_COT(j) = mean_step_work(j)/((m+M)*g*step_length);
% 
% end
% 
% step_idx = 1:max_num_steps;
% 
% 
% %% step plotting
% color_mean_vel   = [0.00, 0.45, 0.74];  % Dark Blue
% color_mean_power = [0.64, 0.08, 0.18];  % Deep Maroon/Red
% color_mean_cot   = [0.47, 0.67, 0.19];  % Forest Green
% 
% fig4 = figure(4);
% tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
% 
% nexttile;
% plot(step_idx, mean_step_vel, 'Color', color_mean_vel, 'LineWidth', 2);
% hold on;
% 
% xlim([1, inf]);
% 
% % xlabel('Step Index');
% ylabel('Mean Step Wheel Velocity (rad/s)');
% grid on
% 
% text(0.02, 0.80, '(a)', 'Units', 'normalized', ...
%     'FontSize', 10, 'FontName', 'Helvetica', ...
%     'BackgroundColor', 'w', 'EdgeColor', 'none'); % 'w' = white background, 'k' = black border
% 
% hold off;
% 
% nexttile;
% plot(step_idx, mean_step_pow, 'Color', color_mean_power, 'LineWidth', 2);
% hold on;
% 
% xlim([1, inf]);
% 
% % xlabel('Step Index');
% ylabel('Mean Step Power (W)');
% grid on
% 
% text(0.02, 0.80, '(b)', 'Units', 'normalized', ...
%     'FontSize', 10, 'FontName', 'Helvetica', ...
%     'BackgroundColor', 'w', 'EdgeColor', 'none'); % 'w' = white background, 'k' = black border
% 
% hold off;
% % 
% nexttile;
% plot(step_idx, mean_step_COT, 'Color', color_mean_cot, 'LineWidth', 2);
% hold on;
% 
% xlim([1, inf]);
% 
% xlabel('Step Index');
% ylabel('Mean Step COT');
% grid on
% 
% text(0.02, 0.80, '(c)', 'Units', 'normalized', ...
%     'FontSize', 10, 'FontName', 'Helvetica', ...
%     'BackgroundColor', 'w', 'EdgeColor', 'none'); % 'w' = white background, 'k' = black border
% 
% hold off;
% 
% % nexttile;
% % plot(step_idx, step_duration, 'Color', color_mean_cot, 'LineWidth', 2);
% % hold on;
% % 
% % xlim([1, inf]);
% % 
% % xlabel('Step Index');
% % ylabel('Step Duration');
% % grid on
% % 
% % text(0.02, 0.80, '(d)', 'Units', 'normalized', ...
% %     'FontSize', 10, 'FontName', 'Helvetica', ...
% %     'BackgroundColor', 'w', 'EdgeColor', 'none'); % 'w' = white background, 'k' = black border
% % 
% % hold off;
% % 
% % nexttile;
% % plot(step_idx, mean_step_work, 'Color', color_mean_cot, 'LineWidth', 2);
% % hold on;
% % 
% % xlim([1, inf]);
% % 
% % xlabel('Step Index');
% % ylabel('Step Work');
% % grid on
% % 
% % text(0.02, 0.80, '(d)', 'Units', 'normalized', ...
% %     'FontSize', 10, 'FontName', 'Helvetica', ...
% %     'BackgroundColor', 'w', 'EdgeColor', 'none'); % 'w' = white background, 'k' = black border
% % 
% % hold off;
% 
% format_my_plot(6.5, 7);
% 
% export_filename = "../matlab_plots/mean_step_values_" + num2str(pitch) + ".png";
% exportgraphics(gcf, export_filename, 'Resolution', 600);
