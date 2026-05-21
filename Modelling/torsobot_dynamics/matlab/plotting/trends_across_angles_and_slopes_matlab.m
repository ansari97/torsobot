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

test_slopes = 0:2:88; % deg
abs_test_pitches = 30:2:60;


data_cell=cell(length(test_slopes), length(abs_test_pitches));

% when limit cycle has been reached
limit_cycle_vel = zeros(length(test_slopes), length(abs_test_pitches));
limit_cycle_pow = zeros(length(test_slopes), length(abs_test_pitches));
limit_cycle_COT = zeros(length(test_slopes), length(abs_test_pitches));
limit_cycle_work = zeros(length(test_slopes), length(abs_test_pitches));
limit_cycle_work_clipped = zeros(length(test_slopes), length(abs_test_pitches));
percentage_threshold_idx = zeros(length(test_slopes), length(abs_test_pitches));
% mid_stance_vel = zeros(1, length(pitch));

controller_name = "PDGC_controller";


for s = 1:length(test_slopes)
    for q = 1:length(abs_test_pitches)

        test_slope = test_slopes(s);
        abs_phi_desired = abs_test_pitches(q);
        phi_desired = deg2rad(abs_phi_desired) - deg2rad(test_slope);

        file_name = controller_name + "_" + "slope" + num2str(test_slope) + "pitch" + num2str(abs_phi_desired) + "_sol_data";
        data_cell{s, q} = load("../mat_files/" + file_name);
    
        sol = data_cell{s,q}.sol;
        time = sol(1,:);
        torso_pitch = sol(3,:);
        torso_pitch_rate = sol(5,:);
        wheel_pos = sol(2,:);
        wheel_vel = sol(4,:);
    
        mot_pow = data_cell{s,q}.power;
    
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
        mean_step_pow = zeros(1, max_num_steps);
        mean_step_COT = zeros(1, max_num_steps);
        mean_step_work = zeros(1, max_num_steps);
        mean_step_work_clipped = zeros(1, max_num_steps);
        for j = 1:max_num_steps
            % j;
            % pre_impact_idx(j);
            % pre_impact_idx(j+1);
            % mean_step_vel(j) = mean(wheel_vel_new(pre_impact_idx(j):pre_impact_idx(j+1))); 
            % mean_step_pow(j) = mean(mot_pow(pre_impact_idx(j):pre_impact_idx(j+1))); 
            % 
            %  j;
            idx_start = pre_impact_idx(j);
            idx_end   = pre_impact_idx(j+1);
        
            step_time = time(idx_end) - time(idx_start);
        
            mean_step_vel(j) = trapz(time(idx_start:idx_end), wheel_vel(idx_start:idx_end))/step_time;
            mean_step_pow(j) = trapz(time(idx_start:idx_end), mot_pow(idx_start:idx_end))/step_time;
        
            % mean_step_vel(j) = mean(wheel_vel(pre_impact_idx(j):pre_impact_idx(j+1))); 
            % mean_step_pow(j) = mean(mot_pow(pre_impact_idx(j):pre_impact_idx(j+1))); 
            
            mean_step_work(j) = trapz(time(idx_start:idx_end), mot_pow(idx_start:idx_end));
            mean_step_work_clipped(j) = trapz(time(idx_start:idx_end), mot_pow_clipped(idx_start:idx_end));
            step_length = sqrt(2*L^2 - 2*L*L*cos(2*pi/n));
        
            mean_step_COT(j) = mean_step_work(j)/((m+M)*g*step_length);
    
        end
    
        save_steps = 2;
        limit_cycle_vel(s,q) = mean(mean_step_vel(end-save_steps:end));
        limit_cycle_pow(s,q) = mean(mean_step_pow(end-save_steps:end));
        limit_cycle_COT(s,q) = mean(mean_step_COT(end-save_steps:end));
        limit_cycle_work(s,q) = mean(mean_step_work(end-save_steps:end));
        limit_cycle_work_clipped(s,q) = mean(mean_step_work_clipped(end-save_steps:end));
    
        % get index where limit cycle is reached
        mean_step_vel_diff = diff(mean_step_vel);
        percentage_diff = mean_step_vel_diff./mean_step_vel(1:end-1)*100;
    
        percentage_threshold = 0.1; 
    
        [~, percentage_threshold_idx(s,q)] = min(abs(abs(percentage_diff) - percentage_threshold));



    end
end

% get where max occurs
% [max_vel, max_vel_idx] = max(limit_cycle_vel)
% max_vel_pitch = abs_test_pitches(max_vel_idx)
% 
% [max_COT, max_COT_idx] = max(limit_cycle_COT)
% max_COT_pitch = abs_test_pitches(max_COT_idx)






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



% create meshgrid
[S, P] = meshgrid(test_slopes, abs_test_pitches);

color_mean_vel   = [0.00, 0.45, 0.74];  % Dark Blue
color_mean_power = [0.64, 0.08, 0.18];  % Deep Maroon/Red
color_mean_cot   = [0.47, 0.67, 0.19];  % Forest Green

% tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

% nexttile
fig1 = figure(1);
surf(S, P, limit_cycle_vel');

fig2 = figure(2);
surf(S, P, limit_cycle_work');

fig3 = figure(3);
surf(S, P, limit_cycle_work_clipped');


% nexttile
% plot(pitch, limit_cycle_work, "-",'Color', color_mean_power, 'LineWidth', 2);
% hold on;
% ylabel("Limit cycle Work (W)")
% xlim([pitch(1), pitch(end)]);
% hold off;

% nexttile
% plot(pitch, limit_cycle_COT, "-",'Color', color_mean_cot, 'LineWidth', 2);
% hold on;
% ylabel("Limit cycle Mean COT")
% xlim([pitch(1), pitch(end)]);
% hold off;
% 
% xlabel("Torso Pitch (deg)")
% format_my_plot(6.5, 7.5);
% 
% export_filename = "../matlab_plots/limit_cycle_againt_pitch_" + ".png";
% exportgraphics(gcf, export_filename, 'Resolution', 600);

% normalized trends
% limit_cycle_vel_norm = limit_cycle_vel/max(limit_cycle_vel);
% limit_cycle_pow_norm = limit_cycle_pow/max(limit_cycle_pow);
% limit_cycle_COT_norm = limit_cycle_COT/max(limit_cycle_COT);
% 
% fig2 = figure(2);
% plot(pitch, limit_cycle_vel_norm, "-",'Color', color_mean_vel, 'LineWidth', 2); hold on;
% % plot(pitch, limit_cycle_pow_norm, "-",'Color', color_mean_power, 'LineWidth', 2);
% plot(pitch, limit_cycle_COT_norm, "-",'Color', color_mean_cot, 'LineWidth', 2);
% 
% xlim([pitch(1), pitch(end)]); 
% ylim([0, 1.1]); 
% 
% xlabel("Torso Pitch (deg)")
% ylabel("Normalized Values")
% 
% legend("Limit Cycle Mean Wheel Velocity", ...
%     "Limit Cycle Mean COT",...
%     'Location', "best");
% 
% hold off;
% 
% format_my_plot(6.5);
% 
% export_filename = "../matlab_plots/limit_cycle_againt_pitch_combined_" + ".png";
% exportgraphics(gcf, export_filename, 'Resolution', 600);

%% limit cycle step index vs torso pitch
% fig3 = figure(3);
% 
% plot(pitch, percentage_threshold_idx, "-",'Color', color_mean_vel, 'LineWidth', 2);
% hold on;
% xlabel("Torso Pitch (deg)")
% ylabel("Limit Cycle Step Index")
% xlim([pitch(1), pitch(end)]); 
% ylim([min(percentage_threshold_idx) - 1, max(percentage_threshold_idx) + 1]); 
% hold off;
% format_my_plot(6.5);
% 
% export_filename = "../matlab_plots/limit_cycle_step_idx_" + ".png";
% exportgraphics(gcf, export_filename, 'Resolution', 600);


%% plot step data for a specific pitch
% pitch = -30;
% 
% file_name = controller_name + "_" + "pitch" + pitch + "_sol_data.mat"
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
% % mot_pow = clip(mot_pow, 0, inf);
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
