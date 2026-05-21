%%% for plotting csvs

clc;
clear;
close all;

%% Get files

% Enter filename
filename = input("CSV filename: ", 's');
start_time = input("Start time (ms): ", 's');
start_time = str2double(start_time);

end_time = input("End time (ms): ", 's');
end_time = str2double(end_time);

% csv directory and file name
csv_directory  = "..\..\..\..\Code\ros2_ws\data_logs\csvs\";
csv_filename = csv_directory + filename + ".csv";

% yaml directory and file name
yaml_directory  = "..\..\..\..\Code\ros2_ws\data_logs\metadata\";
yaml_filename = yaml_directory + filename + ".yaml";

disp(["Reading file from: ", csv_filename])

%% get data from yaml
yaml_data = yaml.loadFile(yaml_filename);

desired_torso_pitch_deg = yaml_data.x___.ros__parameters.desired_torso_pitch_deg
controller = yaml_data.x___.ros__parameters.controller;
kp = yaml_data.x___.ros__parameters.kp;
ki = yaml_data.x___.ros__parameters.ki;
kd = yaml_data.x___.ros__parameters.kd;
wheel_max_torque = yaml_data.x___.ros__parameters.wheel_max_torque;
control_max_integral = yaml_data.x___.ros__parameters.control_max_integral;

torso_pitch_init = yaml_data.trial_values.torso_pitch_init;
wheel_rel_pos_init = yaml_data.trial_values.wheel_rel_pos_init;

%% get data from csv
csv_data = readtable(csv_filename);

time = csv_data.timestamp;
time_diff_s = mean(diff(time))/1e9 % convert from ns to s

time = time - time(1); % get relative time from the first value
time = time/(1e6); % convert to ms

[~, start_idx] = min(abs(time - start_time));
[~, end_idx] = min(abs(time - end_time));

torso_pitch = csv_data.torso_pitch;
torso_pitch_rate = csv_data.torso_pitch_rate;
wheel_pos = csv_data.wheel_pos;
wheel_vel = csv_data.wheel_vel;
wheel_torque = csv_data.wheel_torque;
wheel_cmd_torque = csv_data.wheel_cmd_torque;
encoder_steps = csv_data.encoder_steps;
encoder_ang = csv_data.encoder_ang;
encoder_speed = csv_data.encoder_speed;

% plot original data for wheel_pos
fig1 = figure(1);
subplot(2, 1, 1);
plot(time, wheel_pos);
subplot(2, 1, 2);
plot(time, torso_pitch);

% from cutoff idx onwards
time = time(start_idx:end_idx);
torso_pitch = torso_pitch(start_idx:end); 
torso_pitch_rate = torso_pitch_rate(start_idx:end);
wheel_pos = wheel_pos(start_idx:end);
wheel_vel_raw = wheel_vel(start_idx:end);
wheel_torque = wheel_torque(start_idx:end);
wheel_cmd_torque = wheel_cmd_torque(start_idx:end);
encoder_steps = encoder_steps(start_idx:end);
encoder_ang = encoder_ang(start_idx:end);
encoder_speed = encoder_speed(start_idx:end);

%% recalculating wheel pos and velocity data 
limit = pi/10;
total_width = pi/5;

wheel_rel_pos = encoder_ang - wheel_rel_pos_init;
wheel_pos_calc = wheel_rel_pos + (torso_pitch - torso_pitch_init);
wheel_pos_calc = wheel_pos_calc + limit;

% filter wheel_pos to remove chatter
wheel_pos_filt = medfilt1(wheel_pos_calc, 4, 'truncate');

% wrap position again

wheel_pos_wrap = mod(wheel_pos_filt + limit, total_width) - limit;

% find where impact occurs using diff and comparing it to a threshold
% difference (+/-2pi/10) on the wrapped angle
threshold = 0.5;
wheel_pos_diff = diff(wheel_pos_wrap);
pre_impact_idx = find(abs(wheel_pos_diff) > threshold);

num_steps_in_run = length(pre_impact_idx)

%% get data for first 30 steps only
max_num_steps = 40;
pre_impact_idx = pre_impact_idx(1:max_num_steps+1);

time = time(pre_impact_idx(1):pre_impact_idx(end));
time = time - time(1);
torso_pitch = torso_pitch(pre_impact_idx(1):pre_impact_idx(end)); 
torso_pitch_rate = torso_pitch_rate(pre_impact_idx(1):pre_impact_idx(end));
wheel_pos = wheel_pos(pre_impact_idx(1):pre_impact_idx(end));
wheel_pos_filt = wheel_pos_filt(pre_impact_idx(1):pre_impact_idx(end));
wheel_pos_wrap = wheel_pos_wrap(pre_impact_idx(1):pre_impact_idx(end));
wheel_vel_raw = wheel_vel_raw(pre_impact_idx(1):pre_impact_idx(end));
wheel_torque = wheel_torque(pre_impact_idx(1):pre_impact_idx(end));
wheel_cmd_torque = wheel_cmd_torque(pre_impact_idx(1):pre_impact_idx(end));
encoder_steps = encoder_steps(pre_impact_idx(1):pre_impact_idx(end));
encoder_ang = encoder_ang(pre_impact_idx(1):pre_impact_idx(end));
encoder_speed = encoder_speed(pre_impact_idx(1):pre_impact_idx(end));

% 
% adjust pre_impact_idx
pre_impact_idx = pre_impact_idx - pre_impact_idx(1) + 1;

%% calculate wheel velocity again
% perform center difference on all values
wheel_vel_calc = gradient(wheel_pos_wrap, time_diff_s);
% backward and forward differentiation at the pre and post impact points
for i= 1:length(pre_impact_idx)-1
    % wheel_vel_calc(pre_impact_idx(i)-1) = (wheel_pos_calc(pre_impact_idx(i)-1) - wheel_pos_calc(pre_impact_idx(i)-2))/time_diff_s;
    if i ~= 1
        wheel_vel_calc(pre_impact_idx(i)) = (wheel_pos_filt(pre_impact_idx(i)) - wheel_pos_filt(pre_impact_idx(i)-1))/time_diff_s;
    end
    wheel_vel_calc(pre_impact_idx(i)+1) = (wheel_pos_filt(pre_impact_idx(i)+2) - wheel_pos_filt(pre_impact_idx(i)+1))/time_diff_s;
    % wheel_vel_calc(pre_impact_idx(i)+2) = (wheel_pos_calc(pre_impact_idx(i)+3) - wheel_pos_calc(pre_impact_idx(i)+2))/time_diff_s;
end

%% Filtering wheel velocity data

wheel_vel_med_fil = medfilt1(wheel_vel_raw, 3, 'truncate');

d1 = designfilt("lowpassiir", ...
    FilterOrder=2, ... 
    HalfPowerFrequency=40, ... % Now this literally means 8 Hz
    SampleRate=100, ...       % Your 100 Hz data frequency
    DesignMethod="butter");
wheel_vel_butter_fil = filtfilt(d1, wheel_vel_raw);

% Savitzky-Golay
dt = time_diff_s;           
order = 3;           % cubic polynomial
window_length = 11;  

% 'g' is a matrix containing the coefficients for different derivatives
[~, g] = sgolay(order, window_length);

% We use g(:, 1) which holds the smoothing coefficients
pos_smoothed = conv(wheel_pos_filt, g(:,1), 'same');

wheel_vel_sg = conv(wheel_pos_filt, flipud(g(:,2)), 'same') / dt;

% our choice for velocity
wheel_vel = wheel_vel_butter_fil;

%% Average step velocity
mean_step_vel = zeros(1, max_num_steps);
for i = 1:max_num_steps
    mean_step_vel(i) = mean(wheel_vel(pre_impact_idx(i)+5 : pre_impact_idx(i+1))); % the 5 gets rid of the drop in velocity
end

%% mid stance step velocity
% mid_stance_idx = find(abs(wheel_pos) < 0.01);
mid_stance_idx = find(wheel_pos_wrap(1:end-1) < 0 & wheel_pos_wrap(2:end) >= 0);
% mid_stance_idx = mid_stance_idx(1:end);
num_mid_stance = length(mid_stance_idx);
mid_stance_vel = zeros(1, num_mid_stance);

for i = 1:num_mid_stance
    mid_stance_vel(i) = wheel_vel(mid_stance_idx(i));
end

%% Get step where limit velocity has been reached
% mid stance and mean step velocity
steps = 1:1:length(mean_step_vel);
steps_log = 1./(steps);
mean_step_vel_coeff = polyfit(steps_log, mean_step_vel, 1);
mean_step_vel_fit = polyval(mean_step_vel_coeff, steps_log);

mid_stances = 1:length(mid_stance_vel);
mid_stances_log = 1./(mid_stances);
mid_stance_vel_coeff = polyfit(mid_stances_log, mid_stance_vel, 1);
mid_stance_vel_fit = polyval(mid_stance_vel_coeff, mid_stances_log);

% filter velocity instead of fit
mean_step_vel_filt = mean_step_vel;
mean_step_vel_filt(3:end) = medfilt1(mean_step_vel(3:end), 5, 'truncate');
mid_stance_vel_filt = mid_stance_vel;
mid_stance_vel_filt(3:end) = medfilt1(mid_stance_vel(3:end), 5, 'truncate');

pt_threshold = 2; % percent
% mean step vel
mean_step_vel_diff = diff(mean_step_vel_filt);
mean_step_pt_diff = mean_step_vel_diff./mean_step_vel_filt(1:end-1)*100; % percentage diff

limit_cycle_step_idx_mean_step_vel = find(abs(mean_step_pt_diff)<pt_threshold);
limit_cycle_step_idx_mean_step_vel = limit_cycle_step_idx_mean_step_vel(1)

% mid stance vel
mid_stance_vel_diff = diff(mid_stance_vel_filt);
mid_stance_pt_diff = mid_stance_vel_diff./mid_stance_vel_filt(1:end-1)*100; % percentage diff

limit_cycle_step_idx_mid_stance_vel = find(abs(mid_stance_pt_diff)<pt_threshold);
limit_cycle_step_idx_mid_stance_vel = limit_cycle_step_idx_mid_stance_vel(1)


%% time data plotting
torso_pitch = rad2deg(torso_pitch); % convert to deg

title_text = "Pitch: " + desired_torso_pitch_deg + "^o";

fig2 = figure(2);
sgtitle(title_text);

subplot(2, 1, 1);
plot(time, wheel_pos); hold on;
xlim([time(1), time(end)])
ylabel("wheel pos (rad)"); 
hold off;

subplot(2, 1, 2);
plot(time, wheel_pos_wrap); hold on;
xlim([time(1), time(end)])
ylabel("wheel pos clean(rad)"); 
hold off;

fig3 = figure(3);
sgtitle(title_text);

subplot(5, 1, 1);
plot(time, wheel_pos_wrap); hold on;
xlim([time(1), time(end)])
ylabel("wheel pos (rad)"); 
hold off;

% subplot(6, 1, 2);
% plot(time, wheel_pos_calc); hold on;
% xlim([time(1), time(end)])
% ylabel("wheel pos (rad)"); hold off;

subplot(5, 1, 2);
plot(time, wheel_vel_raw); hold on;
ylabel("wheel vel (rad/s)");
ylim([-1.5 6]);
xlim([time(1), time(end)])

legend("raw", "Location", "northeast");
hold off;

subplot(5, 1, 3);
plot(time, wheel_vel_med_fil); hold on;
ylabel("wheel vel (rad/s)");
ylim([-1.5 6]);

xlim([time(1), time(end)])

legend("med\_filtered", "Location", "northeast");
hold off;

subplot(5, 1, 4);
plot(time, wheel_vel_sg); hold on;
ylabel("wheel vel (rad/s)");
ylim([-1.5 6]);

xlim([time(1), time(end)])

legend("sgolay\_filtered", "Location", "northeast");
hold off;

subplot(5, 1, 5);
plot(time, wheel_vel_butter_fil); hold on;
ylabel("wheel vel (rad/s)");
ylim([-1.5 6]);

xlim([time(1), time(end)])

legend("butterworth\_filtered", "Location", "northeast");
hold off;

%% Power
power = wheel_torque.*encoder_speed;
power_filt = medfilt1(power, 5, 'truncate');
fig4 = figure(4);
plot(time, power_filt); hold on;
ylabel("Motor Power (W)");hold off;

save_directory  = "..\..\..\..\Code\ros2_ws\data_logs\graphs\power_plots\";
save_filename = save_directory + filename;

exportgraphics(fig4, save_filename + "_pow.png", Resolution=300)

% look at encoder speed data 
% get mean power per step
mean_step_power = zeros(1, max_num_steps);
for i = 1:max_num_steps
    mean_step_power(i) = mean(power_filt(pre_impact_idx(i)+5 : pre_impact_idx(i+1))); % the 5 gets rid of the drop in velocity
end
mean_step_power_filt = mean_step_power;
mean_step_power_filt(3:end) = medfilt1(mean_step_power(3:end), 5, 'truncate');

%% mid stance and mean step velocity
fig5 = figure(5);

color_step_raw      = [0.60, 0.80, 0.98]; % Soft Light Blue
color_step_filt     = [0.00, 0.45, 0.74]; % Deep Blue
color_mid_raw       = [0.99, 0.75, 0.55]; % Soft Peach/Orange
color_mid_filt      = [0.85, 0.33, 0.10]; % Deep Rust/Orange
color_power         = [0.30, 0.30, 0.30]; % Dark Slate Gray

yyaxis left
plot(steps, mean_step_vel, "-", 'Color', color_step_raw, 'LineWidth', 1); hold on;
plot(steps, mean_step_vel_filt, "-", 'Color', color_step_filt, 'LineWidth', 2);
plot(mid_stances, mid_stance_vel, "-", 'Color', color_mid_raw, 'LineWidth', 1);
plot(mid_stances, mid_stance_vel_filt, "-", 'Color', color_mid_filt, 'LineWidth', 2);
ylabel("wheel vel (rad/s)");
ylim([0, max(mean_step_vel_filt)*1.2]);

yyaxis right
plot(steps, mean_step_power_filt, "--", 'Color', color_power, 'LineWidth', 2);
ylabel("motor power (W)");
ylim([0, max(mean_step_power_filt)*1.2]);

xlabel("Step Index");
xlim([1, steps(end)]);
legend('Mean Step Vel (Raw)', 'Mean Step Vel (Filt)', ...
       'Mid-Stance Vel (Raw)', 'Mid-Stance Vel (Filt)', ...
       'Motor Power (Filt)', 'Location', 'northeast');

title(title_text);
grid on

format_my_plot();
hold off;

% save figures
save_directory  = "..\..\..\..\Code\ros2_ws\data_logs\graphs\mean_across\";
save_filename = save_directory + filename;

exportgraphics(fig5, save_filename + "_vel_pow.png", Resolution=300)
% exportgraphics(fig6, save_filename + "_pow.png")

%% scatter plot/phase portrait
fig7 = figure(7);
color_target = [0.80, 0.20, 0.20];

c = linspace(0, time(end), length(time));
scatter(wheel_pos, wheel_vel, 30, c, 'filled', 'MarkerFaceAlpha', 0.5); 
hold on;

xline(-pi/10, '--', 'Color', color_target, 'LineWidth', 1.5);
xline(pi/10, '--', 'Color', color_target, 'LineWidth', 1.5);

title(title_text); 
xlabel('Wheel Position (rad)');
ylabel('Wheel Velocity (rad/s)');

xlim([-0.33, 0.33]);

cb = colorbar;
cb.Label.String = 'Time (ms)';       
cb.Label.FontSize = 12;
cb.Label.FontWeight = 'bold';

grid on; 

hold off;

format_my_plot();

% save figures
save_directory  = "..\..\..\..\Code\ros2_ws\data_logs\graphs\phase_plots\";
save_filename = save_directory + filename;

exportgraphics(fig7, save_filename + "_wheelphase.png", Resolution=300)

%% scatter plot/phase portrait for the torso
fig8 = figure(8);
color_target = [0.80, 0.20, 0.20];

c = linspace(0, time(end), length(time));
scatter(torso_pitch, torso_pitch_rate, 30, c, 'filled', 'MarkerFaceAlpha', 0.5); 
hold on;

xline(desired_torso_pitch_deg, '--', 'Color', color_target, 'LineWidth', 1.5);

title(title_text); % Assuming title_text is already defined
xlabel('Torso Pitch (deg)');
ylabel('Torso Pitch Rate (rad/s)');

cb = colorbar;
cb.Label.String = 'Time (s)';       
cb.Label.FontSize = 12;
cb.Label.FontWeight = 'bold';

grid on; 

hold off;

format_my_plot();

% save figures
save_directory  = "..\..\..\..\Code\ros2_ws\data_logs\graphs\phase_plots\";
save_filename = save_directory + filename;

exportgraphics(fig7, save_filename + "_torsophase.png", Resolution=300)
%% time data plotting
fig9 = figure(9);
% torso_pitch = rad2deg(torso_pitch); % convert to deg
sgt = sgtitle(title_text);
sgt.FontSize = 14;
sgt.FontWeight = 'bold';
sgt.FontName = 'Helvetica';

% subplot(5, 1, 1);
% plot(time, wheel_torque);hold on;
% plot(time, wheel_cmd_torque, 'r'); 
% xlim([time(1), time(end)]);
% legend("wheel torque", "wheel cmd torque", Location="northeast");
% ylabel("wheel torque (Nm)")
% grid on; 
% grid minor;
% hold off;

color_torso_pos = [0.00, 0.45, 0.74];  % Dark Blue
color_torso_vel = [0.30, 0.75, 0.93];  % Light Blue
color_wheel_pos = [0.85, 0.33, 0.10];  % Dark Orange/Rust
color_wheel_vel = [0.93, 0.69, 0.13];  % Golden Orange
color_target    = [0.80, 0.20, 0.20];  % Muted Red for target ylines

subplot(4, 1, 1);
plot(time, torso_pitch, 'Color', color_torso_pos, 'LineWidth', 2); hold on;
yline(desired_torso_pitch_deg, "--", 'Color', color_target, 'LineWidth', 2); 
xlim([time(1), time(end)]);
ylim([0.80*min(torso_pitch), 1.20*max(torso_pitch)])
legend('Actual Pitch', 'Target Pitch', 'Location', 'northeast');
ylabel("Torso Pitch (deg)");
grid on; 
hold off;

subplot(4, 1, 2);
plot(time, torso_pitch_rate, 'Color', color_torso_vel, 'LineWidth', 2);hold on;
ylabel("Torso Pitch Rate (rad/s)");
grid on; 
xlim([time(1), time(end)]);
ylim([1.20*min(torso_pitch_rate), 1.20*max(torso_pitch_rate)])
hold off;

subplot(4, 1, 3);
plot(time, wheel_pos_wrap, 'Color', color_wheel_pos, 'LineWidth', 2); hold on;
yline(-pi/10, '--', 'Color', color_target, 'LineWidth', 1.5);
yline(pi/10, '--', 'Color', color_target, 'LineWidth', 1.5);
ylabel("Wheel Position (rad)");
grid on; 
xlim([time(1), time(end)]);
ylim([-0.33, 0.33])
hold off;

subplot(4, 1, 4);
plot(time, wheel_vel, 'Color', color_wheel_vel, 'LineWidth', 2); hold on;
ylabel("Wheel Velocity (rad/s)");
xlabel("Time (ms)");
grid on; 
xlim([time(1), time(end)]);
ylim([0.80*min(wheel_vel), 1.20*max(wheel_vel)])
hold off;
format_my_plot();

% save figures
save_directory  = "..\..\..\..\Code\ros2_ws\data_logs\graphs\time_plots\";
save_filename = save_directory + filename;

exportgraphics(fig9, save_filename + "_time.png", Resolution=300)


%% Save mean values from all steps
mat_filename = "./mat_files/" + num2str(desired_torso_pitch_deg) + "_mean_step_data.mat";
save(mat_filename, "mean_step_vel_filt", "mid_stance_vel_filt", "mean_step_power_filt");

%% Save mean values from last 3 steps
save_steps = 5;
mean_step_vel_save = mean(mean_step_vel_filt(end-save_steps:end));
mid_stance_vel_save = mean(mid_stance_vel_filt(end-save_steps:end));
mean_step_power_save = mean(mean_step_power_filt(end-save_steps:end));

mat_filename = "./mat_files/" + num2str(desired_torso_pitch_deg) + "deg_data.mat";
save(mat_filename, "mean_step_vel_save", "mid_stance_vel_save", "mean_step_power_save");

%% functions
function encoder_ang =  encoderCountsToRad(local_signed_encoder_steps) 
  encoder_ang = (local_signed_encoder_steps) / (2048*4) * 2 * PI;

end