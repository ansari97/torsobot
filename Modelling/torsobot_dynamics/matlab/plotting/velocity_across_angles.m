%%% velocities vs time for different angles
clc
clear
close all

pitch = [90, 80, 75, 60, 45, 30, 20];

filenames = {"2026-03-09_13-00-35", ...
    "2026-03-09_13-10-03",...
    "2026-03-09_13-12-02",...
    "2026-03-09_13-15-50",...
    "2026-03-09_13-21-21",...
    "2026-03-09_13-22-47",...
    "2026-03-09_13-24-25"};

start_times = {4530,...
    14500,...
    23900,...
    6000,...
    10300,...
    27000,...
    12000};

end_times = {55000,...
    50000,...
    43200,...
    50000,...
    50000,...
    50000,...
    90000};

wheel_vel_raw_cell = cell(1, length(filenames));
wheel_vel_filt_cell = cell(1, length(filenames));
step_data = cell(1, length(filenames));

% csv directory and file name
csv_directory  = "..\..\..\..\Code\ros2_ws\data_logs\csvs\";

time_end = 10000; % ms

max_vel = 0;

time_plot = 0:10:time_end;

colors = [
    0.00, 0.45, 0.74;  % 1. Dark Blue
    0.85, 0.33, 0.10;  % 2. Deep Rust
    0.93, 0.69, 0.13;  % 3. Goldenrod
    0.49, 0.18, 0.56;  % 4. Muted Purple
    0.47, 0.67, 0.19;  % 5. Forest Green
    0.30, 0.75, 0.93;  % 6. Light Blue (Cyan)
    0.64, 0.08, 0.18   % 7. Crimson (Dark Red)
];

fig1 = figure(1);
hold on;

for i=1:length(filenames)
    csv_filename = csv_directory + filenames{i} + ".csv";
    csv_data = readtable(csv_filename);

    file_name = num2str(pitch(i)) + "_mean_step_data.mat";
    step_data{i} = load("./mat_files/" + file_name);

    time = csv_data.timestamp;
    
    time = time - time(1); % get relative time from the first value
    time = time/(1e6); % convert to ms
    
    [~, start_idx] = min(abs(time - start_times{i}))
    [~, end_idx] = min(abs(time - (start_times{i} + time_end)))
    
    wheel_vel = csv_data.wheel_vel;
    
    % from cutoff idx onwards
    
    wheel_vel_raw_cell{i} = wheel_vel(start_idx:end_idx);
    wheel_vel_filt_cell{i} = medfilt1(wheel_vel_raw_cell{i}, 80, 'truncate');

    plot(time_plot, wheel_vel_filt_cell{i}, 'LineWidth', 2, 'Color', colors(i,:));

    if max(wheel_vel_filt_cell{i}) > max_vel
        max_vel = max(wheel_vel_filt_cell{i});
    end
end

xlim([time_plot(1), time_plot(end)])
ylim([0, 1.1*max_vel])
legend_entries = string(pitch) + " deg";
legend(legend_entries, Location="southeast")
ylabel("Wheel Velocity (rad/s)")
xlabel("Time (ms)");
grid on;

format_my_plot();

%% step data
num_steps = length(step_data{1}.mean_step_power_filt);

fig2 = figure(2);
fig2.Position = [100, 100, 1920, 1080];

max_mean_step_vel = 0;
max_mid_stance_vel = 0;
max_mean_pow = 0;

for i=1:length(filenames)

     % get step where limit cycle is reached
    diff_threshold = 1; % percent

    mean_step_vel_diff = diff(step_data{i}.mean_step_vel_filt);
    mean_step_vel_percentage_diff = mean_step_vel_diff./step_data{i}.mean_step_vel_filt(1:end-1) * 100;
    limit_cycle_step_idx_mean_step_vel = find(abs(mean_step_vel_percentage_diff)<diff_threshold);


    subplot(3, 1, 1);
    plot(step_data{i}.mean_step_vel_filt, 'LineWidth', 2, 'Color', colors(i,:)); hold on;
    % plot(limit_cycle_step_idx_mean_step_vel(1), step_data{i}.mean_step_vel_filt(limit_cycle_step_idx_mean_step_vel(1)), "*", 'Color', colors(i,:))
    
    subplot(3, 1, 2);
    plot(step_data{i}.mid_stance_vel_filt, 'LineWidth', 2, 'Color', colors(i,:)); hold on;

    subplot(3, 1, 3);
    plot(step_data{i}.mean_step_power_filt, 'LineWidth', 2, 'Color', colors(i,:)); hold on;

    if max(step_data{i}.mean_step_vel_filt) > max_mean_step_vel
        max_mean_step_vel = max(step_data{i}.mean_step_vel_filt);
    end

    if max(step_data{i}.mid_stance_vel_filt) > max_mid_stance_vel
        max_mid_stance_vel = max(step_data{i}.mid_stance_vel_filt);
    end

    if max(step_data{i}.mean_step_power_filt) > max_mean_pow
        max_mean_pow = max(step_data{i}.mean_step_power_filt);
    end

   
end

subplot(3, 1, 1);
legend_entries = string(pitch) + " deg";
legend(legend_entries, Location="best")
xlim([1, num_steps]);
ylim([0, 1.2*max_mean_step_vel]);
ylabel("Mean Step Velocity (rad/s)")
grid on;

subplot(3, 1, 2);
xlim([1, num_steps]);
ylim([0, 1.2*max_mid_stance_vel]);
ylabel("Mid Stance Velocity (rad/s)")
grid on;

subplot(3, 1, 3);
xlim([1, num_steps]);
ylim([0, 1.2*max_mean_pow]);
ylabel("Motor Power (W)")
xlabel("Step Index");
grid on;

format_my_plot();





