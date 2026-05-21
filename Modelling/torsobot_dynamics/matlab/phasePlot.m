function phasePlot(sol, controller_param, collision_angle, step_size, plot_type)

t = sol(1, :);
wheel_ang = sol(2, :);
torso_ang = sol(3, :);
wheel_vel = sol(4, :);
torso_vel = sol(5, :);

wheel_pos = wheel_ang;
torso_pitch_deg = rad2deg(torso_ang);
torso_pitch_rate = torso_vel;
time = t;

desired_torso_pitch_deg = rad2deg(controller_param.phi_desired);

color_torso_pos = [0.00, 0.45, 0.74];  % Dark Blue
color_torso_vel = [0.30, 0.75, 0.93];  % Light Blue
color_wheel_pos = [0.85, 0.33, 0.10];  % Dark Orange/Rust
color_wheel_vel = [0.93, 0.69, 0.13];  % Golden Orange
color_target    = [0.80, 0.20, 0.20];  % Muted Red for target ylines

pad_ylim = @(y, frac) [min(y) - frac*(max(y)-min(y)), ...
                       max(y) + frac*(max(y)-min(y))];

if plot_type == 1
    %% define range
    range = collision_angle*step_size*(10^(-15)); % larger value of range causes errors

    % find indices where collision occurs
    % can also be got from the event time vector
    ind = find(abs(abs(wheel_ang) - collision_angle) <= range);

    figure;
    plot(wheel_ang(1), wheel_vel(1), 'ko', 'MarkerSize', 10);
    hold on;

    % delete collision angle and vel value from the vector for plotting and
    % store in a different array
    ang_collision = wheel_ang(ind);
    vel_collision = wheel_vel(ind);
    wheel_ang(ind) = [];
    wheel_vel(ind) = [];

    % plot swing as back dots and collision transfer as red dots
    plot(wheel_ang, wheel_vel, "k.");

    for i = 1:length(ang_collision)
        if mod(i, 2) == 0
            plot([ang_collision(i) ang_collision(i+1)], [vel_collision(i) vel_collision(i+1)], 'r:');
        end
    end
    % plot(ang_collision(2:end), vel_collision(2:end), 'r:');

    xline([-collision_angle collision_angle], 'b-');

    xlim([-collision_angle, collision_angle]);
    title("theta_dot vs theta")
    xlabel('theta (rad)')
    ylabel('theta_dot (rad/s)')
    % ylim([-2, 2]);
    % grid on;
    hold off;

    else
        % % size(t, 2)
        % figure;
        % 
        % c = linspace(0, 1, size(t, 2));
        % 
        % % for the wheel
        % subplot(2, 1, 1);
        % plot(wheel_ang(1), wheel_vel(1), 'ro', 'MarkerSize', 10);
        % hold on;
        % % plot(wheel_ang, wheel_vel, 'bo', 'MarkerSize', 1);
        % scatter(wheel_ang, wheel_vel, 5, c, "Marker", ".");
        % % % clim([0 0.9]); % Maps data in the range [0, 0.5] to the full colormap
        % % colormap('gray');
        % colorbar;
        % 
        % xline([0, -collision_angle collision_angle], 'k-');
        % yline(0, 'k-');
        % 
        % xlim([-collision_angle, collision_angle]);
        % title("theta_{dot} vs theta")
        % xlabel('theta (rad)')
        % ylabel('theta_{dot} (rad/s)')
        % ylim(max(abs(wheel_vel))*[-1.1, 1.1]);
        % % grid on;
        % hold off;
        % 
        % % for the torso
        % subplot(2, 1, 2);
        % plot(torso_ang(1), torso_vel(1), 'ro', 'MarkerSize', 10);
        % hold on;
        % xline(controller_param.phi_desired, "k--");
        % % plot(torso_ang, torso_vel, 'bo', 'MarkerSize', 1);
        % scatter(torso_ang, torso_vel, 5, c, "Marker", ".");
        % % clim([0 0.9]); % Maps data in the range [0, 0.5] to the full colormap
        % % colormap('gray');
        % colorbar;
        % 
        % title("phi_{dot} vs phi")
        % xlabel('phi (rad)')
        % ylabel('phi_{dot} (rad/s)')
        % ylim(max(abs(torso_vel))*[-1.1, 1.1]);
        % % grid on;
        % hold off;

        fig3 = figure(3);

        tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
        
        ax_torso = nexttile;
        scatter(torso_pitch_deg, torso_pitch_rate, 40, time, 'o', 'filled', ...
            'MarkerEdgeColor', 'none', ... % Thin dark edge for definition
            'MarkerFaceAlpha', 0.8, ...              % Slight transparency for overlap
            'LineWidth', 0.5);
        
        colormap('parula');           % The standard Blue-to-Yellow gradient
        cb = colorbar;                % Creates the colorbar on the right
        cb.Label.String = 'Time (s)'; % Adds the label to the colorbar
        cb.Label.FontSize = 12;
        
        hold on;
        xline(desired_torso_pitch_deg, "--", 'Color', color_target, 'LineWidth', 1.5); 
        
        xlim auto;
        ylim(pad_ylim(torso_pitch_rate, 0.10));
        
        xlabel("Torso Pitch (deg)")
        ylabel('Torso Pitch Rate (rad/s)');
        
        grid on;
        
        hold off;
        
        nexttile;
        scatter(wheel_pos, wheel_vel, 40, time, 'o', 'filled', ...
            'MarkerEdgeColor', 'none', ... % Thin dark edge for definition
            'MarkerFaceAlpha', 0.8, ...              % Slight transparency for overlap
            'LineWidth', 0.5);
        
        colormap('parula');           % The standard Blue-to-Yellow gradient
        cb = colorbar;                % Creates the colorbar on the right
        cb.Label.String = 'Time (s)'; % Adds the label to the colorbar
        cb.Label.FontSize = 12;
        
        hold on;
        xline([-pi/10, pi/10], "--", 'Color', color_target, 'LineWidth', 1.5); 
        
        xlim(0.33*[-1, 1]);
        ylim(pad_ylim(wheel_vel, 0.10));
        
        xlabel("Wheel Position (rad)")
        ylabel('Wheel Velocity (rad/s)');
        
        grid on;
        
        hold off;
        
        format_my_plot(10, 4.5);
        
        % --- inset on torso phase plot around 45° ---
        main_pos = ax_torso.Position;
        
        % top-right corner inside the torso tile
        inset_x = main_pos(1) + 0.62 * main_pos(3);
        inset_y = main_pos(2) + 0.35 * main_pos(4);
        inset_w = 0.30 * main_pos(3);
        inset_h = 0.55 * main_pos(4);
        
        ax_inset = axes('Position', [inset_x, inset_y, inset_w, inset_h]);
        
        % zoom window around 45° — adjust width as needed
        x_lo = desired_torso_pitch_deg-5;  x_hi = desired_torso_pitch_deg+5;
        mask = torso_pitch_deg >= x_lo & torso_pitch_deg <= x_hi;
        
        scatter(ax_inset, torso_pitch_deg(mask), torso_pitch_rate(mask), 25, ...
                time(mask), 'o', 'filled', ...
                'MarkerEdgeColor', 'none', 'MarkerFaceAlpha', 0.8);
        hold(ax_inset, 'on');
        xline(ax_inset, desired_torso_pitch_deg, "--", ...
              'Color', color_target, 'LineWidth', 1);
        
        xlim(ax_inset, [x_lo, x_hi]);
        ylim(ax_inset, pad_ylim([-2, 3], 0.10));   % <-- tweak this
        
        % match main plot's color mapping
        colormap(ax_inset, 'parula');
        clim(ax_inset, [min(time), max(time)]);
        
        % clean look, no labels
        set(ax_inset);
        grid(ax_inset, 'on');
        box(ax_inset, 'on');
        ax_inset.LineWidth = 0.8;
                
        % save figures
        filename = "presentation_phase_plot" +  num2str(desired_torso_pitch_deg) + ".png";
        
        save_directory  = ".\matlab_plots\";
        save_filename = save_directory + filename;
        
        exportgraphics(gcf, save_filename, 'Resolution', 600);

    end

end