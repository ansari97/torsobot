function phasePlot(sol, controller_param, collision_angle, step_size, plot_type)

t = sol(1, :);
wheel_ang = sol(2, :);
torso_ang = sol(3, :);
wheel_vel = sol(4, :);
torso_vel = sol(5, :);

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
    % size(t, 2)
    figure;

    c = linspace(0, 1, size(t, 2));

    % for the wheel
    subplot(2, 1, 1);
    plot(wheel_ang(1), wheel_vel(1), 'ro', 'MarkerSize', 10);
    hold on;
    % plot(wheel_ang, wheel_vel, 'bo', 'MarkerSize', 1);
    scatter(wheel_ang, wheel_vel, 5, c, "Marker", ".");
    % % clim([0 0.9]); % Maps data in the range [0, 0.5] to the full colormap
    % colormap('gray');
    colorbar;

    xline([0, -collision_angle collision_angle], 'k-');
    yline(0, 'k-');

    xlim([-collision_angle, collision_angle]);
    title("theta_{dot} vs theta")
    xlabel('theta (rad)')
    ylabel('theta_{dot} (rad/s)')
    ylim(max(abs(wheel_vel))*[-1.1, 1.1]);
    % grid on;
    hold off;

    % for the torso
    subplot(2, 1, 2);
    plot(torso_ang(1), torso_vel(1), 'ro', 'MarkerSize', 10);
    hold on;
    xline(controller_param.phi_desired, "k--");
    % plot(torso_ang, torso_vel, 'bo', 'MarkerSize', 1);
    scatter(torso_ang, torso_vel, 5, c, "Marker", ".");
    % clim([0 0.9]); % Maps data in the range [0, 0.5] to the full colormap
    % colormap('gray');
    colorbar;

    title("phi_{dot} vs phi")
    xlabel('phi (rad)')
    ylabel('phi_{dot} (rad/s)')
    ylim(max(abs(torso_vel))*[-1.1, 1.1]);
    % grid on;
    hold off;
end

end