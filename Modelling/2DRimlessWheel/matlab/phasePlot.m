function phasePlot(ang, vel, collision_angle, step_size, plot_type)

if plot_type == 1
    %% define range
    range = collision_angle*step_size*(10^(-15)); % larger value of range causes errors

    % find indices where collision occurs
    % can also be got from the event time vector
    ind = find(abs(abs(ang) - collision_angle) <= range);

    figure;
    plot(ang(1), vel(1), 'ko', 'MarkerSize', 10);
    hold on;

    % delete collision angle and vel value from the vector for plotting and
    % store in a different array
    ang_collision = ang(ind);
    vel_collision = vel(ind);
    ang(ind) = [];
    vel(ind) = [];

    % plot swing as back dots and collision transfer as red dots
    plot(ang, vel, "k.");

    for i = 1:length(ang_collision)
        if mod(i, 2) == 0
            plot([ang_collision(i) ang_collision(i+1)], [vel_collision(i) vel_collision(i+1)], 'r:');
        end
    end
    % plot(ang_collision(2:end), vel_collision(2:end), 'r:');

    xline([-collision_angle collision_angle], 'b-');

    xlim([-collision_angle, collision_angle]);
    xlabel('ang (rad)')
    ylabel('ang vel(rad/s)')
    % ylim([-2, 2]);
    % grid on;
    hold off;

else
    figure;

    plot(ang(1), vel(1), 'ro', 'MarkerSize', 10);
    hold on;
    plot(ang, vel, 'bo', 'MarkerSize', 1);

    xline([0, -collision_angle collision_angle], 'k-');
    yline(0, 'k-');

    xlim([-collision_angle, collision_angle]);
    xlabel('ang (rad)')
    ylabel('ang vel (rad/s)')
    ylim(max(abs(vel))*[-1.1, 1.1]);
    % grid on;
    hold off;
end

end