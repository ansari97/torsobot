function wheelPlot(slope_ang, L, n, l, t, theta, phi, p_contact, ax_limits)
% wheelPlot function for plotting the wheel at the particular slope
%
%   wheelPlot(x_slope, h_slope, h_plot, slope_ang, l, n, t, ang, p_init)

% calculate the coordinates of the center of mass
com = p_contact + L*[-sin(theta + slope_ang), cos(theta + slope_ang)];

% calculate torso location
p_torso = com + l*[-sin(slope_ang + phi), cos(slope_ang + phi)];

% calculate the coordinates of all the spoke end points
p_feet = feetCoordinates(slope_ang, com, L, n, theta);

% RGB colors for the wheel
wheel_color_val = [0.2, 0.5, 0.4, 0.2];
collision_foot_color_val = [1, 0.5, 0.25];
torso_color_val = [1, 0, 0];
slope_color_val = [0, 0, 0];
line_width = 3;
torso_line_width = 5;
collision_foot_size = 25;
foot_size = 2;
torso_marker_size = 40;

% slope gradient
slope_gradient = tan(slope_ang);

% plot the slope
plot([ax_limits{1}, ax_limits{2}], [slope_gradient*ax_limits{1}, slope_gradient*ax_limits{2}], LineWidth = 1, Color = slope_color_val);

hold on;

% plot time value as title
title(strcat('t: ', num2str(round(t, 4)), ' s'))

axis equal;
axis manual;

% set xlabel
xlabel("m");

% set limits
xlim([ax_limits{1}, ax_limits{2}]);
ylim([ax_limits{3}, ax_limits{4}]);

for i = 1:n
    
    % plot torso
    plot([com(1), p_torso(1)], [com(2), p_torso(2)], color=torso_color_val, LineWidth=torso_line_width)
    plot(p_torso(1), p_torso(2), color=torso_color_val, Marker = ".", MarkerSize= torso_marker_size)
    
    % plot wheel collision spoke
    if i == 1
        plot([com(1), p_feet(i, 1)], [com(2), p_feet(i, 2)], LineWidth=line_width, Color=collision_foot_color_val);
        plot(p_feet(i, 1), p_feet(i, 2), Marker = ".", MarkerSize= collision_foot_size ,Color=collision_foot_color_val);
    
    % plot rest of the spokes
    else
        plot([com(1), p_feet(i, 1)], [com(2), p_feet(i, 2)], LineWidth=line_width, Color=wheel_color_val);
        plot(p_feet(i, 1), p_feet(i, 2), Marker = "o", MarkerSize= foot_size ,Color=wheel_color_val);
    end
    
    plot(com(1), com(2), color=torso_color_val, Marker = ".", MarkerSize= torso_marker_size)

end
hold off;
end
