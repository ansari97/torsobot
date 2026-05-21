function wheelPlot(ax, slope_ang, L, n, l, l_t, t, theta, phi, p_contact, ax_limits)
% wheelPlot function for plotting the wheel at the particular slope
%
%   wheelPlot(x_slope, h_slope, h_plot, slope_ang, l, n, t, ang, p_init)

% calculate the coordinates of the center of mass
wheel_com = p_contact + L*[-sin(theta + slope_ang), cos(theta + slope_ang)];

% calculate torso location
p_torso_com = wheel_com + l*[-sin(slope_ang + phi), cos(slope_ang + phi)];
p_torso = wheel_com + l_t*[-sin(slope_ang + phi), cos(slope_ang + phi)]; % overall length of the torso

% calculate the coordinates of all the spoke end points
p_feet = feetCoordinates(slope_ang, wheel_com, L, n, theta);

% RGB colors for the wheel
wheel_color_val = [215, 215, 215]/255;
wheel_color_val(4) = 0.8;
collision_foot_color_val = [216, 149, 121]/255;
torso_color_val = [0, 115, 189]/255;
slope_color_val = [0, 0, 0];
line_width = 3;
torso_line_width = 2;
collision_foot_size = 25;
foot_size = 2;
torso_marker_size = 5;

% slope gradient
slope_gradient = tan(slope_ang);

% plot the slope
plot(ax, [ax_limits{1}, ax_limits{2}], [slope_gradient*ax_limits{1}, slope_gradient*ax_limits{2}], LineWidth = 2, Color = slope_color_val);

hold(ax, 'on');

% plot time value as title
% title(strcat('t: ', num2str(round(t, 4)), ' s'))

ax.XTick = [];
ax.YTick = [];
ax.Color = 'white';

% set xlabel
xlabel("m");

% set limits
xlim([ax_limits{1}, ax_limits{2}]);
ylim([ax_limits{3}, ax_limits{4}]);


daspect(ax, [1 1 1]);          % 1:1 scale without touching limits
ax.XLimMode = 'manual';        % lock limits — no auto-adjust
ax.YLimMode = 'manual';

% axis(ax, 'equal');
axis(ax, 'off');

for i = 1:n
    
    % plot torso
    plot(ax, [wheel_com(1), p_torso(1)], [wheel_com(2), p_torso(2)], color=torso_color_val, LineWidth=torso_line_width)
    plot(ax, p_torso_com(1), p_torso_com(2), color=torso_color_val, Marker = "o", MarkerSize= torso_marker_size)
    plot(ax, p_torso(1), p_torso(2), color=torso_color_val, Marker = "o", MarkerSize= torso_marker_size)
    
    % plot wheel collision spoke
    if i == 1
        plot(ax, [wheel_com(1), p_feet(i, 1)], [wheel_com(2), p_feet(i, 2)], LineWidth=line_width, Color=collision_foot_color_val);
        plot(ax, p_feet(i, 1), p_feet(i, 2), Marker = ".", MarkerSize= collision_foot_size ,Color=collision_foot_color_val);
    
    % plot rest of the spokes
    else
        plot(ax, [wheel_com(1), p_feet(i, 1)], [wheel_com(2), p_feet(i, 2)], LineWidth=line_width, Color=wheel_color_val);
        plot(ax, p_feet(i, 1), p_feet(i, 2), Marker = "o", MarkerSize= foot_size ,Color=wheel_color_val);
    end
    
    plot(ax, wheel_com(1), wheel_com(2), color=wheel_color_val, Marker = "o", MarkerSize= torso_marker_size)

end
hold off;
end
