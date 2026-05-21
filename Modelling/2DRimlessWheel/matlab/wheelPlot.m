function wheelPlot(ax, slope_ang, l, n, t, ang, p_contact)
% wheelPlot function for plotting the wheel at tehe particular slope
%
%   wheelPlot(x_slope, h_slope, h_plot, slope_ang, l, n, t, ang, p_init)
%%%

spoke_ang = 2*pi/n;
slope_dist = 2 * l * sin(pi/n);

% calculate the coordnates of the center of mass
com = p_contact + l*[-sin(ang + slope_ang), cos(ang + slope_ang)];

% calculate the coordinates of all the spoke end points
P_feet = feetCoordinates(com, l, n, slope_ang, ang);

% colors for the wheel
color_val = [215, 215, 215]/255;
color_val(4) = 0.8;
% color_val = [0.2, 0.5, 0.4];
collision_color_val = [216, 149, 121]/255;
line_width = 3;
foot_size = 2;
collision_foot_size = 25;

% length of the slope for the plot
slope_length = 20 * l; % slope length is 10 times the body length

plot_lim = slope2cart(slope_length, slope_ang);

plot(ax, 0 + [-plot_lim(1) plot_lim(1)], 0 + [-plot_lim(2) (plot_lim(2))], "LineWidth", 1, "Color", [0, 0, 0]);

hold(ax, 'on');
ax.XTick = [];
ax.YTick = [];
ax.Color = 'white';
% ax2.Color = 'white';


% text(0, 0, strcat('t: ', num2str(t), ' s'));
% title(strcat('t: ', num2str(round(t, 4)), ' s'))

% Slope-aligned basis
slope_dir  = [cos(slope_ang),  sin(slope_ang)];
slope_perp = [-sin(slope_ang), cos(slope_ang)];

strides_per_window = 3;

% Which stride number is the current contact point?
stride_index = round(dot(p_contact, slope_dir) / slope_dist);

% Which 3-stride window does it belong to? (strides 0–2 → window 0, etc.)
window_index = floor(stride_index / strides_per_window);

% Anchor at the midpoint of this window's strides
mid_stride = window_index * strides_per_window + (strides_per_window - 1)/2;
view_center = mid_stride * slope_dist * slope_dir + l * slope_perp;

% Big enough to show 3 strides + wheel margin
view_half = 1.5 * slope_dist + 1.5 * l;

xlim(ax, view_center(1) + [-view_half, view_half]);
ylim(ax, view_center(2) + [-view_half, view_half]);

daspect(ax, [1 1 1]);          % 1:1 scale without touching limits
ax.XLimMode = 'manual';        % lock limits — no auto-adjust
ax.YLimMode = 'manual';

% axis(ax, 'equal');
axis(ax, 'off');

% if p_contact(1) - xlim < 2
%     % xlim
% end


for i = 1:n

    if i == 1
        plot(ax, [com(1), P_feet(i, 1)], [com(2), P_feet(i, 2)], LineWidth=line_width, Color=collision_color_val);

        plot(ax, P_feet(i, 1), P_feet(i, 2), Marker = ".", MarkerSize= collision_foot_size ,Color=collision_color_val);
    else
        plot(ax, [com(1), P_feet(i, 1)], [com(2), P_feet(i, 2)], LineWidth=line_width, Color=color_val);
        plot(ax, P_feet(i, 1), P_feet(i, 2), Marker = ".", MarkerSize= foot_size ,Color=color_val);

    end

    plot(ax, com(1), com(2), color=color_val, Marker = "o", MarkerSize= collision_foot_size)


end

hold(ax, 'off');
end

function p = slope2cart(s, slope_ang)
x = s*cos(slope_ang);
y = s*sin(slope_ang);
p = [x, y];
end
