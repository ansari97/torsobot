function wheelPlot(slope_ang, l, n, t, ang, p_contact)
% wheelPlot function for plotting the wheel at tehe particular slope
%
%   wheelPlot(x_slope, h_slope, h_plot, slope_ang, l, n, t, ang, p_init)
%%%

spoke_ang = 2*pi/n;

% calculate the coordnates of the center of mass
com = p_contact + l*[-sin(ang + slope_ang), cos(ang + slope_ang)];

% calculate the coordinates of all the spoke end points
P_feet = feetCoordinates(com, l, n, slope_ang, ang);

% colors for the wheel
color_val = [0.2, 0.5, 0.4];
collision_color_val = [1, 0, 0];
line_width = 4;
foot_size = 25;

% length of the slope for the plot
slope_length = 20 * l; % slope length is 10 times the body length

plot_lim = slope2cart(0.5*slope_length, slope_ang);

plot(0 + [-plot_lim(1) plot_lim(1)], 0 + [-plot_lim(2) (plot_lim(2))], "LineWidth", 1, "Color", [1.0, 0.5,0.5]);

hold on;

% text(0, 0, strcat('t: ', num2str(t), ' s'));
title(strcat('t: ', num2str(round(t, 4)), ' s'))

axis equal;
xlim(0 +[-plot_lim(1) plot_lim(1)])
ylim(0 + [-plot_lim(2) (plot_lim(2) + 3*l)])

% if p_contact(1) - xlim < 2
%     % xlim
% end


for i = 1:n

    if i == 1
        plot([com(1), P_feet(i, 1)], [com(2), P_feet(i, 2)], LineWidth=line_width, Color=collision_color_val);

        plot(P_feet(i, 1), P_feet(i, 2), Marker = ".", MarkerSize= foot_size ,Color=collision_color_val);
    else
        plot([com(1), P_feet(i, 1)], [com(2), P_feet(i, 2)], LineWidth=line_width, Color=color_val);
        plot(P_feet(i, 1), P_feet(i, 2), Marker = ".", MarkerSize= foot_size ,Color=color_val);

    end

end

hold off;
end

function p = slope2cart(s, slope_ang)
x = s*cos(slope_ang);
y = s*sin(slope_ang);
p = [x, y];
end
