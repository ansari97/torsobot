function frame = wheelTrajPlot(slope_ang, l, n, sol, event_sol)
% wheelTrajPlot function for plotting the wheel given the slope and wheel parameters, and the wheel state
%
%   frame = wheelTrajPlot(slope_x_length, slope_ang, l, n, sol, event_sol)
%%%

% Slope parameters for plotting
slope_length = 20 * l; % slope length is 10 times the body length
% [slope_x, slope_y] = slope2cart(slope_length, slope_ang)

% unpack the solution matrix
t = sol(1, :); % time vector
ang = sol(2, :); % wheel angle
vel = sol(3, :); % wheel velocity vector

% unpack the event solution matrix
collision_time = event_sol(1, :);

% define angles
collision_ang = pi/n;
spoke_ang = 2*pi/n;

% plot height and width
% h_plot = slope_y + 2*l;
scaling = 1000;



% f.Position(3:4) = scaling*[x_slope h_plot];

% determine the initial spoke contact point
slope_dist = sqrt(2*l^2 - 2*l^2*cos(spoke_ang)); % distance between points of contact along the slope
%
% s_init = cart2slope(x_slope, y_slope) - 5*slope_dist;

% initialize foot at (0,0)
p_contact = [0,0];

% frame = zeros(len);

% e is some small angle tolerance
epsilon = 1e-10;

% create a separate figure
f = figure;

slope_length = 20 * l; % slope length is 10 times the body length

plot_lim = slope2cart(0.5*slope_length, slope_ang);

hold on;

% last_collision = false; % is true when last index had a collision

for i = 1:length(ang)

    % going down when making contact
    if vel(i)>0 && abs(ang(i) - collision_ang) < epsilon
        p_contact = p_contact - slope2cart(slope_dist, slope_ang);
        wheelPlot(slope_ang, l, n, t(i), -ang(i), p_contact);
        % last_collision = true;

        % going up when making contact
    elseif vel(i)<0 && abs(ang(i) - collision_ang) < epsilon
        p_contact = p_contact + slope2cart(slope_dist, slope_ang);
        wheelPlot(slope_ang, l, n, t(i), ang(i), p_contact);
        % last_collision = true;
    else
        wheelPlot(slope_ang, l, n, t(i), ang(i), p_contact);
        % last_collision = false;
    end

    hold off;
    pause(0.05);


    frame(i) = getframe; %get frame

end


end

%% helper functions

function y = yPoint(x, slope_ang)
y = tan(slope_ang)*x;
end

function s = cart2slope(x, y)
s = sqrt(x^2 + y^2);
end

function p = slope2cart(s, slope_ang)
x = s*cos(slope_ang);
y = s*sin(slope_ang);
p = [x, y];
end

function com = foot2com(p_foot,l, ang)
com = p_foot + l*[-cos(ang), sin(ang)];
end