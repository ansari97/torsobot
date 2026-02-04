function frames = wheelTrajPlot(slope_angle, robot_param, sol, frame_skip)
% wheelTrajPlot function for plotting the wheel given the slope and wheel parameters, and the wheel state
%
%   frame = wheelTrajPlot(slope_x_length, slope_ang, L, n, sol, event_sol)
%%%

% robot parameters
L = robot_param.L;
M = robot_param.M;
Iw = robot_param.Iw;
n = robot_param.n;
l_t = robot_param.l_t;
l = robot_param.l;
m = robot_param.m;
It = robot_param.It;

% Slope parameters for plotting
slope_length = 20 * L; % slope length is 10 times the body length
% [slope_x, slope_y] = slope2cart(slope_length, slope_ang)

% unpack the solution matrix
t = sol(1, :); % time vector
theta = sol(2, :); % wheel angle
phi = sol(3, :); % wheel angle
theta_dot = sol(4, :); % wheel velocity vector

% define angles
collision_ang = pi/n;
spoke_ang = 2*pi/n;

% distance between points of contact along the slope
slope_dist = sqrt(2*L^2 - 2*L^2*cos(spoke_ang));

% initialize foot at (0,0)
init_p_contact = [0, 0];

% e is some small angle tolerance
epsilon = 1e-10;

% create a separate figure
f = figure('Position', [100, 100, 1280, 720]);

% set plotting limits
plot_lim = slope2cart(0.5*slope_length, slope_angle);

% initialize varibales
frame_ind = 1;
p_contact = init_p_contact;

for i = 1:length(theta)

    % changing angle for plotting
    change_ang = false;

    % going down when making contact
    if theta_dot(i)>0 && abs(theta(i) - collision_ang) < epsilon
        p_contact = p_contact - slope2cart(slope_dist, slope_angle);
        change_ang = true;

    % going up when making contact
    elseif theta_dot(i)<0 && abs(theta(i) - collision_ang) < epsilon
        p_contact = p_contact + slope2cart(slope_dist, slope_angle);
    end

    % set xlimits for the plot
    if p_contact(1) == init_p_contact(1)
        plot_xlim_to_send = init_p_contact(1) + [-plot_lim(1), plot_lim(1)];
    else % p_contact ~= 0
        if (abs(plot_xlim_to_send(1)) - abs(p_contact(1))) < plot_lim(1)/2 ...
                 % || (abs(plot_xlim_to_send(2)) - abs(p_contact(1))) < plot_lim(1)/2
            plot_xlim_to_send = p_contact(1) + [-plot_lim(1), plot_lim(1)];
        end
    end

    % set ylimits for the plot
    if p_contact(2) == init_p_contact(2)
        plot_ylim_to_send = init_p_contact(2) + [-plot_lim(2), (plot_lim(2) + 3*L)];
    else % p_contact ~= 0
        if (abs(plot_ylim_to_send(1)) - abs(p_contact(2))) < plot_lim(2)/2 ...
                % || (abs(plot_ylim_to_send(2)) - abs(p_contact(2))) < plot_lim(2)/2
            plot_ylim_to_send = p_contact(2) + [-plot_lim(2), (plot_lim(2) + 3*L)];
        end
    end

    % send limits to the plotting function
    plot_lim_to_send_cell = num2cell([plot_xlim_to_send, plot_ylim_to_send]);

    if ~mod(i, frame_skip)
        if change_ang
            wheelPlot(slope_angle, L, n, l, l_t,  t(i), -theta(i), phi(i), p_contact, plot_lim_to_send_cell);
        else
            wheelPlot(slope_angle, L, n, l, l_t, t(i), theta(i), phi(i), p_contact, plot_lim_to_send_cell);
        end

        pause(0.05);
        frames(frame_ind) = getframe(f); %get frame
        frame_ind = frame_ind + 1;
    end
end

pause(0.5);
close(f);
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

function com = foot2com(p_foot,L, theta)
com = p_foot + L*[-cos(theta), sin(theta)];
end