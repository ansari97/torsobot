close all
clear
clc

%% define wheel parameters
L = 300e-3; % total length of the rod in m
l = L/2; % length from pivot to center of mass, m
m = 500e-3; % torso mass in kg
I_r = 1/12*m*L^2; % for a slender rod about the rotation axis passing through the CoM
% I_r = 100;
r = 100e-3; % wheel radius in m
M = 250e-3; % wheel mass in kg
I_w = 1/2*M*r^2; % for a solid disc of mass M and radius r; could be different if wheel geometry is different

wheel_param = [l, m, I_r, r, M, I_w];


%% define initial conditions
init_x = 5;
init_ang = deg2rad(30);
init_vel = 1; 
init_ang_vel = 0.5;

init_con = [init_x; init_ang; init_vel; init_ang_vel];

%% Desired values
q_des = [0, 0, 0, 0]';

%% Controller gain
[K, CLP, controllable, A, B] = controllerGainLQR(wheel_param);

%% solver settings
solver_max_step = 0.02;
time_interval = [0 20];
% create ode object
E = odeEvent(EventFcn=@(t, q)collisionEvent(t, q, wheel_param), ...
    Direction="ascending", ...
    Response="stop");

F = ode(ODEFcn = @(t, q) wheeledPendulumDynamcis(t, q, wheel_param, K), InitialValue = init_con, EventDefinition = E,  Solver = 'ode45');
% set solver options
F.SolverOptions.MaxStep = solver_max_step;

% Solve ODE
q_sol = solve(F, time_interval(1), time_interval(2), Refine=8);

%% solver results
state = q_sol.Solution;
% matrix of solution values (time as row 1, angle as 2, ang_vel as 3)
t = q_sol.Time;
sol = [t; state(1, :); state(2, :); state(3, :); state(4, :)];

% Torque as a function of time
T = -K*(state-q_des);

disp(strcat('Simulation time: ', num2str(max(t)), ' s'));

plot(t, state(1, :)); hold on;
title("x vs time");
xlabel("t (s)");
ylabel("x (m)");
grid on;
xline(0);
yline(0);
hold off;

figure;
plot(t, state(2, :)); hold on;
title("theta vs time");
xlabel("t (s)");
ylabel("${\theta} (rad)$", 'Interpreter','latex');
grid on;
xline(0);
yline(0);
hold off;

figure;
plot(t, state(3, :)); hold on;
title("x\_dot vs time");
xlabel("t (s)");
ylabel("$\dot{x} (m/s)$", 'Interpreter','latex');
grid on;
xline(0);
yline(0);
hold off;

figure;
plot(t, state(4, :)); hold on;
title("theta\_dot vs time");
xlabel("t (s)");
ylabel('$\dot{\theta} (rad/s)$', 'Interpreter','latex');
grid on;
xline(0);
yline(0);
hold off;

figure;
plot(state(2, :), state(4, :)); hold on;
title("theta\_dot vs theta");
xlabel('${\theta} (rad)$', 'Interpreter','latex');
ylabel('$\dot{\theta} (rad/s)$', 'Interpreter','latex');
grid on;
xline(0);
yline(0);
hold off;

figure;
plot(state(1, :), state(3, :)); hold on;
title("x\_dot vs x");
xlabel('${x} (m)$', 'Interpreter','latex');
ylabel('$\dot{x} (m/s)$', 'Interpreter','latex');
grid on;
xline(0);
yline(0);
hold off;

figure;
plot(t, T); hold on;
title("T vs t");
xlabel('t (s)');
ylabel('T (N.m)');
grid on;
xline(0);
yline(0);
hold off;

% frame = drawFigure(wheel_param, t, state);

make_movie = false;

% make a video
if make_movie
    len = length(t);
    time_max = max(t); % end time for the solution
    time_min = min(t); % start time of the solution (should be 0)

    t_movie = time_min:solver_max_step*10:time_max; % time for the movie frame
    % t_ind = zeros(length(t_movie))
    for i = 1:length(t_movie)
        ind = find(abs(t - t_movie(i)) < solver_max_step/10);

        if ~isempty(ind) > 0
            if length(ind)>1
                ind = round(mean(ind));
            end
            t_ind(i) = ind;
        elseif isempty(ind)
            % do nothing
        end
    end


    frame = frame(t_ind);
    movie(frame, 1, 10);
end

