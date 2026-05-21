%%% collect simulation data
function run_walking_simulation()

cd 'C:\Users\ahmed\Torsobot\Modelling\torsobot_dynamics\matlab'

close all; % close all open figures
clear; % clear the workspace
clear collisionResponse; % clears the value of persistent counter variable
clc; % clear command window

%% get parameters
p = get_parameters_safely();
slope_angle = p.slope_angle;
robot_param = p.robot_param;
collision_angle = p.collision_angle;
controller = p.controller;
g = p.g;

%% Initial conditions
theta_init = collision_angle + 0.01; % initial theta (wheel angle)
phi_init = pi;%deg2rad(0 - slope_angle); % initial phi (torso angle from the reference axis)
theta_dot_init = 3; % initial theta rate
phi_dot_init = 0; % initial phi rate

%% Define slope
%  Slope, received as argument in degrees to the function
slope_angle = deg2rad(slope_angle);  % angle in radians

%% Solver setup
time_interval = [0 100]; % time interval for the ODE solution
solver_type = 'ode45';
solver_max_step = 0.1; % max time step; 0.02 is reasonable
frame_skip = 100; % number of frames to skip for animation;
% skipping too many frames creates a seemingly disconneted animation

% This block corrects the initial conditions
% going up and collision angle incorrectly set to just before colliding
% up
if theta_dot_init < 0 && theta_init == -collision_angle
    theta_init = collision_angle;

    % going down and collision angle incorrectly set to just before colliding
    % down
elseif theta_dot_init > 0 && theta_init == collision_angle
    theta_init = -collision_angle;
end

% resetting angle value to be within range
if abs(theta_init) > collision_angle
    % if going up
    if theta_dot_init < 0
        theta_init = collision_angle;
        % if going down
    else
        theta_init = -collision_angle;
    end
end

e_sum = 0; % for PID controller

init_con = [theta_init, phi_init, theta_dot_init, phi_dot_init]; %, e_sum];

stop_vel = 0.02; % stop simulation if wheel angular velocity is less than this value

%% Do not change
% solver_param = {init_con, stop_vel, time_interval, solver_type, solver_max_step};
% struct for solver_param
solver_param.init_con = init_con;
solver_param.stop_vel = stop_vel;
solver_param.time_interval = time_interval;
solver_param.solver_type = solver_type;
solver_param.solver_max_step = solver_max_step;

%% Define intermediate variables
L = robot_param.L;
M = robot_param.M;
Iw = robot_param.Iw;
n = robot_param.n;
l_t = robot_param.l_t;
l = robot_param.l;
m = robot_param.m;
It = robot_param.It;

spoke_angle = 2*pi/n; % angle between two spokes

init_con = solver_param.init_con;
stop_vel = solver_param.stop_vel;
time_interval = solver_param.time_interval;
solver_type = solver_param.solver_type;
solver_max_step = solver_param.solver_max_step;

%% Dynamics setup
% Define ODE event
% for some reason "both" direction does not work for the collision event

max_num_steps = 50;

E = odeEvent(EventFcn=@collisionEvent, ...
    Direction="ascending", ...
    Response="stop",...
    CallbackFcn=@collisionResponse);

% create ode object
ode_param.slope_angle = slope_angle;
ode_param.robot_param = robot_param;
ode_param.controller = controller;
ode_param.collision_angle = collision_angle;
ode_param.stop_vel = stop_vel;

% start at this time
t = time_interval(1);

test = "pitch";

controller_name = controller.controller;

if test == "pitch"

    abs_phi_desired = 15:1:20; % from slope normal
    test_slope = 0;
    
    for j = 1: length(abs_phi_desired)
        j

        phi_desired = deg2rad(abs_phi_desired(j) - test_slope);

        ode_param.slope_angle = deg2rad(test_slope);
        ode_param.controller.phi_desired = phi_desired;
        
        step_ctr = 1;
        state = [];
        actual_time = [];
        init_con = solver_param.init_con;
        init_con(2) = solver_param.init_con(2)-deg2rad(test_slope);
        
        last_event_time = 0;
        t = 0;
        
        [sol, torque, power] = runSim();

        save_filename = controller_name + "_" +"slope" + test_slope + "pitch" + num2str(abs_phi_desired(j)) + "_sol_data";
        save("mat_files\" + save_filename + ".mat", "sol", "torque", "power");

    end

elseif test == "slope"
    test_slopes = 0:0.5:80; % deg
    abs_phi_desired = deg2rad(30); % desired in gravity coordinate system (gravity)

     for j = 1: length(test_slopes)
        j

        test_slope = test_slopes(j);
        phi_desired = abs_phi_desired - deg2rad(test_slope);

        ode_param.slope_angle = deg2rad(test_slope);
        ode_param.controller.phi_desired = phi_desired; % adjust since the state variable for phi_desired is relative to the slope normal

        step_ctr = 1;
        state = [];
        actual_time = [];
        init_con = solver_param.init_con;
        init_con(2) = solver_param.init_con(2)-deg2rad(test_slope);
        
        last_event_time = 0;
        t = 0;

        [sol, torque, power] = runSim();

        save_filename = controller_name + "_" + "slope" + num2str(rad2deg(ode_param.slope_angle)) + "pitch" + num2str(rad2deg(abs_phi_desired)) + "_sol_data";
        save("mat_files\" + save_filename + ".mat", "sol", "torque", "power");
     end

elseif test == "both"
    test_slopes = 0:2:88; % deg
    abs_test_pitches = 0:2:60;

    for s = 1:length(test_slopes)
        for q = 1:length(abs_test_pitches)

            test_slope = test_slopes(s)
            abs_phi_desired = abs_test_pitches(q)
            phi_desired = deg2rad(abs_phi_desired) - deg2rad(test_slope);
    
            ode_param.slope_angle = deg2rad(test_slope);
            ode_param.controller.phi_desired = phi_desired; 

            step_ctr = 1;
            state = [];
            actual_time = [];
            init_con = solver_param.init_con;
            init_con(2) = solver_param.init_con(2)-deg2rad(test_slope);
            
            last_event_time = 0;
            t = 0;
    
            [sol, torque, power] = runSim();
    
            save_filename = controller_name + "_" + "slope" + num2str(test_slope) + "pitch" + num2str(abs_phi_desired) + "_sol_data";
            save("mat_files\" + save_filename + ".mat", "sol", "torque", "power");
        end
    end
end


% frames = wheelTrajPlot(slope_angle, robot_param, sol, frame_skip);


% function
    function [sol, T, pow] = runSim()
    
        for i=1:max_num_steps       
        
            % Solve ODE
            F = ode(ODEFcn = @continuousDynamics, InitialValue = init_con, EventDefinition = E, Solver= solver_type, Parameters=ode_param);
            % set solver options
            F.SolverOptions.MaxStep = solver_max_step;
            F.RelativeTolerance = 1e-9; % Tighten from default 1e-3
            F.AbsoluteTolerance = 1e-12; % Tighten from default 1e-6
        
            y_sol = solve(F, t, t+100, Refine=8);
        
            % y_before = y_sol.Solution
        
            % pause()
            y_before = y_sol.EventSolution;
        
            % t and i does not do anything
            [stop,  y_after, p] = collisionResponse(t, y_before, i, ode_param);
        
            mean_vel = mean(y_sol.Solution(3, :));
            if i == max_num_steps
                mean_vel
            end

            % t = y_sol.EventTime
            init_con = y_after;
            
            % save values
            actual_time = [actual_time, y_sol.Time + last_event_time];
        
            last_event_time = last_event_time + y_sol.EventTime;
        
            state = [state, y_sol.Solution(1:4, :)];
            
            step_ctr = i;
            if actual_time(end)>time_interval(2)
                mean_vel
                break;
            end
        end
        
        sol = [actual_time; state];
    
        %% external calculations for torque
        T = zeros(1, length(actual_time));
        e = zeros(1, length(actual_time));
    
        for i = 1:1:length(actual_time)
            % For FL controller, this is no longer the torque because this returns the auxiliary input u
            % [T(i), e(i)] = PDTorqueController(t(i), state(:, i), controller_param);
        
            % [T(i), e(i)] = PDGCTorqueController(t(i), state(:, i), ode_param);
        
            y = state(:, i);
        
            if ode_param.controller.controller == "PDFL_controller"
            % FL Controller
        
                % Dynamics
                R = m*l*L*cos(y(1) - y(2));
                S = l^2*m + It;
                V = Iw + L^2*(m + M);
                
                N = [1, 0, 0, 0;
                    0, 1, 0, 0;
                    0, 0, R, S;
                    0, 0, V, R];
                
                f1 = m*l*L*y(3)^2*sin(y(1) - y(2)) + m*g*l*sin(ode_param.slope_angle + y(2));
                f2 = -m*l*L*y(4)^2*sin(y(1) - y(2)) + (m + M)*g*L*sin(ode_param.slope_angle + y(1));
                
                f = [y(3), y(4), f1, f2]';
                
                H = [0, 0, -1, 1]';
        
                % Torque controller using feedback linearization
                [u(i), e(i)] = PDFLTorqueController(actual_time(i), y, ode_param.controller); % u is the PD output
            
                T(i) = (V*f1 - R*f2)/(V + R) - ((V*S - R^2)/(V + R)) * u(i);
        
            elseif ode_param.controller.controller == "PDGC_controller"
            % PDGC
            
                % Gravity compensated PD control
                [T(i), e(i)] = PDGCTorqueController(actual_time(i), y, ode_param); % T is the torque output at the wheel; already correctted for direction
              
            end
        
        end
        
        
        
        %% calculations for motor power at the wheel
        wheel_rel_vel = state(3, :) - state(4, :);
        pow = T.*wheel_rel_vel;
    
    
    end
end

function p = get_parameters_safely()
    parameters; % Runs your external script safely in this isolated bubble
    
    % Pack the top-level variables you need into a struct
    p.slope_angle = slope_angle;
    p.robot_param = robot_param;
    p.collision_angle = collision_angle;
    p.controller = controller;
    p.g = g;
end