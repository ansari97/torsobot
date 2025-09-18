function y = takeOneStep(init_con, p)
%%% takeOneStep takes one wheel step starting from some initial condition
% right after a collision, and returns the state right after the next
% collision

slope_angle = p{1};
robot_param= p{2};
collision_angle= p{3};
solver_param = p{4};

[stop_vel, time_interval, solver_type, solver_max_step] = solver_param{:};

% continuous dynamics
% return state just before collision
E = odeEvent(EventFcn=@collisionEvent, ...
    Direction="ascending", ...
    Response="stop");

p = {slope_angle, robot_param, collision_angle, stop_vel};

% create ode object
F = ode(ODEFcn = @continuousDynamics, InitialValue = init_con, EventDefinition = E, Solver= solver_type, Parameters=p);
% set solver options
F.SolverOptions.MaxStep = solver_max_step;

% Solve ODE
y_sol = solve(F, time_interval(1), time_interval(2), Refine=8);

if ~isempty(y_sol.EventTime)
    event_sol = [y_sol.EventTime; y_sol.EventSolution(1,:); y_sol.EventSolution(2,:); y_sol.EventSolution(3,:); y_sol.EventSolution(4,:)];
else
    error('No collision occurs during the time interval used. Increase the time interval for the simulation.')
end

% collision dynamics from before collision to after collision
% state after collision
% event_sol
[stop,  y, p] = collisionResponse(event_sol(1), event_sol(2:5), 1, p);

% return
% y = ;

end