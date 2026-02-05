function y = takeOneStep(init_con, p)
%%% takeOneStep takes one wheel step starting from some initial condition
% right after a collision, and returns the state right after the next
% collision

slope_angle = p.slope_angle;
robot_param = p.robot_param;
collision_angle= p.collision_angle;
solver_param = p.solver_param;

stop_vel = solver_param.stop_vel;
time_interval = solver_param.time_interval;
solver_type = solver_param.solver_type;
solver_max_step = solver_param.solver_max_step;

p.stop_vel = stop_vel;

% continuous dynamics
% return state just before collision
E = odeEvent(EventFcn=@collisionEvent, ...
    Direction="ascending", ...
    Response="stop");

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