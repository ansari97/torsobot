function event = collisionEvent(t,y,p)
%%% collisionEvent detects the collision based on the wheel angle, theta
% y = [theta, phi, theta_dot, phi_dot]
% p is the vector of parameters received as argument to the ODE function

collision_angle = p{3}; % value is positive

% if velocity is +ve, return event when y(1) equals the collision angle
if y(3)>=0
    event = y(1) - collision_angle;
else
    % the -ve sign changes descending condition to ascending condition
    % for some reason "both" direction does not work for the collision
    % event
    event = -(y(1) + collision_angle); 
end

end