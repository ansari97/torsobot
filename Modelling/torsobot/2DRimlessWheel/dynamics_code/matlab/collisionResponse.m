function [stop,  y, p] = collisionResponse(t, y,i,p)

% reset the angle
if y(2) >= 0
    y(1) = - pi/p(2);
else
    y(1) = + pi/p(2);
end

% velocity is scaled down by the vel_coeff factor
y(2) = p(4)*y(2); 

% if velocity is less than a threshold value, stop solving
if abs(y(2)) < p(5)
    stop = true;
else
    stop = false;
end

end