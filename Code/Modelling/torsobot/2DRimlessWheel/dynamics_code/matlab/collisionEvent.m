function event = collisionEvent(t,y,p)

% if velocity is +ve, return event when y(1) or ang is negative
if y(2)>=0
    event = y(1) - p(3);
else
    event = -(y(1) + p(3)); % changes descending condition to ascending condition
end

end