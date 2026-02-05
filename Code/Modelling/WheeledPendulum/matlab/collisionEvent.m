function event = collisionEvent(t, q, param)

l = param(1);
r = param(4);
L = 2*l;

event = abs(q(2)) - (pi  - acos(r/L));

end