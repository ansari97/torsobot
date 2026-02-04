function P_feet = feetCoordinates(slope_ang, com, L, n, theta)
spoke_ang = 2*pi/n;
P_feet = zeros(n, 2);

for i = 1:n
    psi = pi - ((i - 1)*spoke_ang - (theta + slope_ang));
    P_feet(i, :) = com + L*[-sin(psi), cos(psi)];
end

end