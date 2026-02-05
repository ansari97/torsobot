function h = circle(x,r)

th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th);
h = plot(xunit, yunit, 'r', LineWidth=2);
hold on;

end