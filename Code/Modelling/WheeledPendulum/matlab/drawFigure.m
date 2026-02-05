function frame = drawFigure(params, t, state)

% unpack params
r = params(4);
L = 2*params(1);

x = state(1, :);
theta = state(2, :);

phi = -x/r;

figure;

plot_span = 20*r;
init_xlim = [x(1)-plot_span/2, x(1)+plot_span/2];

for i = 1:length(theta)

    % plot circle
    circle(x(i),r);

    axis equal;

    if abs(x(i) - init_xlim(1)) < plot_span/4 || abs(x(i) - init_xlim(2)) < plot_span/4
        axis([x(i)-plot_span/2, x(i)+plot_span/2, -r, L*3]);
    else
        axis([init_xlim, -r, L*3]);
    end

    init_xlim = xlim;

    % draw line to show circle angle
    plot([x(i), x(i)-r*sin(phi(i))], [0, r*cos(phi(i))], 'r');

    % draw  torso
    plot([x(i), x(i)-L*sin(theta(i))], [0, L*cos(theta(i))], 'k', LineWidth=2);

    title(strcat('t: ', num2str(round(t(i), 4)), ' s'));
    pause(0.1);
    hold off;

    frame(i) = getframe;

end
end

