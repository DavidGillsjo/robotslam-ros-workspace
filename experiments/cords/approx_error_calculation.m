r = 63710088; % mean earth radius (meters)

d = @(theta) r * theta; % spherical distance
error = @(theta) r * (tan(theta) - theta);

x = linspace(0, 0.000016);

plot(d(x), error(x));

xlabel('Distance from point of tangency (meters)') % x-axis label
ylabel('Approximation error (meters)') % y-axis label