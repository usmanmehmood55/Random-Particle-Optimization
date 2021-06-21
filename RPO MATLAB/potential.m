function J = potential(height, width, point, source)

x_gaussian = ((point(1) - source(1))^2);
y_gaussian = ((point(2) - source(2))^2);
J = height * exp(-width * (x_gaussian + y_gaussian));     % two diamensional gaussian

end