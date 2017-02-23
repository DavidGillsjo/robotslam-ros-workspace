%load 'mhuset2';
load 'fake';
image = smooth_edges(image);
image.data = imgaussfilt(image.data, 5);
image.data = logical(image.data > 250);
%image = rotation_correct(image);
image = close_walls(image);
[graph, intersections] = sweep_cell_decomposition(image);

imshow(image.data);
hold on;
plot(intersections(:, 1), intersections(:, 2), 'o');
hold off;