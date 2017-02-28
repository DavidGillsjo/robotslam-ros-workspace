load 'mhuset2';

image.data = logical(image.data > 220);
image = rotation_correct(image);

cell_size = 0.3;
grid_size = (size(image.data) * image.resolution) / cell_size;
%grid = zeros(ceil(grid_size));

free_matrix = find_free_cells(image, grid_size);

goal_cell = find(free_matrix == 1, 1);

distances = bfs(goal_cell, free_matrix);
distances(free_matrix == 0) = -inf;

path = path_plan(distances);

figure
imshow(free_matrix, [0 1]);
hold on
plot(path(:, 2), path(:, 1));
hold off