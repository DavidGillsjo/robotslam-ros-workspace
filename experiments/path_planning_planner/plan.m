load 'mhuset2';

image.data = logical(image.data > 220);
image = rotation_correct(image);

cell_size = 0.3;
grid_size = (size(image.data) * image.resolution) / cell_size;
%grid = zeros(ceil(grid_size));

free_matrix = find_free_cells(image, grid);

goal_cell = find(free_matrix == 1, 1);

distances = bfs(goal_cell, free_matrix);


%imshow(image.data);