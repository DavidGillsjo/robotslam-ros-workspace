load 'mhuset2';
original_image = image;
image = rotation_correct(image);

grid_size = [10 10];
image.data = [
  0 0 0 0 0 0 0 0 0 0
  0 1 0 1 1 1 1 1 1 0
  0 1 0 1 1 1 1 1 1 0
  0 1 0 1 1 1 1 1 1 0 
  0 1 1 1 1 1 1 1 1 0
  0 1 1 1 1 1 1 1 1 0
  0 1 1 1 1 1 0 1 1 0
  0 1 1 1 1 1 0 1 1 0
  0 1 1 1 1 1 0 1 1 0
  0 0 0 0 0 0 0 0 0 0
];

image.data(image.data == 1) = 255;
free_matrix = find_free_cells(image, grid_size);

%obstacle_matrix = distance_to_obstacles(free_matrix);
goal_cell = sub2ind(size(free_matrix), 8, 9);

distances = bfs(goal_cell, free_matrix);
distances(free_matrix == 0) = -inf;

start = find(distances == max(max(distances)), 1);
path = path_plan(distances, start);% - 0.5;

% figure
imshow(imresize(free_matrix, 64, 'box'), [0 1]);
hold on
plot(path(:, 2)*64-32+1/2, path(:, 1)*64-32+1/2);
set(gca, 'Box', 'off');
hold off
