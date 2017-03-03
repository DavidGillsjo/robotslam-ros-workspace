load 'mhuset2';
%image.data = logical(image.data > 220);
original_image = image;
image = rotation_correct(image);

threshold = 220;
image.data(image.data < threshold) = 0;
image.data(image.data >= threshold) = 255;

cell_size = 0.40;
grid_size = (size(image.data) * image.resolution) / cell_size;

free_matrix = find_free_cells(image, grid_size);

%obstacle_matrix = distance_to_obstacles(free_matrix);
goal_cell = find(free_matrix == 1, 1);

distances = bfs(goal_cell, free_matrix);
%distances = dijkstra(goal_cell, free_matrix, obstacle_matrix);
distances(free_matrix == 0) = -inf;

% start = find(distances == max(max(distances)), 1);
start = 8680; % hardcoded position closer to robot start pos in stage.
path = path_plan(distances, start) - 0.5;

% figure
% imshow(free_matrix, [0 1]);
% hold on
% plot(path(:, 2), path(:, 1));
% hold off
%plot_full(free_matrix, 1, path);

image_size = size(image.data);

scal = round(1 / (grid_size(1) / image_size(1)), 2);

offset = image.origin / image.resolution;

correct_path = path*scal + offset(1:2);

format_paths = format_path(correct_path * image.resolution);

fid = fopen('route.json', 'w');
fprintf(fid, jsonencode(format_paths));
fclose(fid);

img = rotation_correct(original_image);
%plot_full(image.data, scal, path, 1 - free_matrix);
plot_full(image.data, scal, path);