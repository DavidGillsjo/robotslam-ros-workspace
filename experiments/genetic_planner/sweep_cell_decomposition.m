function [ graph, intersections ] = sweep_cell_decomposition( image )
%SWEEP_CELL_DECOMPOSITION Summary of this function goes here
%   Detailed explanation goes here
    
    graph = 'snopp';
    intersections = [];
    last_num_intersections = 0;
    for col_i = 1:size(image.data, 1)
        col = image.data(:, col_i);
        diff_vector = find(diff(col) ~= 0);
        num_intersections = length(diff_vector);
        if (last_num_intersections ~= num_intersections)
            last_num_intersections = num_intersections;
            for row_i = diff_vector'
                intersections = [intersections; [col_i row_i]];
            end
        end
    end
end
