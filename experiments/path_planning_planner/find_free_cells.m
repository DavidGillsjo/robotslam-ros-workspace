function [ matrix ] = find_free_cells( image, grid_size )
%FIND_FREE_CELLS Returns matrix with indicies to all empty grid cells in
%   image   
    image_size = size(image.data);
    %grid_size = size(grid); 
    scale = grid_size(1) / image_size(1);
    matrix = imresize(image.data, scale, 'box'); 
    %plot_full(matrix, 1, [0 0]);
    matrix(matrix < 255) = 0;
    
    [L, num] = bwlabel(matrix);
    maxS = 0;
    for i = 1:num
        if (maxS < size(find(L == i), 1))
            maxS = size(find(L == i));
            matrix = L == i;
        end
    end
end

