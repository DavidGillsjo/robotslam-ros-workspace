function [ image ] = close_walls( image )
%CLOSE_WALLS Attempts to close all horizontal and vertical walls.
    line_length = 10;
    se = strel('disk', line_length);
    image.data = imclose(image.data, se);
    se = strel('disk', line_length);
    image.data = imclose(image.data, se);

end

