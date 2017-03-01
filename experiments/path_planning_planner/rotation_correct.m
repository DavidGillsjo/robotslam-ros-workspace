function [ image ] = rotation_correct( image )
%ROTATION_CORRECT Corrects rotation so that building is oriented correctly.
%   Just turns a hardcoded amount of degrees. However, I was thinking
%   about using houghlines to detect the longest walls/lines and then
%   automatically rotate the image so that those lines are parallel
%   to the coordinate base.

    image.data = imrotate(image.data, -90);
end

