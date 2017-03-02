function [ output_args ] = plot_full( image_data, scal, path, image_data_2 )
%PLOT_FULL Summary of this function goes here
%   Detailed explanation goes here
    image_data = uint8(image_data * 255);
    if nargin < 4
        image_data_2 = uint8(ones(size(image_data)));
    else
        image_data_2 = uint8(image_data_2);
        image_scale = size(image_data, 1) / size(image_data_2, 1);
        image_data_2 = imresize(image_data_2, image_scale, 'box'); 
    end
    
    mx = max(max(image_data));
    image_data(:, :, 2) = image_data .* image_data_2;
    image_data(:, :, 3) = image_data(:, :, 1);
    
    figure
    %p = correct_path * image.resolution;
    p = path*scal;

    imshow(image_data, [0 mx]);
    %imagesc(image_data);
    hold on;
    plot(p(:, 2), p(:, 1));
    hold off

    axis on;
    grid on;
%    set(gca,'xtick', (0:scal:size(image_data, 2)));
%    set(gca,'ytick', (0:scal:size(image_data, 1)));
     set(gca,'xtick', (0:scal:size(image_data, 2)), 'xticklabel', 0:1:size(image_data, 2)/scal);
     set(gca,'ytick', (0:scal:size(image_data, 1)), 'yticklabel', 0:1:size(image_data, 1)/scal);
end

