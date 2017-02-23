function [ image ] = smooth_edges( image )
%SMOOTH_EDGES Summary of this function goes here
%   Detailed explanation goes here
    %Standard IPT Image
    %Its edges
    E = edge(image.data,'canny');
    %Dilate the edges
    Ed = imdilate(E,strel('disk', 10));
    %Filtered image
    Ifilt = imfilter(image.data,fspecial('gaussian'));
    %Use Ed as logical index into I to and replace with Ifilt
    image.data(Ed) = Ifilt(Ed);
end

