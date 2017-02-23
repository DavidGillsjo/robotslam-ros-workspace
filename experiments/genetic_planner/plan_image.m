%load 'image';
load 'fake';
image.data = logical(image.data > 250);
%image = rotation_correct(image);


imshow(image.data);