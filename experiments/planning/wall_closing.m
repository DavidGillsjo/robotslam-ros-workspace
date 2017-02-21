line_length = 20;
se = strel('line', line_length, 0);
BW = imclose(BW, se);
se = strel('line', line_length, 90);
BW = imclose(BW, se);