% Detect MSER regions
I = imread('fail1.pgm');
[regions, mserCC] = detectMSERFeatures(I);

% Show all detected MSER REgions
figure
imshow(I)
hold on
plot(regions, 'showPixelList', true, 'showEllipses', false)

% Measure MSER region eccentricity to gauge region circularity.
stats = regionprops('table', mserCC, 'Eccentricity');

% Circular regions have low eccentricity. Threshold eccentricity values to
% only keep the circular regions.
eccentricityIdx = stats.Eccentricity < 0.55;
circularRegions = regions(eccentricityIdx);

% Show circular regions
figure
imshow(I)
hold on
plot(circularRegions, 'showPixelList', true, 'showEllipses', false)