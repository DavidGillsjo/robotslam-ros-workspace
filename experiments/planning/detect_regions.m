regions = detectMSERFeatures(BW);
figure; imshow(BW); hold on;
plot(regions,'showPixelList',true,'showEllipses',true);