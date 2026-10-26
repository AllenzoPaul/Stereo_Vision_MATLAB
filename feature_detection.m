I = imread("cameraman.tif");
imshow(I);

points = detectHarrisFeatures(I);

figure;
imshow(I);
hold on;

plot(points);
title("Detected Features")

