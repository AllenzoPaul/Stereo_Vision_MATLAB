I1 = imread("cameraman.tif");

%move picture 30 pixels horizontally
I2 = imtranslate(I1,[30 0]); 

figure;

subplot(1,2,1);
imshow(I1);
title("Frame 1");

subplot(1,2,2);
imshow(I2);
title("Frame 2");

points1 = detectHarrisFeatures(I1);
points2 = detectHarrisFeatures(I2);

%extracting descriptors
[features1, validPoints1] = extractFeatures(I1,points1);
[features2, validPoints2] = extractFeatures(I2,points2);

%matching features
indexPairs = matchFeatures(features1,features2);

%retrieving matched points
matchedPoints1 = validPoints1(indexPairs(:,1));
matchedPoints2 = validPoints2(indexPairs(:,2));

%visualizing matched points
figure;

showMatchedFeatures(I1,I2, ...
    matchedPoints1,matchedPoints2);

title("Matched Features");



