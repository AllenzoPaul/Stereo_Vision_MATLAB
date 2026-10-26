leftImages = imageDatastore(fullfile(toolboxdir("vision"), ...
    "visiondata","calibration","stereo","left"));
rightImages = imageDatastore(fullfile(toolboxdir("vision"), ...
    "visiondata","calibration","stereo","right"));

I1 = readimage(leftImages,1);
I2 = readimage(rightImages,1);
figure;
imshowpair(I1,I2,"montage");
title("Stereo Image Pair");

load("webcamsSceneReconstruction.mat");
whos stereoParams

[J1,J2,reprojectionMatrix] = ...
    rectifyStereoImages(I1,I2,stereoParams);
figure;
imshow(stereoAnaglyph(J1,J2));
title("Rectified Stereo Pair");

disparityMap = disparitySGM(im2gray(J1),im2gray(J2));
figure;
imshow(disparityMap,[0 64]);
title("Disparity Map");

xyzPoints = reconstructScene(disparityMap,reprojectionMatrix);
size(xyzPoints)
ptCloud = pointCloud(xyzPoints);
figure;
pcshow(ptCloud);
title("3D Reconstruction");