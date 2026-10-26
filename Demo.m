clear;
clc;

disp("=== SLAM MATLAB Capability Test ===");

% Image Processing
disp("Image Processing:");
disp(exist('rgb2gray','file'));

% Feature detection
disp("ORB:");
disp(exist('detectORBFeatures','file'));

% Feature extraction
disp("Feature extraction:");
disp(exist('extractFeatures','file'));

% Feature matching
disp("Feature matching:");
disp(exist('matchFeatures','file'));

% Essential matrix
disp("Essential matrix:");
disp(exist('estimateEssentialMatrix','file'));

% Camera pose
disp("Relative pose:");
disp(exist('estrelpose','file'));

% Triangulation
disp("Triangulation:");
disp(exist('triangulate','file'));

% Rigid transformation
disp("Rigid transform:");
disp(exist('rigidtform3d','file'));

% Stereo reconstruction
disp("Stereo parameters:");
disp(exist('stereoParameters','file'));

disp("=== Test Complete ===");