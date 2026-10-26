clc;
clear;
close all;

img = imread("/home/allenzo/Pictures/archi.png");

grey = rgb2gray(img);

figure;

subplot(1,2,1);
imshow(img);
title("original image");

subplot(1,2,2);
imshow(grey)
title("Greyscale Image")