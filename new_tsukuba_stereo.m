%% New Tsukuba Stereo Vision Dataset
% Project:
% 360 Degree Stereo Vision Obstacle Avoidance in Drones
%
% Dataset:
% MATLAB NewTsukuba image sequence
%
% Current Version: V1
%
% Purpose:
% Inspect the New Tsukuba image sequence and establish
% its image structure before implementing stereo reconstruction.


clear;
clc;
close all;


%% ============================================================
% USER CONFIGURATION
% =============================================================

% ------------------------------------------------------------
% Number of images to inspect
% ------------------------------------------------------------
% USER CONFIGURATION:
% Controls how many consecutive images are loaded
% for initial dataset inspection.

numImagesToInspect = 6;


% ------------------------------------------------------------
% Starting image number
% ------------------------------------------------------------
% USER CONFIGURATION:
% Controls which image number the inspection starts from.

startImageNumber = 1;


%% ============================================================
% 1. LOCATE NEW TSUKUBA DATASET
% =============================================================

tsukubaPath = fullfile( ...
    toolboxdir('vision'), ...
    'visiondata', ...
    'NewTsukuba');


if ~isfolder(tsukubaPath)

    error('New Tsukuba dataset was not found.');

end


fprintf('\n');
fprintf('============================================\n');
fprintf('       NEW TSUKUBA DATASET\n');
fprintf('============================================\n');

fprintf('Dataset location:\n%s\n', tsukubaPath);


%% ============================================================
% 2. FIND IMAGE FILES
% =============================================================

imageFiles = dir(fullfile( ...
    tsukubaPath, ...
    'image*.jpg'));


if isempty(imageFiles)

    error('No New Tsukuba images were found.');

end


fprintf('\nTotal images found: %d\n', ...
    length(imageFiles));


%% ============================================================
% 3. SORT IMAGE FILES
% =============================================================

imageNames = {imageFiles.name};

imageNumbers = zeros(length(imageNames), 1);


for i = 1:length(imageNames)

    name = imageNames{i};

    numberString = regexp( ...
        name, ...
        '\d+', ...
        'match', ...
        'once');

    imageNumbers(i) = str2double(numberString);

end


[~, sortIndex] = sort(imageNumbers);

imageFiles = imageFiles(sortIndex);


%% ============================================================
% 4. DISPLAY DATASET RANGE
% =============================================================

fprintf('\n');
fprintf('First image: %s\n', ...
    imageFiles(1).name);

fprintf('Last image: %s\n', ...
    imageFiles(end).name);


%% ============================================================
% 5. VALIDATE STARTING IMAGE
% =============================================================

if startImageNumber < 1 || ...
        startImageNumber > length(imageFiles)

    error('Invalid startImageNumber.');

end


endImageNumber = min( ...
    startImageNumber + numImagesToInspect - 1, ...
    length(imageFiles));


%% ============================================================
% 6. LOAD SAMPLE IMAGES
% =============================================================

sampleImages = cell( ...
    endImageNumber - startImageNumber + 1, ...
    1);


for i = startImageNumber:endImageNumber

    imagePath = fullfile( ...
        imageFiles(i).folder, ...
        imageFiles(i).name);

    sampleImages{i - startImageNumber + 1} = ...
        imread(imagePath);

end


%% ============================================================
% 7. DISPLAY SAMPLE IMAGES
% =============================================================

figure( ...
    'Name', ...
    'New Tsukuba Image Sequence');


numSamples = length(sampleImages);


for i = 1:numSamples

    subplot(2, ceil(numSamples / 2), i);

    imshow(sampleImages{i});

    title(imageFiles( ...
        startImageNumber + i - 1).name);

end


sgtitle('New Tsukuba Dataset - Sample Images');


%% ============================================================
% 8. DISPLAY IMAGE INFORMATION
% =============================================================

fprintf('\n');
fprintf('============================================\n');
fprintf('       IMAGE INFORMATION\n');
fprintf('============================================\n');


for i = 1:numSamples

    currentImage = sampleImages{i};

    fprintf('%s : ', ...
        imageFiles( ...
        startImageNumber + i - 1).name);

    fprintf('%d x %d', ...
        size(currentImage, 2), ...
        size(currentImage, 1));

    if size(currentImage, 3) == 3

        fprintf(' RGB');

    else

        fprintf(' Grayscale');

    end

    fprintf('\n');

end


fprintf('\n');
fprintf('============================================\n');
fprintf('       DATASET INSPECTION COMPLETE\n');
fprintf('============================================\n');