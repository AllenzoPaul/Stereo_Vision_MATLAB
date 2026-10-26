%% ============================================================
% UTIAS STEREO VISION - LIVE OBJECT + DEPTH
% ============================================================

clear;
clc;
close all;

%% ============================================================
% 1. USER CONFIGURATION
% =============================================================

datasetPath = '~/datasets/utias';

startFrame = 1;

maxFrames = 1000;

disparityRange = [0 128];

detectionThreshold = 0.4;

minDepth = 0.5;

maxDepth = 30;

displayEvery = 1;

%% ============================================================
% 2. DATASET PATHS
% =============================================================

leftPath  = fullfile(datasetPath, 'images', 'left');
rightPath = fullfile(datasetPath, 'images', 'right');

leftFiles = dir(fullfile(leftPath, '*.png'));
rightFiles = dir(fullfile(rightPath, '*.png'));

if isempty(leftFiles) || isempty(rightFiles)
    error('Stereo images were not found.');
end

if numel(leftFiles) ~= numel(rightFiles)
    error('Left and right image counts do not match.');
end

numFrames = min( ...
    [numel(leftFiles), ...
     numel(rightFiles), ...
     maxFrames]);

if startFrame > numFrames
    error('Start frame exceeds available frames.');
end

fprintf('Stereo pairs available : %d\n', ...
    min(numel(leftFiles), numel(rightFiles)));

fprintf('Frames selected        : %d\n\n', ...
    numFrames - startFrame + 1);

%% ============================================================
% 3. CAMERA PARAMETERS
% =============================================================

focalLength = [387.777 387.777];

principalPoint = [257.446 197.718];

imageSize = [384 512];

baseline = 0.239965;

%% ============================================================
% 4. LOAD OBJECT DETECTOR
% =============================================================

fprintf('Loading YOLOv4 detector...\n');

try

    detector = yolov4ObjectDetector( ...
        "tiny-yolov4-coco");

catch

    error([ ...
        'YOLOv4 model is not installed.\n\n' ...
        'Install the Computer Vision Toolbox Model for YOLO v4 ' ...
        'Object Detection support package from MATLAB Add-On Explorer.' ...
        ]);

end

fprintf('YOLOv4 detector loaded.\n\n');

%% ============================================================
% 5. CREATE LIVE DISPLAY
% =============================================================

figureHandle = figure( ...
    'Name','Drone Stereo Vision - Object Detection + Depth', ...
    'NumberTitle','off', ...
    'Color','w');

layout = tiledlayout(1,2, ...
    'TileSpacing','compact', ...
    'Padding','compact');

%% Left camera display

leftAxes = nexttile(layout,1);

leftDisplay = imshow( ...
    zeros(imageSize(1), ...
          imageSize(2), ...
          'uint8'), ...
    'Parent',leftAxes);

title(leftAxes, ...
    'Left Camera - Object Detection');

%% Right depth display

depthAxes = nexttile(layout,2);

depthDisplay = imagesc( ...
    depthAxes, ...
    nan(imageSize));

axis(depthAxes,'image');

xlabel(depthAxes,'X (pixels)');
ylabel(depthAxes,'Y (pixels)');

title(depthAxes, ...
    'Stereo Depth Map');

depthColorbar = colorbar(depthAxes);

depthColorbar.Label.String = ...
    'Depth (m)';

clim(depthAxes, ...
    [minDepth maxDepth]);

%% ============================================================
% 6. DISPLAY STATUS
% =============================================================

statusText = annotation( ...
    figureHandle, ...
    'textbox', ...
    [0.05 0.01 0.90 0.045], ...
    'String','Initializing...', ...
    'HorizontalAlignment','center', ...
    'VerticalAlignment','middle', ...
    'FontSize',11, ...
    'FontWeight','bold', ...
    'EdgeColor','none');

%% ============================================================
% 7. PROCESS STEREO IMAGE SEQUENCE
% =============================================================

fprintf('Starting live stereo perception...\n\n');

frameTimes = zeros(numFrames-startFrame+1,1);

for k = startFrame:numFrames

    loopStart = tic;

    %% --------------------------------------------------------
    % Read stereo images
    % ---------------------------------------------------------

    leftImage = imread( ...
        fullfile(leftPath, ...
        leftFiles(k).name));

    rightImage = imread( ...
        fullfile(rightPath, ...
        rightFiles(k).name));

    %% --------------------------------------------------------
    % Calculate disparity
    % ---------------------------------------------------------

    grayLeft = im2gray(leftImage);

    grayRight = im2gray(rightImage);

    disparityMap = disparitySGM( ...
        grayLeft, ...
        grayRight, ...
        'DisparityRange',disparityRange);

    %% --------------------------------------------------------
    % Calculate depth
    % ---------------------------------------------------------

    depthMap = ...
        (focalLength(1) * baseline) ...
        ./ disparityMap;

    invalidDepth = ...
        ~isfinite(depthMap) | ...
        disparityMap <= 0 | ...
        depthMap < minDepth | ...
        depthMap > maxDepth;

    depthMap(invalidDepth) = NaN;

    %% --------------------------------------------------------
    % Object detection
    % ---------------------------------------------------------

    [bboxes, scores, labels] = detect( ...
        detector, ...
        leftImage, ...
        Threshold=detectionThreshold);

    %% --------------------------------------------------------
    % Calculate object depth
    % ---------------------------------------------------------

    objectDepths = ...
        nan(size(bboxes,1),1);

    for i = 1:size(bboxes,1)

        x = round(bboxes(i,1));
        y = round(bboxes(i,2));

        w = round(bboxes(i,3));
        h = round(bboxes(i,4));

        x1 = max(1,x);
        y1 = max(1,y);

        x2 = min(size(depthMap,2),x+w-1);
        y2 = min(size(depthMap,1),y+h-1);

        if x2 <= x1 || y2 <= y1
            continue;
        end

        roiDepth = ...
            depthMap(y1:y2,x1:x2);

        validDepth = ...
            roiDepth(isfinite(roiDepth));

        if isempty(validDepth)
            continue;
        end

        objectDepths(i) = ...
            median(validDepth);

    end

    %% --------------------------------------------------------
    % Find nearest detected object
    % ---------------------------------------------------------

    validObjectDepths = ...
        objectDepths(isfinite(objectDepths));

    if isempty(validObjectDepths)

        nearestDistance = NaN;

    else

        nearestDistance = ...
            min(validObjectDepths);

    end

    %% --------------------------------------------------------
    % Update left camera display
    % ---------------------------------------------------------

    set(leftDisplay, ...
        'CData',leftImage);

    delete(findall(leftAxes, ...
        'Type','rectangle'));

    delete(findall(leftAxes, ...
        'Type','text'));

    hold(leftAxes,'on');

    for i = 1:size(bboxes,1)

        rectangle( ...
            leftAxes, ...
            'Position',bboxes(i,:), ...
            'LineWidth',2);

        if isfinite(objectDepths(i))

            labelText = sprintf( ...
                '%s %.2f | %.2f m', ...
                string(labels(i)), ...
                scores(i), ...
                objectDepths(i));

        else

            labelText = sprintf( ...
                '%s %.2f | Depth unavailable', ...
                string(labels(i)), ...
                scores(i));

        end

        text( ...
            leftAxes, ...
            bboxes(i,1), ...
            max(1,bboxes(i,2)-8), ...
            labelText, ...
            'FontSize',9, ...
            'FontWeight','bold', ...
            'Color','w', ...
            'BackgroundColor','k', ...
            'Margin',2, ...
            'Interpreter','none');

    end

    hold(leftAxes,'off');

    title(leftAxes, ...
        sprintf( ...
        'Left Camera - Frame %d',k));

    %% --------------------------------------------------------
    % Update depth display
    % ---------------------------------------------------------

    set(depthDisplay, ...
        'CData',depthMap);

    clim(depthAxes, ...
        [minDepth maxDepth]);

    title(depthAxes, ...
        sprintf( ...
        'Stereo Depth Map - Frame %d',k));

    %% --------------------------------------------------------
    % Calculate FPS
    % ---------------------------------------------------------

    frameTimes(k-startFrame+1) = ...
        toc(loopStart);

    currentFPS = ...
        1 / frameTimes(k-startFrame+1);

    %% --------------------------------------------------------
    % Update status
    % ---------------------------------------------------------

    if isfinite(nearestDistance)

        statusString = sprintf( ...
            'Frame: %d / %d     |     FPS: %.1f     |     Objects: %d     |     Nearest Object: %.2f m', ...
            k, ...
            numFrames, ...
            currentFPS, ...
            size(bboxes,1), ...
            nearestDistance);

    else

        statusString = sprintf( ...
            'Frame: %d / %d     |     FPS: %.1f     |     Objects: %d     |     Nearest Object: --', ...
            k, ...
            numFrames, ...
            currentFPS, ...
            size(bboxes,1));

    end

    statusText.String = ...
        statusString;

    %% --------------------------------------------------------
    % Update MATLAB graphics
    % ---------------------------------------------------------

    drawnow limitrate;

    %% --------------------------------------------------------
    % Console output
    % ---------------------------------------------------------

    if mod(k-startFrame+1,displayEvery) == 0

        if isfinite(nearestDistance)

            fprintf( ...
                'Frame %4d / %4d | Objects: %2d | Nearest: %6.2f m | FPS: %5.1f\n', ...
                k, ...
                numFrames, ...
                size(bboxes,1), ...
                nearestDistance, ...
                currentFPS);

        else

            fprintf( ...
                'Frame %4d / %4d | Objects: %2d | Nearest:    --   | FPS: %5.1f\n', ...
                k, ...
                numFrames, ...
                size(bboxes,1), ...
                currentFPS);

        end

    end

    %% --------------------------------------------------------
    % Stop if window is closed
    % ---------------------------------------------------------

    if ~isvalid(figureHandle)

        fprintf('\nDisplay window closed.\n');

        break;

    end

end

%% ============================================================
% 8. FINAL RESULTS
% =============================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('LIVE STEREO PERCEPTION COMPLETE\n');
fprintf('============================================================\n');

fprintf('Frames processed       : %d\n', ...
    k-startFrame+1);

fprintf('Last frame             : %d\n', ...
    k);

fprintf('============================================================\n');

fprintf('\nStereo perception successfully generated:\n');

fprintf('  ✓ Stereo disparity\n');
fprintf('  ✓ Depth map\n');
fprintf('  ✓ Object detection\n');
fprintf('  ✓ Object distance estimation\n');
fprintf('  ✓ Live perception display\n');

fprintf('\n');