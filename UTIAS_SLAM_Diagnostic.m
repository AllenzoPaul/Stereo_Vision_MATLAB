%% ============================================================
% UTIAS STEREO VISUAL SLAM DIAGNOSTIC
% ============================================================

clear;
clc;
close all;

%% ============================================================
% 1. USER CONFIGURATION
% =============================================================

datasetPath = '~/datasets/utias';

maxFrames = 300;

maxNumPoints = 600;

trackFeatureRange = [30 120];

skipMaxFrames = 5;

displayEvery = 5;

stopOnTrackingLost = false;

%% ============================================================
% 2. DATASET PATHS
% =============================================================

leftPath  = fullfile(datasetPath, 'images', 'left');
rightPath = fullfile(datasetPath, 'images', 'right');

leftFiles  = dir(fullfile(leftPath, '*.png'));
rightFiles = dir(fullfile(rightPath, '*.png'));

if isempty(leftFiles) || isempty(rightFiles)
    error('Stereo images were not found.');
end

if numel(leftFiles) ~= numel(rightFiles)
    error('Left and right image counts do not match.');
end

numFrames = min( ...
    [numel(leftFiles), numel(rightFiles), maxFrames]);

fprintf('Stereo pairs available : %d\n', ...
    min(numel(leftFiles), numel(rightFiles)));

fprintf('Frames selected        : %d\n\n', numFrames);

%% ============================================================
% 3. CAMERA PARAMETERS
% =============================================================

focalLength = [387.777 387.777];

principalPoint = [257.446 197.718];

imageSize = [384 512];

baseline = 0.239965;

intrinsics = cameraIntrinsics( ...
    focalLength, ...
    principalPoint, ...
    imageSize);

%% ============================================================
% 4. CREATE STEREO VISUAL SLAM
% =============================================================

vslam = stereovslam( ...
    intrinsics, ...
    baseline, ...
    MaxNumPoints=maxNumPoints, ...
    TrackFeatureRange=trackFeatureRange, ...
    SkipMaxFrames=skipMaxFrames);

fprintf('Stereo vSLAM object created.\n\n');

%% ============================================================
% 5. INITIALIZE DIAGNOSTIC FIGURES
% =============================================================

figure(1);

set(gcf, ...
    'Name', 'UTIAS SLAM - Stereo Input', ...
    'NumberTitle', 'off');

figure(2);

set(gcf, ...
    'Name', 'UTIAS SLAM - Map and Trajectory', ...
    'NumberTitle', 'off');

figure(3);

set(gcf, ...
    'Name', 'UTIAS SLAM - Tracking Status', ...
    'NumberTitle', 'off');

statusHistory = zeros(numFrames,1);

keyFrameHistory = false(numFrames,1);

mapPointHistory = zeros(numFrames,1);

poseHistory = zeros(numFrames,1);

%% ============================================================
% 6. PROCESS STEREO IMAGE SEQUENCE
% =============================================================

fprintf('Starting stereo visual SLAM...\n\n');

for k = 1:numFrames

    leftImage = imread( ...
        fullfile(leftPath, leftFiles(k).name));

    rightImage = imread( ...
        fullfile(rightPath, rightFiles(k).name));

    addFrame( ...
        vslam, ...
        leftImage, ...
        rightImage);

    %% --------------------------------------------------------
    % Check SLAM status
    % ---------------------------------------------------------

    status = checkStatus(vslam);

    statusCode = uint8(status);

    statusHistory(k) = statusCode;

    %% --------------------------------------------------------
    % Check for new keyframe
    % ---------------------------------------------------------

    newKeyFrame = hasNewKeyFrame(vslam);

    keyFrameHistory(k) = newKeyFrame;

    %% --------------------------------------------------------
    % Query current map and poses
    % ---------------------------------------------------------

    try

        xyzPoints = mapPoints(vslam);

        mapPointHistory(k) = size(xyzPoints,1);

    catch

        mapPointHistory(k) = 0;

    end

    try

        [camPoses,~] = poses(vslam);

        poseHistory(k) = numel(camPoses);

    catch

        poseHistory(k) = 0;

    end

    %% --------------------------------------------------------
    % Status text
    % ---------------------------------------------------------

    if statusCode == 0

        statusText = 'TRACKING LOST';

    elseif statusCode == 1

        statusText = 'TRACKING SUCCESSFUL';

    elseif statusCode == 2

        statusText = 'FREQUENT KEYFRAMES';

    else

        statusText = 'UNKNOWN';

    end

    %% --------------------------------------------------------
    % Command window output
    % ---------------------------------------------------------

    if mod(k,displayEvery) == 0 || k == 1

        fprintf( ...
            'Frame %4d / %4d | Status: %-20s | KeyFrame: %d | Map: %5d | Poses: %4d\n', ...
            k, ...
            numFrames, ...
            statusText, ...
            newKeyFrame, ...
            mapPointHistory(k), ...
            poseHistory(k));

    end

    %% --------------------------------------------------------
    % Display stereo input
    % ---------------------------------------------------------

    if mod(k,displayEvery) == 0 || k == 1

        figure(1);

        subplot(1,2,1);

        imshow(leftImage);

        title(sprintf('Left Image - Frame %d',k));

        subplot(1,2,2);

        imshow(rightImage);

        title(sprintf('Right Image - Frame %d',k));

        drawnow;

    end

    %% --------------------------------------------------------
    % Display SLAM map
    % ---------------------------------------------------------

    if newKeyFrame

        figure(2);

        plot(vslam);

        title(sprintf( ...
            'UTIAS Stereo SLAM - Frame %d', ...
            k));

        drawnow;

    end

    %% --------------------------------------------------------
    % Display tracking history
    % ---------------------------------------------------------

    if mod(k,displayEvery) == 0 || k == 1

        figure(3);

        clf;

        subplot(2,1,1);

        plot( ...
            1:k, ...
            statusHistory(1:k), ...
            'LineWidth',1.5);

        ylim([-0.2 2.2]);

        yticks([0 1 2]);

        yticklabels({ ...
            'Tracking Lost', ...
            'Tracking Successful', ...
            'Frequent Keyframes'});

        xlabel('Frame');

        ylabel('Status');

        title('SLAM Tracking Status');

        grid on;

        subplot(2,1,2);

        plot( ...
            1:k, ...
            mapPointHistory(1:k), ...
            'LineWidth',1.5);

        hold on;

        plot( ...
            1:k, ...
            poseHistory(1:k), ...
            'LineWidth',1.5);

        hold off;

        xlabel('Frame');

        ylabel('Count');

        title('Map Points and Camera Poses');

        legend( ...
            'Map Points', ...
            'Camera Poses', ...
            'Location','best');

        grid on;

        drawnow;

    end

    %% --------------------------------------------------------
    % Optional early stop
    % ---------------------------------------------------------

    if stopOnTrackingLost && statusCode == 0

        fprintf('\nTracking lost at frame %d.\n',k);

        break;

    end

end

%% ============================================================
% 7. WAIT FOR SLAM PROCESSING TO FINISH
% =============================================================

fprintf('\nWaiting for SLAM processing to finish...\n');

while ~isDone(vslam)

    if hasNewKeyFrame(vslam)

        figure(2);

        plot(vslam);

        title('Finalizing UTIAS Stereo SLAM');

        drawnow;

    end

    pause(0.01);

end

%% ============================================================
% 8. EXTRACT FINAL SLAM RESULTS
% =============================================================

fprintf('\nExtracting final SLAM results...\n');

xyzPoints = mapPoints(vslam);

[camPoses,viewIds] = poses(vslam);

%% ============================================================
% 9. FINAL RESULTS
% =============================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('UTIAS STEREO SLAM DIAGNOSTIC COMPLETE\n');
fprintf('============================================================\n');

fprintf('Frames processed       : %d\n',numFrames);

fprintf('Final map points       : %d\n',size(xyzPoints,1));

fprintf('Final camera poses     : %d\n',numel(camPoses));

fprintf('Keyframes detected     : %d\n',sum(keyFrameHistory));

fprintf('Tracking lost frames   : %d\n', ...
    sum(statusHistory == 0));

fprintf('Tracking successful    : %d\n', ...
    sum(statusHistory == 1));

fprintf('Frequent keyframes     : %d\n', ...
    sum(statusHistory == 2));

fprintf('============================================================\n');

%% ============================================================
% 10. FINAL SLAM VISUALIZATION
% =============================================================

figure(2);

plot(vslam);

title('Final UTIAS Stereo Visual SLAM');

drawnow;

%% ============================================================
% 11. FINAL STATUS
% =============================================================

if isempty(camPoses)

    fprintf('\nWARNING:\n');
    fprintf('No camera poses were generated.\n');
    fprintf('SLAM did not successfully initialize.\n');

elseif isempty(xyzPoints)

    fprintf('\nWARNING:\n');
    fprintf('Camera poses were generated, but no 3-D map points were returned.\n');

else

    fprintf('\nSUCCESS:\n');
    fprintf('Stereo SLAM generated both camera poses and a 3-D map.\n');

end