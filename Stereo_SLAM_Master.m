%% ============================================================
% 360 DEGREE STEREO VISION OBSTACLE AVOIDANCE
% Stereo Visual SLAM Master Prototype
%
% Pipeline:
% Stereo Video -> Rectification -> Disparity/Depth
% -> 3D Point Cloud -> Stereo Visual SLAM -> Camera Trajectory
% -> 3D Map -> Obstacle Analysis
%
% VIO / IMU is NOT included in this version.
% ============================================================

clear;
clc;
close all;


%% ============================================================
% 1. USER CONFIGURATION
% =============================================================

% Leave empty to select files using a file browser.

leftVideoPath  = '';
rightVideoPath = '';

% Stereo calibration file (.mat).
%
% The file must contain a stereoParameters object.
% Example variable:
% stereoParams

stereoCalibrationPath = '';

% Processing frequency.
%
% Lower values reduce computation.
% Example:
% 5 = process approximately 5 frames/sec.

processingFPS = 5;

% Disparity range used for dense stereo reconstruction.
%
% Difference between minimum and maximum must be divisible by 16.

disparityRange = [0 128];

% Disparity visualization.

disparityColormap = 'turbo';

% Depth limits in metres.

minDepth = 0.2;
maxDepth = 20;

% Point-cloud downsampling grid size in metres.

pointCloudGridSize = 0.02;

% Maximum number of SLAM ORB features.

slamMaxNumPoints = 1000;

% SLAM tracking parameters.

slamTrackFeatureRange = [30 100];

slamSkipMaxFrames = 20;

% Enable / disable processing modules.

enableStereoDepth    = true;
enablePointCloud     = true;
enableSLAM           = true;
enableVisualization  = true;

% Display disparity every N processed frames.

displayDisparityEvery = 10;

% Display point cloud every N processed frames.

displayPointCloudEvery = 10;


%% ============================================================
% 2. SELECT INPUT FILES
% =============================================================

if isempty(leftVideoPath)

    [file, path] = uigetfile( ...
        {'*.avi;*.mp4;*.mov', 'Video Files'}, ...
        'Select LEFT stereo video');

    if isequal(file, 0)
        error('Left video selection cancelled.');
    end

    leftVideoPath = fullfile(path, file);

end


if isempty(rightVideoPath)

    [file, path] = uigetfile( ...
        {'*.avi;*.mp4;*.mov', 'Video Files'}, ...
        'Select RIGHT stereo video');

    if isequal(file, 0)
        error('Right video selection cancelled.');
    end

    rightVideoPath = fullfile(path, file);

end


%% ============================================================
% 3. LOAD STEREO CALIBRATION
% =============================================================

if isempty(stereoCalibrationPath)

    [file, path] = uigetfile( ...
        {'*.mat', 'MATLAB Calibration Files'}, ...
        'Select stereo calibration file');

    if isequal(file, 0)
        error('Stereo calibration selection cancelled.');
    end

    stereoCalibrationPath = fullfile(path, file);

end


calibrationData = load(stereoCalibrationPath);

% Try to find the stereoParameters object automatically.

stereoParams = [];

variables = fieldnames(calibrationData);

for k = 1:numel(variables)

    currentVariable = calibrationData.(variables{k});

    if isa(currentVariable, 'stereoParameters')
        stereoParams = currentVariable;
        break;
    end

end

if isempty(stereoParams)

    error(['No stereoParameters object was found in the calibration ', ...
           'file.']);
end


%% ============================================================
% 4. INITIALIZE VIDEO READERS
% =============================================================

leftVideo  = VideoReader(leftVideoPath);
rightVideo = VideoReader(rightVideoPath);

fprintf('\n');
fprintf('LEFT VIDEO : %s\n', leftVideoPath);
fprintf('RIGHT VIDEO: %s\n', rightVideoPath);

fprintf('\n');
fprintf('Resolution : %d x %d\n', ...
    leftVideo.Width, leftVideo.Height);

fprintf('Left FPS   : %.2f\n', leftVideo.FrameRate);
fprintf('Right FPS  : %.2f\n', rightVideo.FrameRate);

fprintf('Duration   : %.2f seconds\n', ...
    min(leftVideo.Duration, rightVideo.Duration));


%% ============================================================
% 5. DETERMINE FRAME PROCESSING INTERVAL
% =============================================================

sourceFPS = min(leftVideo.FrameRate, rightVideo.FrameRate);

frameStep = max(1, round(sourceFPS / processingFPS));

actualProcessingFPS = sourceFPS / frameStep;

fprintf('\n');
fprintf('Source FPS          : %.2f\n', sourceFPS);
fprintf('Requested FPS       : %.2f\n', processingFPS);
fprintf('Frame step          : %d\n', frameStep);
fprintf('Actual processing FPS: %.2f\n', ...
    actualProcessingFPS);


%% ============================================================
% 6. INITIALIZE STEREO VISUAL SLAM
% =============================================================

% Rectified stereo camera intrinsics.

imageSize = [leftVideo.Height leftVideo.Width];

intrinsics = stereoParams.CameraParameters1.Intrinsics;

% Baseline in metres.

baseline = stereoParams.TranslationOfCamera2(1) / 1000;

% Create stereo visual SLAM object.

vslam = stereovslam( ...
    intrinsics, ...
    abs(baseline), ...
    MaxNumPoints=slamMaxNumPoints, ...
    TrackFeatureRange=slamTrackFeatureRange, ...
    SkipMaxFrames=slamSkipMaxFrames);


%% ============================================================
% 7. INITIALIZE VISUALIZATION
% =============================================================

if enableVisualization

    figure('Name', ...
        '360 Degree Stereo Vision - SLAM Prototype', ...
        'NumberTitle', 'off');

end


%% ============================================================
% 8. MAIN PROCESSING LOOP
% =============================================================

frameNumber = 0;
processedFrames = 0;

previousTime = tic;

while hasFrame(leftVideo) && hasFrame(rightVideo)

    frameNumber = frameNumber + 1;

    % Skip frames according to processing FPS.

    if mod(frameNumber - 1, frameStep) ~= 0

        readFrame(leftVideo);
        readFrame(rightVideo);

        continue;

    end


    %% --------------------------------------------------------
    % Read stereo frames
    % ---------------------------------------------------------

    leftFrame  = readFrame(leftVideo);
    rightFrame = readFrame(rightVideo);

    processedFrames = processedFrames + 1;


    %% --------------------------------------------------------
    % Stereo Rectification
    % ---------------------------------------------------------

    [leftRectified, rightRectified] = ...
        rectifyStereoImages( ...
        leftFrame, ...
        rightFrame, ...
        stereoParams);


    %% --------------------------------------------------------
    % Stereo Depth / Disparity
    % ---------------------------------------------------------

    disparityMap = [];

    points3D = [];

    if enableStereoDepth

        disparityMap = disparitySGM( ...
            leftRectified, ...
            rightRectified, ...
            DisparityRange=disparityRange);


        % Reconstruct 3-D points from disparity.

        points3D = reconstructScene( ...
            disparityMap, ...
            stereoParams);

        % Convert millimetres to metres.

        points3D = points3D / 1000;

    end


    %% --------------------------------------------------------
    % Depth Filtering
    % ---------------------------------------------------------

    validDepthMask = [];

    if enableStereoDepth

        Z = points3D(:,:,3);

        validDepthMask = ...
            isfinite(Z) & ...
            Z >= minDepth & ...
            Z <= maxDepth;

    end


    %% --------------------------------------------------------
    % Point Cloud Generation
    % ---------------------------------------------------------

    ptCloud = [];

    if enablePointCloud && enableStereoDepth

        xyz = points3D;

        xyzX = xyz(:,:,1);
        xyzY = xyz(:,:,2);
        xyzZ = xyz(:,:,3);

        validMask = validDepthMask;

        xyzPoints = [ ...
            xyzX(validMask), ...
            xyzY(validMask), ...
            xyzZ(validMask)];


        % Extract RGB values from left image.

        rgbImage = leftRectified;

        rgbPoints = reshape( ...
            rgbImage, ...
            [], ...
            3);

        rgbPoints = rgbPoints(validMask(:), :);


        % Create point cloud.

        ptCloud = pointCloud( ...
            xyzPoints, ...
            Color=rgbPoints);


        % Downsample point cloud.

        if ptCloud.Count > 0

            ptCloudDisplay = pcdownsample( ...
                ptCloud, ...
                'gridAverage', ...
                pointCloudGridSize);

        else

            ptCloudDisplay = ptCloud;

        end

    else

        ptCloudDisplay = [];

    end


    %% --------------------------------------------------------
    % Stereo Visual SLAM
    % ---------------------------------------------------------

    if enableSLAM

        % stereovslam expects rectified stereo images.

        addFrame( ...
            vslam, ...
            leftRectified, ...
            rightRectified);


        % Check SLAM status.

        status = checkStatus(vslam);


        % Status 0 indicates tracking failure.

        if status == uint8(0)

            warning( ...
                'SLAM tracking lost at frame %d.', ...
                frameNumber);

            break;

        end

    end


    %% --------------------------------------------------------
    % Visualization
    % ---------------------------------------------------------

    if enableVisualization


        % Disparity visualization.

        if enableStereoDepth && ...
                mod(processedFrames, displayDisparityEvery) == 0

            subplot(2,2,1);

            imagesc(disparityMap);

            axis image;

            colormap(gca, disparityColormap);

            colorbar;

            title('Disparity Map');

            xlabel('Pixel X');

            ylabel('Pixel Y');

        end


        % Depth visualization.

        if enableStereoDepth && ...
                mod(processedFrames, displayDisparityEvery) == 0

            subplot(2,2,2);

            depthDisplay = points3D(:,:,3);

            depthDisplay(~validDepthMask) = NaN;

            imagesc(depthDisplay, ...
                [minDepth maxDepth]);

            axis image;

            colorbar;

            title('Depth Map');

            xlabel('Pixel X');

            ylabel('Pixel Y');

        end


        % Point cloud visualization.

        if enablePointCloud && ...
                ~isempty(ptCloudDisplay) && ...
                mod(processedFrames, displayPointCloudEvery) == 0

            subplot(2,2,3);

            pcshow(ptCloudDisplay);

            title('Local 3-D Point Cloud');

            xlabel('X (m)');

            ylabel('Y (m)');

            zlabel('Z (m)');

            axis equal;

        end


        % SLAM visualization.

        if enableSLAM && ...
                hasNewKeyFrame(vslam)

            subplot(2,2,4);

            plot(vslam);

            title('Stereo Visual SLAM');

            drawnow limitrate;

        end

    end


    %% --------------------------------------------------------
    % Processing Performance
    % ---------------------------------------------------------

    elapsedTime = toc(previousTime);

    currentFPS = 1 / elapsedTime;

    previousTime = tic;

    fprintf( ...
        'Frame: %5d | Processed: %5d | FPS: %6.2f\n', ...
        frameNumber, ...
        processedFrames, ...
        currentFPS);


end


%% ============================================================
% 9. FINISH SLAM PROCESSING
% =============================================================

if enableSLAM

    fprintf('\n');
    fprintf('Waiting for SLAM processing to finish...\n');

    while ~isDone(vslam)

        if enableVisualization

            figure(gcf);

            plot(vslam);

            title('Finalizing Stereo Visual SLAM');

            drawnow;

        end

    end

end


%% ============================================================
% 10. EXTRACT SLAM MAP
% =============================================================

slamMapPoints = [];
cameraPoses = [];
viewIds = [];

if enableSLAM

    slamMapPoints = mapPoints(vslam);

    [cameraPoses, viewIds] = poses(vslam);

end


%% ============================================================
% 11. FINAL SLAM VISUALIZATION
% =============================================================

if enableSLAM

    figure( ...
        'Name', ...
        'Final Stereo SLAM Map', ...
        'NumberTitle', ...
        'off');

    plot(vslam);

    title('Final Stereo Visual SLAM Map');

end


%% ============================================================
% 12. DISPLAY RESULTS
% =============================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf('PROCESSING COMPLETE\n');
fprintf('============================================================\n');

fprintf('Frames processed : %d\n', processedFrames);


if enablePointCloud && ~isempty(ptCloud)

    fprintf( ...
        'Original point-cloud points : %d\n', ...
        ptCloud.Count);

    fprintf( ...
        'Downsampled point-cloud points: %d\n', ...
        ptCloudDisplay.Count);

end


if enableSLAM

    fprintf( ...
        'SLAM map points : %d\n', ...
        size(slamMapPoints,1));

    fprintf( ...
        'SLAM keyframes  : %d\n', ...
        numel(viewIds));

end

fprintf('============================================================\n');


%% ============================================================
% 13. FINAL POINT CLOUD
% =============================================================

if enablePointCloud && ~isempty(ptCloudDisplay)

    figure( ...
        'Name', ...
        'Final 3-D Point Cloud', ...
        'NumberTitle', ...
        'off');

    pcshow(ptCloudDisplay);

    title('Final Stereo 3-D Point Cloud');

    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');

    axis equal;

    grid on;

end