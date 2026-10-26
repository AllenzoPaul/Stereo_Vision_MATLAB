clear;
clc;
close all;

%% ============================================================
% USER CONFIGURATION
% =============================================================

% ------------------------------------------------------------
% Processing FPS
% ------------------------------------------------------------
% USER CONFIGURATION:
% Controls how frequently stereo frames are processed.
%
% Source video = 10 FPS.
%
% 10 = approximately every frame
% 5  = approximately every second frame
% 2  = approximately 2 frames per second

processingFPS = 7;


% ------------------------------------------------------------
% Disparity Range
% ------------------------------------------------------------
% USER CONFIGURATION:
% Defines the disparity search range used by SGM.
% Values should normally be multiples of 16.

disparityRange = [0 128];

% ------------------------------------------------------------
% Disparity Visualization Colormap
% ------------------------------------------------------------
% USER CONFIGURATION:
% Controls the colour scheme used to display disparity.
%
% Examples:
% 'turbo'
% 'parula'
% 'jet'
% 'hot'
% 'gray'
% 'copper'

disparityColormap = 'turbo';


% ------------------------------------------------------------
% Minimum Depth
% ------------------------------------------------------------
% USER CONFIGURATION:
% Minimum valid reconstructed depth.
%
% Units: meters.

minDepth = 0.2;


% ------------------------------------------------------------
% Maximum Depth
% ------------------------------------------------------------
% USER CONFIGURATION:
% Maximum valid reconstructed depth.
%
% Units: meters.

maxDepth = 20;


% ------------------------------------------------------------
% Point Cloud Downsampling
% ------------------------------------------------------------
% USER CONFIGURATION:
% Controls point-cloud visualization density.
%
% 1 = no downsampling
% 2 = moderate reduction
% 4 = stronger reduction

pointCloudStep = 2;


% ------------------------------------------------------------
% Point Cloud Grid Size
% ------------------------------------------------------------
% USER CONFIGURATION:
% Controls spatial resolution of point-cloud downsampling.
%
% Units: meters.
%
% Smaller value = more points.
% Larger value = fewer points.

pointCloudGridSize = 0.01;


% ------------------------------------------------------------
% Generate Point Cloud
% ------------------------------------------------------------
% USER CONFIGURATION:
%
% true  = calculate and display 3D point cloud
% false = skip all point-cloud processing
%
% Set to false for disparity-only demonstrations.

generatePointCloud = false;


% ------------------------------------------------------------
% Visualization
% ------------------------------------------------------------
% USER CONFIGURATION:
%
% true  = display processing results
% false = disable visualization

showVisualization = true;


%% ============================================================
% 1. LOCATE MATLAB STEREO VISION SAMPLE FILES
% =============================================================

leftVideo = dir(fullfile(toolboxdir('vision'), ...
    '**', 'handshake_left.avi'));

rightVideo = dir(fullfile(toolboxdir('vision'), ...
    '**', 'handshake_right.avi'));

stereoFile = dir(fullfile(toolboxdir('vision'), ...
    '**', 'handshakeStereoParams.mat'));


%% ============================================================
% 2. VALIDATE REQUIRED FILES
% =============================================================

if isempty(leftVideo)
    error('Left stereo video was not found.');
end

if isempty(rightVideo)
    error('Right stereo video was not found.');
end

if isempty(stereoFile)
    error('Stereo calibration file was not found.');
end


%% ============================================================
% 3. CONSTRUCT FILE PATHS
% =============================================================

leftPath = fullfile( ...
    leftVideo(1).folder, ...
    leftVideo(1).name);

rightPath = fullfile( ...
    rightVideo(1).folder, ...
    rightVideo(1).name);

stereoPath = fullfile( ...
    stereoFile(1).folder, ...
    stereoFile(1).name);


%% ============================================================
% 4. OPEN STEREO VIDEO STREAMS
% =============================================================

leftReader = VideoReader(leftPath);
rightReader = VideoReader(rightPath);


%% ============================================================
% 5. DISPLAY VIDEO INFORMATION
% =============================================================

fprintf('\n');
fprintf('============================================\n');
fprintf('       STEREO VIDEO INFORMATION\n');
fprintf('============================================\n');

fprintf('Resolution: %d x %d pixels\n', ...
    leftReader.Width, ...
    leftReader.Height);

fprintf('Left video FPS: %.2f\n', ...
    leftReader.FrameRate);

fprintf('Right video FPS: %.2f\n', ...
    rightReader.FrameRate);

fprintf('Left video duration: %.2f seconds\n', ...
    leftReader.Duration);

fprintf('Right video duration: %.2f seconds\n', ...
    rightReader.Duration);


%% ============================================================
% 6. LOAD STEREO CAMERA CALIBRATION
% =============================================================

load(stereoPath);

fprintf('\nStereo calibration loaded successfully.\n');


%% ============================================================
% 7. DISPLAY CALIBRATION INFORMATION
% =============================================================

whos stereoParams


%% ============================================================
% 8. VALIDATE PROCESSING CONFIGURATION
% =============================================================

if processingFPS <= 0
    error('processingFPS must be greater than zero.');
end

if processingFPS > leftReader.FrameRate

    warning(['Processing FPS is higher than the source video FPS. ', ...
             'Setting processing FPS to source FPS.']);

    processingFPS = leftReader.FrameRate;

end


%% ============================================================
% 9. CALCULATE PROCESSING INTERVAL
% =============================================================

processingInterval = 1 / processingFPS;

fprintf('\n');
fprintf('============================================\n');
fprintf('       PROCESSING CONFIGURATION\n');
fprintf('============================================\n');

fprintf('Source FPS: %.2f\n', ...
    leftReader.FrameRate);

fprintf('Processing FPS: %.2f\n', ...
    processingFPS);

fprintf('Processing interval: %.3f seconds\n', ...
    processingInterval);

fprintf('Disparity range: [%d %d]\n', ...
    disparityRange(1), ...
    disparityRange(2));

fprintf('Minimum depth: %.2f m\n', ...
    minDepth);

fprintf('Maximum depth: %.2f m\n', ...
    maxDepth);

fprintf('Point cloud step: %d\n', ...
    pointCloudStep);

fprintf('Point cloud grid size: %.3f m\n', ...
    pointCloudGridSize);

fprintf('Generate point cloud: %s\n', ...
    string(generatePointCloud));


%% ============================================================
% 10. PREPARE VIDEO PROCESSING
% =============================================================

leftReader.CurrentTime = 0;
rightReader.CurrentTime = 0;

frameNumber = 0;

processedFrames = 0;

nextProcessingTime = 0;


%% ============================================================
% 11. CREATE VISUALIZATION WINDOWS
% =============================================================

if showVisualization

    stereoFigure = figure( ...
        'Name', 'Stereo Vision Processing');


    if generatePointCloud

        pointCloudFigure = figure( ...
            'Name', '3D Point Cloud');

    end

end


%% ============================================================
% 12. START VIDEO PROCESSING
% =============================================================

fprintf('\n');
fprintf('============================================\n');
fprintf('       STARTING VIDEO PROCESSING\n');
fprintf('============================================\n');


while hasFrame(leftReader) && hasFrame(rightReader)


    %% --------------------------------------------------------
    % 12A. READ NEXT STEREO FRAME
    % ---------------------------------------------------------

    leftFrame = readFrame(leftReader);

    rightFrame = readFrame(rightReader);

    frameNumber = frameNumber + 1;

    currentTime = leftReader.CurrentTime;


    %% --------------------------------------------------------
    % 12B. FRAME-RATE CONTROL
    % ---------------------------------------------------------

    if currentTime >= nextProcessingTime


        %% ----------------------------------------------------
        % 12C. STEREO IMAGE RECTIFICATION
        % -----------------------------------------------------

        [leftRectified, rightRectified] = ...
            rectifyStereoImages( ...
            leftFrame, ...
            rightFrame, ...
            stereoParams);


        %% ----------------------------------------------------
        % 12D. CONVERT IMAGES TO GRAYSCALE
        % -----------------------------------------------------

        leftGray = im2gray(leftRectified);

        rightGray = im2gray(rightRectified);


        %% ----------------------------------------------------
        % 12E. CALCULATE DISPARITY
        % -----------------------------------------------------

        disparityMap = disparitySGM( ...
            leftGray, ...
            rightGray, ...
            'DisparityRange', disparityRange);


        %% ====================================================
        % POINT CLOUD GENERATION SECTION — START
        %
        % Set generatePointCloud = false above to skip this
        % entire section.
        % ====================================================

        if generatePointCloud


            %% ------------------------------------------------
            % 12F. RECONSTRUCT 3D SCENE
            % -------------------------------------------------

            points3D = reconstructScene( ...
                disparityMap, ...
                stereoParams);


            %% ------------------------------------------------
            % 12G. CONVERT 3D COORDINATES TO METERS
            % -------------------------------------------------

            points3D = points3D / 1000;


            %% ------------------------------------------------
            % 12H. EXTRACT DEPTH
            % -------------------------------------------------

            depthMap = points3D(:, :, 3);


            %% ------------------------------------------------
            % 12I. CREATE VALID DEPTH MASK
            % -------------------------------------------------

            validDepth = ...
                isfinite(depthMap) & ...
                depthMap >= minDepth & ...
                depthMap <= maxDepth;


            %% ------------------------------------------------
            % 12J. EXTRACT XYZ COORDINATES
            % -------------------------------------------------

            X = points3D(:, :, 1);

            Y = points3D(:, :, 2);

            Z = points3D(:, :, 3);


            X = X(validDepth);

            Y = Y(validDepth);

            Z = Z(validDepth);


            xyzPoints = [X Y Z];


            %% ------------------------------------------------
            % 12K. EXTRACT RGB COLOUR
            % -------------------------------------------------

            rgbImage = reshape( ...
                leftRectified, ...
                [], ...
                3);


            validMask = validDepth(:);


            pointColors = ...
                rgbImage(validMask, :);


            %% ------------------------------------------------
            % 12L. CREATE POINT CLOUD OBJECT
            % -------------------------------------------------

            ptCloud = pointCloud( ...
                xyzPoints, ...
                'Color', ...
                pointColors);


            %% ------------------------------------------------
            % 12M. DOWNSAMPLE POINT CLOUD
            % -------------------------------------------------

            if pointCloudStep > 1

                ptCloudDisplay = pcdownsample( ...
                    ptCloud, ...
                    'gridAverage', ...
                    pointCloudGridSize);

            else

                ptCloudDisplay = ptCloud;

            end


        end


        % ====================================================
        % POINT CLOUD GENERATION SECTION — END
        % ====================================================


        %% ----------------------------------------------------
        % 12N. DISPLAY STEREO + DISPARITY
        % -----------------------------------------------------

        if showVisualization && ...
                isvalid(stereoFigure)

            figure(stereoFigure);


            subplot(1,3,1);

            imshow(leftRectified);

            title(sprintf( ...
                'Left | Frame %d', ...
                frameNumber));


            subplot(1,3,2);

            imshow(rightRectified);

            title('Right');


            subplot(1,3,3);

            imshow( ...
                disparityMap, ...
                disparityRange);


            % Apply selected disparity colormap.

            colormap(gca, disparityColormap);


            % Add numerical disparity scale.

            colorbar;


            % Force colorbar limits to match disparity range.

            clim(disparityRange);


            title('Disparity');


            sgtitle(sprintf( ...
                'Stereo Vision | %.2f s | Processed: %d', ...
                currentTime, ...
                processedFrames + 1));


            drawnow limitrate;

        end


        %% ----------------------------------------------------
        % 12O. DISPLAY 3D POINT CLOUD
        % -----------------------------------------------------

        if generatePointCloud && ...
                showVisualization && ...
                isvalid(pointCloudFigure)

            figure(pointCloudFigure);


            pcshow(ptCloudDisplay);


            xlabel('X (m)');
            ylabel('Y (m)');
            zlabel('Z (m)');


            title(sprintf( ...
                '3D Point Cloud | Frame %d | %.2f s', ...
                frameNumber, ...
                currentTime));


            axis equal;

            grid on;


            drawnow limitrate;

        end


        %% ----------------------------------------------------
        % 12P. UPDATE PROCESSING COUNTERS
        % -----------------------------------------------------

        processedFrames = processedFrames + 1;


        nextProcessingTime = ...
            nextProcessingTime + processingInterval;


    end

end


%% ============================================================
% 13. PROCESSING SUMMARY
% =============================================================

fprintf('\n');
fprintf('============================================\n');
fprintf('       PROCESSING COMPLETED\n');
fprintf('============================================\n');

fprintf('Total frames read: %d\n', ...
    frameNumber);

fprintf('Total frames processed: %d\n', ...
    processedFrames);

fprintf('Processing frequency: %.2f FPS\n', ...
    processingFPS);

fprintf('Point cloud generation: %s\n', ...
    string(generatePointCloud));

fprintf('Video duration: %.2f seconds\n', ...
    leftReader.Duration);

fprintf('============================================\n');