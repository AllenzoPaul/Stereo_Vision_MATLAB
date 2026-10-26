load("handshakeStereoParams.mat");

%Create Video Files and Video Player
videoFileLeft = "handshake_left.avi";
videoFileRight = "handshake_right.avi";

readerLeft = VideoReader(videoFileLeft);
readerRight = VideoReader(videoFileRight);
player = vision.VideoPlayer(Position=[20,200,740 560]);

%Read and Rectify Video Frames
frameLeft = readFrame(readerLeft);
frameRight = readFrame(readerRight);

imshowpair(frameRight,frameLeft, "montage")

[frameLeftRect, frameRightRect, reprojectionMatrix] = ...
    rectifyStereoImages(frameLeft, frameRight, stereoParams);

figure;
imshow(stereoAnaglyph(frameLeftRect, frameRightRect));
title("Rectified Video Frames");

[frameLeft, frameRight, reprojectionMatrix] = rectifyStereoImages(frameLeft,frameRight,stereoParams)

A = stereoAnaglyph(frameLeft, frameRight);
figure
imshow(A)
title("Red-Cyan composite view of the rectified stereo pair image");

flowModel = opticalFlowRAFT;

estimateFlow(flowModel,frameLeft);
flow = estimateFlow(flowModel,frameRight);
figure
imshow(frameLeft)
hold on
plot(flow,DecimationFactor=[40 40],ScaleFactor=0.75,color="g");
hold off
title("Plot of Optical Flow Vectors")

disparityMapRAFT = abs(flow.Vx);
disparityMapRAFT = imresize(disparityMapRAFT, size(frameLeft,[1 2]));

% Modify thresholds based on specific data set
minDisparity = 1;   % in pixels
imageMargin  = 10;  % in pixels

[H,W,~] = size(frameLeft);
[X,Y] = meshgrid(1:W,1:H);
X2 = X + imresize(flow.Vx, size(frameLeft,[1 2]));
Y2 = Y + imresize(flow.Vy, size(frameLeft,[1 2]));

% Determine valid optical flow mask
validFlow = X2>=1 & X2<=W & Y2>=1 & Y2<=H & disparityMapRAFT > minDisparity;

% Replace invalid positions in the disparity map with nan
disparityMapRAFT(~validFlow) = nan;

% Mark image borders as invalid
disparityMapRAFT(:,end-imageMargin:end) = nan;
disparityMapRAFT(end-imageMargin:end,:) = nan;
disparityMapRAFT(1:imageMargin,:) = nan;
disparityMapRAFT(:,1:imageMargin) = nan;

figure
imagesc(disparityMapRAFT)
colorbar
axis image
title("Refined Disparity Map from Optical Flow")

disparityMapRAFT = abs(flow.Vx);
disparityMapRAFT = imresize(disparityMapRAFT, size(frameLeft,[1 2]));






% Modify thresholds based on specific data set
minDisparity = 1;   % in pixels
imageMargin  = 10;  % in pixels

[H,W,~] = size(frameLeft);
[X,Y] = meshgrid(1:W,1:H);
X2 = X + imresize(flow.Vx, size(frameLeft,[1 2]));
Y2 = Y + imresize(flow.Vy, size(frameLeft,[1 2]));

% Determine valid optical flow mask
validFlow = X2>=1 & X2<=W & Y2>=1 & Y2<=H & disparityMapRAFT > minDisparity;

% Replace invalid positions in the disparity map with nan
disparityMapRAFT(~validFlow) = nan;

% Mark image borders as invalid
disparityMapRAFT(:,end-imageMargin:end) = nan;
disparityMapRAFT(end-imageMargin:end,:) = nan;
disparityMapRAFT(1:imageMargin,:) = nan;
disparityMapRAFT(:,1:imageMargin) = nan;

figure
imagesc(disparityMapRAFT)
colorbar
axis image
title("Refined Disparity Map from Optical Flow")




imLeftGray = im2gray(frameLeft);
imRightGray = im2gray(frameRight);
disparityMapSGM = disparitySGM(imLeftGray,imRightGray);


figure

subplot(2,1,1)
imagesc(disparityMapRAFT)
colorbar
axis image
title("Disparity Map using RAFT Optical Flow")

subplot(2,1,2)
imagesc(disparityMapSGM)
colorbar
axis image
title("Disparity Map using Semi-Global Matching")


xyzPointsRAFT = reconstructScene(disparityMapRAFT,reprojectionMatrix);
xyzPointsSGM  = reconstructScene(disparityMapSGM,reprojectionMatrix);



depthMapRAFT = xyzPointsRAFT(:,:,3);
depthMapSGM  = xyzPointsSGM(:,:,3);

figure
subplot(2,1,1)
imagesc(log(1 + depthMapRAFT))
axis image
colorbar
title("Depth Map (log-scale) - RAFT")

subplot(2,1,2)
imagesc(log(1 + depthMapSGM))
axis image
colorbar
title("Depth Map (log-scale) - SGM")





[H,W,~] = size(frameLeft);
[X,Y] = meshgrid(1:W,1:H);
u = X(:);
v = Y(:);
linearIdx = sub2ind([H,W], v, u);
R = frameLeft(:,:,1);
G = frameLeft(:,:,2);
B = frameLeft(:,:,3);
colors = [R(linearIdx), G(linearIdx), B(linearIdx)];

ptCloudRAFT = pointCloud(reshape(xyzPointsRAFT, [H*W 3]), Color=colors);
ptCloudSGM  = pointCloud(reshape(xyzPointsSGM, [H*W 3]), Color=colors);



% Select thresholds using the sliders to remove outliers and retain 
% objects of interest, by visually inspecting the visualized point cloud.
minXThreshold = -3000;
maxXThreshold = 5000;
minYThreshold = -2000;
maxYThreshold = 2000;
maxZThreshold = 6200;
roi = [minXThreshold maxXThreshold minYThreshold  maxYThreshold 0  maxZThreshold];

% Apply selected thresholds on the point clouds
indicesRAFT = findPointsInROI(ptCloudRAFT,roi);
ptCloudRAFTROI = select(ptCloudRAFT,indicesRAFT);
indicesSGM = findPointsInROI(ptCloudSGM,roi);
ptCloudSGMROI = select(ptCloudSGM,indicesSGM);

% Visualize the output point clouds
figure
subplot(1,1,1)
pcshow(ptCloudRAFTROI, VerticalAxis="y", VerticalAxisDir="down");
xlabel("X")
ylabel("Y")
zlabel("Z")
title("Stereo Reconstruction using RAFT")
view(0,-80)

figure
subplot(1,1,1)
pcshow(ptCloudSGMROI, VerticalAxis="y", VerticalAxisDir="down");
xlabel("X")
ylabel("Y")
zlabel("Z")
title("Stereo Reconstruction using SGM")
view(0,-80)