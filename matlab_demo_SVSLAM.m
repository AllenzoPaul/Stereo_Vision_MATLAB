imgFolderLeft = '~/datasets/utias/images/left';
imgFolderRight = '~/datasets/utias/images/right';

imdsLeft = imageDatastore(imgFolderLeft);
imdsRight = imageDatastore(imgFolderRight);

focalLength = [387.777 387.777];  
principalPoint = [257.446 197.718];  
imageSize = [384 512];            
intrinsics = cameraIntrinsics(focalLength,principalPoint,imageSize);
baseline = 0.239965; 

vslam = stereovslam(intrinsics,baseline,MaxNumPoints=600, ...
    TrackFeatureRange=[30 120],SkipMaxFrames=5);

for i = 1:numel(imdsLeft.Files)
    leftImage = readimage(imdsLeft,i);
    rightImage = readimage(imdsRight,i);
    addFrame(vslam,leftImage,rightImage);

    if hasNewKeyFrame(vslam)
        % Query 3-D map points and camera poses
        xyzPoints = mapPoints(vslam);
        [camPoses,viewIds] = poses(vslam);

        % Display 3-D map points and camera trajectory
        plot(vslam);
    end

    % Get current status of system
    status = checkStatus(vslam);

    % Stop adding frames when tracking is lost
    if status == uint8(0)
        break
    end
end 

while ~isDone(vslam)
    plot(vslam)
end

