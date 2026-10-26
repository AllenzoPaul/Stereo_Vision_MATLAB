while hasFrame(readerLeft) && hasFrame(readerRight)
    % Read the frames.
    frameLeft = readFrame(readerLeft);
    frameRight = readFrame(readerRight);

    % Rectify the frames.
    [frameLeftRect, frameRightRect] = ...
        rectifyStereoImages(frameLeft, frameRight, stereoParams);

    % Convert to grayscale.
    frameLeftGray  = im2gray(frameLeftRect);
    frameRightGray = im2gray(frameRightRect);

    % Compute disparity. 
    disparityMap = disparitySGM(frameLeftGray, frameRightGray);

    % Reconstruct 3-D scene.
    points3D = reconstructScene(disparityMap, reprojectionMatrix);
    points3D = points3D ./ 1000;
    ptCloud = pointCloud(points3D, Color=frameLeftRect);
    view(player3D, ptCloud);

    % Detect people.
    bboxes = detect(peopleDetector,frameLeftGray);

    if ~isempty(bboxes)
        % Find the centroids of detected people.
        centroids = [round(bboxes(:, 1) + bboxes(:, 3) / 2), ...
            round(bboxes(:, 2) + bboxes(:, 4) / 2)];

        % Find the 3-D world coordinates of the centroids.
        centroidsIdx = sub2ind(size(disparityMap), centroids(:, 2), centroids(:, 1));
        X = points3D(:, :, 1);
        Y = points3D(:, :, 2);
        Z = points3D(:, :, 3);
        centroids3D = [X(centroidsIdx), Y(centroidsIdx), Z(centroidsIdx)];

        % Find the distances from the camera in meters.
        dists = sqrt(sum(centroids3D .^ 2, 2));

        % Display the detect people and their distances.
        labels = dists+" meters";
        dispFrame = insertObjectAnnotation(frameLeftRect, "rectangle", bboxes,...
            labels);
    else
        dispFrame = frameLeftRect;
    end

    % Display the frame.
    step(player, dispFrame);
end
