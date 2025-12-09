% VI-SLAM的主程序，运行这个即开始仿真
clear
clc

% TODO:阅读代码，增加注释，搞清楚显示的figure 3是什么意思
% TODO:实时同步显示ground truth路径和估计出来的路径
% TODO:调整攻击参数，观察攻击效果
% TODO:（实时）绘制ground truth的坐标位置、运动速度和姿态曲线以及估计出来的坐标位置、运动速度和姿态曲线，比较攻击前后的差异
% TODO:打印/绘图算法的中间过程。到底是哪一步开始出现了错误？给出一个insight



uavData = helperDownloadSLAMData(); % 这行代码load了仿真数据，请在这个函数中修改

images     = uavData.images; % 仿真图像序列
intrinsics = uavData.intrinsics; % 相机的内参
timeStamps = uavData.timeStamps; % 每帧图像的时间戳

imuParams = factorIMUParameters(SampleRate=100,GyroscopeNoise=0.1, ...
            GyroscopeBiasNoise=3e-3,AccelerometerNoise=0.3, ...
            AccelerometerBiasNoise=1e-3,ReferenceFrame="ENU");

startBiasIdx  = 40; %  Index from which the bias estimation starts
startFrameIdx = 150; % Index from which the SLAM pipline starts
initBias = helperEstimateInitialBias(uavData,startBiasIdx,startFrameIdx,imuParams);

% Set random seed for reproducibility
rng(0);

% Detect and extract ORB features
scaleFactor = 1.2;
numLevels   = 8;
numPoints   = 5000;
numSkipFrames = 20;
numPointsKeyFrame = 200;

currI = images{1, startFrameIdx};
[preFeatures, prePoints] = helperDetectAndExtractFeatures(currI, scaleFactor, numLevels, numPoints, intrinsics); 

currFrameIdx = startFrameIdx + 1;
firstI       = currI; % Preserve the first frame 

isMapInitialized  = false;

% Map initialization loop
while ~isMapInitialized && currFrameIdx < size(images,2)

    currI = images{currFrameIdx}; % 从图像序列中读取当前帧图像currI

    [currFeatures, currPoints] = helperDetectAndExtractFeatures(currI, scaleFactor, numLevels, numPoints, intrinsics); % 在当前帧中检测特征点并提取特征描述子
    
    currFrameIdx = currFrameIdx + 1;
    
    % Find putative feature matches
    indexPairs = matchFeatures(preFeatures, currFeatures, Unique=true, MaxRatio=0.5, MatchThreshold=80);

    % If not enough matches are found, check the next frame
    minMatches = 100;
    if size(indexPairs, 1) < minMatches
        continue
    end
    
    preMatchedPoints  = prePoints(indexPairs(:,1),:);
    currMatchedPoints = currPoints(indexPairs(:,2),:);
    
    % Compute homography and evaluate reconstruction
    [tformH, scoreH, inliersIdxH] = helperComputeHomography(preMatchedPoints, currMatchedPoints);

    % Compute fundamental matrix and evaluate reconstruction
    [tformF, scoreF, inliersIdxF] = helperComputeFundamentalMatrix(preMatchedPoints, currMatchedPoints);
    
    % Select the model based on a heuristic
    ratio = scoreH/(scoreH + scoreF);
    ratioThreshold = 0.45;
    if ratio > ratioThreshold
        inlierTformIdx = inliersIdxH;
        tform          = tformH;
    else
        inlierTformIdx = inliersIdxF;
        tform          = tformF;
    end

    % Computes the camera location up to scale. Use half of the 
    % points to reduce computation
    inlierPrePoints  = preMatchedPoints(inlierTformIdx);
    inlierCurrPoints = currMatchedPoints(inlierTformIdx);
    [relPose, validFraction] = estrelpose(tform, intrinsics, ...
        inlierPrePoints(1:2:end), inlierCurrPoints(1:2:end));
    
    % If not enough inliers are found, move to the next frame
    if validFraction < 0.9 || numel(relPose)~=1
        continue
    end
    
    % Triangulate two views to obtain 3-D map points
    minParallax = 1; % In degrees
    [isValid, xyzWorldPoints, inlierTriangulationIdx] = helperTriangulateTwoFrames(...
        rigidtform3d, relPose, inlierPrePoints, inlierCurrPoints, intrinsics, minParallax);
    
    if ~isValid
        continue
    end
    
    % Get the original index of features in the two key frames
    indexPairs = indexPairs(inlierTformIdx(inlierTriangulationIdx),:);
    
    isMapInitialized = true;
    
    disp(['Map initialized with frame ', num2str(startFrameIdx),' and frame ', num2str(currFrameIdx-1)])
end % End of map initialization loop


if isMapInitialized
    % Show matched features
    hfeature = showMatchedFeatures(firstI, currI, prePoints(indexPairs(:,1)), ...
        currPoints(indexPairs(:, 2)), "Montage");
else
    error('Unable to initialize the map.')
end


% Create an empty imageviewset object to store key frames
vSetKeyFrames = imageviewset;

% Create an empty worldpointset object to store 3-D map points
mapPointSet   = worldpointset;

% Add the first key frame. Place the camera associated with the first 
% key frame at the origin, oriented along the Z-axis
preViewId     = 1;
vSetKeyFrames = addView(vSetKeyFrames, preViewId, rigidtform3d, Points=prePoints,...
    Features=preFeatures.Features);

% Add the second key frame
currViewId    = 2;
vSetKeyFrames = addView(vSetKeyFrames, currViewId, relPose, Points=currPoints,...
    Features=currFeatures.Features);

% Add connection between the first and the second key frame
vSetKeyFrames = addConnection(vSetKeyFrames, preViewId, currViewId, relPose, Matches=indexPairs);

% Add 3-D map points
[mapPointSet, newPointIdx] = addWorldPoints(mapPointSet, xyzWorldPoints);

% Add observations of the map points
preLocations  = prePoints.Location;
currLocations = currPoints.Location;
preScales     = prePoints.Scale;
currScales    = currPoints.Scale;

% Add image points corresponding to the map points in the first key frame
mapPointSet   = addCorrespondences(mapPointSet, preViewId, newPointIdx, indexPairs(:,1));

% Add image points corresponding to the map points in the second key frame
mapPointSet   = addCorrespondences(mapPointSet, currViewId, newPointIdx, indexPairs(:,2));


% Load the bag of features data created offline
bofData         = load("uavBoF.mat");
encodedImages   = [];

% Add features of the first two key frames to the database
encodedImages = [encodedImages; encode(bofData.bag,firstI,Verbose=false)];
encodedImages = [encodedImages; encode(bofData.bag,currI ,Verbose=false)];


% Initialize the factor graph
%   - viewToNode : array to convert imageViewSet indices to factorGraph pose id
%   - pointToNode: array to convert worldPointSet indices to factorGraph 3d point id
[fGraph, refinedAbsPoses, refinedPoints, viewToNode, pointToNode] = helperInitFactorGraph(vSetKeyFrames, mapPointSet, intrinsics);

% Update key frames with the refined poses
vSetKeyFrames = updateView(vSetKeyFrames, refinedAbsPoses);
vSetKeyFrames = updateConnection(vSetKeyFrames, preViewId, currViewId, relPose);

% Update map points with the refined positions
mapPointSet = updateWorldPoints(mapPointSet, newPointIdx, refinedPoints);

% Update view direction and depth 
mapPointSet = updateLimitsAndDirection(mapPointSet, newPointIdx, vSetKeyFrames.Views);

% Update representative view
mapPointSet = updateRepresentativeView(mapPointSet, newPointIdx, vSetKeyFrames.Views);

% Update factor graph with refined poses 
fgRefinedPose = double([refinedAbsPoses.AbsolutePose(1).Translation rotm2quat(refinedAbsPoses.AbsolutePose(1).R)]);
nodeState(fGraph,1,fgRefinedPose);
fgRefinedPose = double([refinedAbsPoses.AbsolutePose(2).Translation rotm2quat(refinedAbsPoses.AbsolutePose(2).R)]);
nodeState(fGraph,2,fgRefinedPose);

% Update factor graph with refined points
ptsIds = nodeIDs(fGraph,NodeType="POINT_XYZ");
nodeState(fGraph,ptsIds,refinedPoints);

% Visualize matched features in the current frame
close(hfeature.Parent.Parent);
featurePlot   = helperVisualizeMatchedFeatures(currI, currPoints(indexPairs(:,2)));


% Visualize initial map points and camera trajectory
mapPlot       = helperVisualizeMotionAndStructureVI(vSetKeyFrames, mapPointSet);

showPlotLegend(mapPlot);


% ViewId of the current key frame
currKeyFrameId   = currViewId;

% ViewId of the last key frame
lastKeyFrameId   = currViewId;

% Index of the last key frame in the input image sequence
lastKeyFrameIdx  = currFrameIdx - 1; 

% Indices of all the key frames in the input image sequence
addedFramesIdx   = [1; lastKeyFrameIdx];



% Main loop
isLastFrameKeyFrame = true;
isIMUInitialized    = false;
isLoopClosed        = false;

keyTimeStamps   = [timeStamps.imageTimeStamps(startFrameIdx); timeStamps.imageTimeStamps(currFrameIdx-1)];
keyFrameToFrame = [startFrameIdx; currFrameIdx-1];  % Array to convert keyframe ids to frame id
biasToNode      = []; % Array to convert imageViewSet indices to factorGraph bias id
velToNode       = []; % Array to convert imageViewSet indices to factorGraph velocity id

loopCtr = 0; % Loop closure rolling window counter
scale   = 0; % Placeholder for IMU-Camera scale

% Factor graph optimization parameters
fgso = factorGraphSolverOptions;
fgso.MaxIterations           = 20;
fgso.VerbosityLevel          = 0;
fgso.FunctionTolerance       = 1e-5;
fgso.GradientTolerance       = 1e-5;
fgso.StepTolerance           = 1e-5;
fgso.TrustRegionStrategyType = 0;

% 用于记录solInfo的FinalCost
finalCostList = [];

while currFrameIdx < 400 %size(images,2)  % TODO
    disp(['currFrameIdx: ',num2str(currFrameIdx)])

    if ~isIMUInitialized && currKeyFrameId>50
        camPoses = poses(vSetKeyFrames);

        % Estimate the gravity rotation and pose scale that helps in transforming input camera poses to the 
        % local navigation reference frame of IMU
        [gDir,scale] = helperAlignIMUCamera(camPoses,uavData,imuParams,...
            keyFrameToFrame,currKeyFrameId);

        if scale>0.3

            % Compare the scaled poses to the ground truth
            alignementPlot = helperDrawScaledandRotatedTraj(uavData,camPoses,scale,startFrameIdx,keyFrameToFrame);

            % Scale estimation was successful
            isIMUInitialized = true;
    
            xyzPoints1 = mapPointSet.WorldPoints;
            xyzPoints2 = nodeState(fGraph,nodeIDs(fGraph,NodeType="POINT_XYZ"));
    
            % Transform and scale the input camera poses and XYZ points using the estimated gravity direction and pose scale.
            [updatedCamPoses,updatedXYZPoints1,updatedXYZPoints2] = helperTransformToIMU(...
                camPoses,xyzPoints1,xyzPoints2,gDir,scale); %two sets of points because we can't remove nodes from graphs yet
    
            % Update mapPointSet and vSetKeyFrames
            vSetKeyFrames = updateView(vSetKeyFrames, updatedCamPoses);
            mapPointSet = updateWorldPoints(mapPointSet, (1:mapPointSet.Count), updatedXYZPoints1);
    
            % Update the poses and 3D points values in the factor graph
            for i=1:height(updatedCamPoses)
                pose = updatedCamPoses.AbsolutePose(i); 
                nodeState(fGraph, viewToNode(i), double([pose.Translation rotm2quat(pose.R)]));
            end
            nodeState(fGraph,nodeIDs(fGraph,NodeType="POINT_XYZ"),updatedXYZPoints2);
    
            % Update the factor graph by linking the IMU mesurements
            veloId = generateNodeID(fGraph,length(viewToNode));
            biasId = generateNodeID(fGraph,length(viewToNode));
    
            velToNode  = [velToNode;veloId'];
            biasToNode = [biasToNode;biasId'];
    
            initVelPrior  = factorVelocity3Prior(velToNode(1));
            initBiasPrior = factorIMUBiasPrior(biasToNode(1));
    
            addFactor(fGraph,initVelPrior);
            addFactor(fGraph,initBiasPrior);
    
            nodeState(fGraph, biasToNode(1), initBias);
    
            for i=2:length(viewToNode)
                imuMeasurements = helperExtractIMUMeasurements(uavData, keyFrameToFrame(i-1), keyFrameToFrame(i));
                imuId = [viewToNode(i-1),velToNode(i-1),biasToNode(i-1), ...
                        viewToNode(i)  ,velToNode(i)  ,biasToNode(i)];
                fIMU = factorIMU(imuId,imuMeasurements.gyro,imuMeasurements.accel,imuParams,SensorTransform=uavData.camToIMUTransform); % 提取IMU因子
                addFactor(fGraph,fIMU);
            end
    
            optimize(fGraph,fgso);
            
        end
    end

    % Extract ORB features from the current image
    currI = images{1, currFrameIdx};
    [currFeatures, currPoints] = helperDetectAndExtractFeatures(currI, scaleFactor, numLevels, ...
        numPoints, intrinsics);

    % Track the last key frame
    [currPose, mapPointsIdx, featureIdx] = helperTrackLastKeyFrameVI(mapPointSet, ...
        vSetKeyFrames.Views, currFeatures, currPoints, lastKeyFrameId, intrinsics, scaleFactor);

    % Track the local map and check if the current frame is a key frame.
    [localKeyFrameIds, currPose, mapPointsIdx, featureIdx, isKeyFrame] = ...
        helperTrackLocalMapVI(mapPointSet, vSetKeyFrames, mapPointsIdx, ...
        featureIdx, currPose, currFeatures, currPoints, intrinsics, scaleFactor, numLevels, ...
        isLastFrameKeyFrame, lastKeyFrameIdx, currFrameIdx, numSkipFrames, numPointsKeyFrame);

    % Visualize matched features
    updatePlot(featurePlot, currI, currPoints(featureIdx));
    
    if ~isKeyFrame
        isLastFrameKeyFrame = false;
        currFrameIdx        = currFrameIdx + 1;
        continue
    else
        isLastFrameKeyFrame = true;
        keyFrameToFrame = [keyFrameToFrame; currFrameIdx];
        encodedImages   = [encodedImages; encode(bofData.bag,currI,Verbose=false)];
        keyTimeStamps   = [keyTimeStamps; timeStamps.imageTimeStamps(currFrameIdx)];
        loopCtr         = loopCtr+1;
    end


    % Update current key frame ID
    currKeyFrameId  = currKeyFrameId + 1;

    % Extract IMU measurements between two KFs
    imuMeasurements = helperExtractIMUMeasurements(uavData, keyFrameToFrame(end-1), keyFrameToFrame(end));
    
    % Add the new key frame and IMU data
    [fGraph, viewToNode, velToNode, biasToNode, mapPointSet, vSetKeyFrames] = helperAddNewKeyFrame(...
        fGraph, viewToNode, pointToNode, velToNode, biasToNode, mapPointSet, vSetKeyFrames, imuMeasurements, ...
        currPose, currFeatures, currPoints, mapPointsIdx, featureIdx, localKeyFrameIds, intrinsics, ...
        uavData.camToIMUTransform, imuParams, isIMUInitialized);
    
    % Remove outlier map points that are observed in fewer than 3 key frames
    [mapPointSet,pointToNode] = helperCullRecentMapPoints(mapPointSet, pointToNode, mapPointsIdx, newPointIdx);
    
    % Create new map points by triangulation
    minNumMatches = 10;
    minParallax   = 3;
    [fGraph, pointToNode, mapPointSet, vSetKeyFrames, newPointIdx] = helperCreateNewMapPointsVI(...
        fGraph, pointToNode, viewToNode, mapPointSet, vSetKeyFrames, ...
        currKeyFrameId, intrinsics, scaleFactor, minNumMatches, minParallax);

    loopClosureCheck = 350; % Before this frame, loop closure does not occur. 
    % Check loop closure after some key frames have been created  
    if currKeyFrameId > loopClosureCheck && loopCtr>10

        % Minimum number of feature matches of loop edges
        loopEdgeNumMatches = 100;

        % Detect possible loop closure key frame candidates
        potentialLoopCandidates = helperFindPotentialLC(vSetKeyFrames);

        if ~isempty(potentialLoopCandidates)
            [isDetected, validLoopCandidates] = helperCheckLoopClosureVI(encodedImages, potentialLoopCandidates);
            if isDetected 
                % Add loop closure connections
                [fGraph, isLoopClosed, mapPointSet, vSetKeyFrames] = helperAddLoopConnectionsVI(fGraph, viewToNode, pointToNode,...
                    mapPointSet, vSetKeyFrames, validLoopCandidates, currKeyFrameId, ...
                    currFeatures, currPoints, loopEdgeNumMatches, intrinsics);
                if isLoopClosed
                    loopCtr=0;
                end
            end
        end

    end

    % Local bundle adjustment
    [refinedViews, dist] = connectedViews(vSetKeyFrames, currKeyFrameId, MaxDistance=2);
    refinedKeyFrameIds = refinedViews.ViewId;
    fixedViewIds = refinedKeyFrameIds(dist==2);
    fixedViewIds = fixedViewIds(1:min(10, numel(fixedViewIds)));

    refinedKeyFrameIds = [refinedKeyFrameIds; currKeyFrameId];

    [fGraph, solInfo] = helperLocalFactorGraphOptimization(fGraph, viewToNode, refinedKeyFrameIds, fixedViewIds);
    % 打印solInfo信息
    disp([num2str(length(finalCostList) + 1), ': solInfo.FinalCost: ', num2str(solInfo.FinalCost)]);
    finalCostList = [finalCostList; solInfo.FinalCost];

    allOptNodes=solInfo.OptimizedNodeIDs;
    optPointNodes=setdiff(allOptNodes,viewToNode(refinedKeyFrameIds));
    [~,mapPointIdx]=intersect(pointToNode,optPointNodes,'stable');

    % Update world points
    mapPointSet = updateWorldPoints(mapPointSet,mapPointIdx,nodeState(fGraph,pointToNode(mapPointIdx)));

    % Update view poses
    for i=1:length(refinedKeyFrameIds)
        viewId = refinedKeyFrameIds(i);
        fgPose = nodeState(fGraph,viewToNode(viewId));
        absPose = rigidtform3d(quat2rotm(fgPose(4:7)),fgPose(1:3));
        vSetKeyFrames = updateView(vSetKeyFrames,viewId,absPose);
    end

    % Update view direction and depth
    mapPointSet = updateLimitsAndDirection(mapPointSet, mapPointIdx, vSetKeyFrames.Views);
    
    % Update representative view
    mapPointSet = updateRepresentativeView(mapPointSet, mapPointIdx, vSetKeyFrames.Views);

    % Visualize 3D world points and camera trajectory
    updatePlot(mapPlot, vSetKeyFrames, mapPointSet);
    
    % Update IDs and indices
    lastKeyFrameId  = currKeyFrameId;
    lastKeyFrameIdx = currFrameIdx;
    addedFramesIdx  = [addedFramesIdx; currFrameIdx]; %#ok<AGROW>
    currFrameIdx    = currFrameIdx + 1;

end % End of main loop


% Optimize the poses
vSetKeyFramesOptim = vSetKeyFrames;
fGraphOptim = fGraph;

optimize(fGraphOptim,fgso);

% Update map points after optimizing the poses
for i=1:vSetKeyFrames.NumViews
    fgPose = nodeState(fGraphOptim,viewToNode(i));
    absPose = rigidtform3d(quat2rotm(fgPose(4:7)),fgPose(1:3));
    vSetKeyFramesOptim = updateView(vSetKeyFramesOptim,i,absPose);
end

mapPointSet = helperUpdateGlobalMap(mapPointSet,vSetKeyFrames,vSetKeyFramesOptim);

updatePlot(mapPlot, vSetKeyFrames, mapPointSet);

% Plot the optimized camera trajectory
plotOptimizedTrajectory(mapPlot, poses(vSetKeyFramesOptim))

% Update legend
showPlotLegend(mapPlot);

% Load ground truth 
gTruth     = uavData.gTruth;

% Plot the actual camera trajectory 
plotActualTrajectory(mapPlot, uavData, keyFrameToFrame, gDir);

% Show legend
showPlotLegend(mapPlot);



[rmse, ame] = helperGetAccuracy(poses(vSetKeyFramesOptim),uavData,keyTimeStamps,gDir);

% 绘制finalCostList的曲线图
figure;
plot(finalCostList);

% 保存finalCostList变量到.mat文件
save('finalCostList_attack.mat','finalCostList');






function [fGraph, refinedAbsPoses, refinedPoints, viewToNode, pointToNode]  = helperInitFactorGraph(vSetKeyFrames, mapPointSet, intrinsics)

    K = intrinsics.K;
    camInfo = ((K(1,1)/1.5)^2)*eye(2);

    refinedKeyFrameIds = vSetKeyFrames.Views.ViewId';
    
    graphIds = [];
    graphMes = [];
    graphPts = [];
    
    fGraph = factorGraph;
    
    % Generate the necessary IDs for poses and 3D points
    zeroId      = generateNodeID(fGraph,1);
    viewToNode  = generateNodeID(fGraph,vSetKeyFrames.NumViews)';
    pointToNode = generateNodeID(fGraph,mapPointSet.Count)';
    
    mapPointIdx = [];
    
    % Extract the data from mapPointSet to initialize the factor graph
    for i=1:length(refinedKeyFrameIds)
      viewId=refinedKeyFrameIds(i);
      [pointIndices,featureIndices]=findWorldPointsInView(mapPointSet,viewId);
    
      pointIds = pointToNode(pointIndices);
      poseIds  = ones(length(pointIds),1)*viewToNode(viewId);
      graphIds = [graphIds; [poseIds pointIds]];
    
      ptsInView   = vSetKeyFrames.Views.Points{viewId,1}.Location;
      graphMes = [graphMes; ptsInView(featureIndices,:)];
    
      mapPointIdx = [mapPointIdx; pointIndices];
      graphPts = [graphPts; mapPointSet.WorldPoints(pointIndices,:)];
    end
    
    [mapPointIdx, uniqueIndex, ~] = unique(mapPointIdx);
    graphPts = graphPts(uniqueIndex,:);
    
    % Set up a pose prior using the inital position and fix it
    initPos = double([vSetKeyFrames.Views.AbsolutePose(1,1).Translation rotm2quat(vSetKeyFrames.Views.AbsolutePose(1,1).R)]);
    initPosePrior = factorPoseSE3Prior(viewToNode(1),Measurement=initPos);
    addFactor(fGraph,initPosePrior);
    fixNode(fGraph,viewToNode(1));
    
    % Set up a factorCameraSE3AndPointXYZ using the matches extracted above
    fCam=factorCameraSE3AndPointXYZ(graphIds, K, Measurement=graphMes, ...
    Information=camInfo);
    addFactor(fGraph,fCam);
    nodeState(fGraph,pointToNode(mapPointIdx),double(graphPts));
    
    % Set up a pose node using the current position
    pose=vSetKeyFrames.Views.AbsolutePose(2,1);
    pose=double([pose.Translation rotm2quat(pose.R)]);
    nodeState(fGraph,viewToNode(2),pose);
    
    % Fix a few 3D points for more stability
    ptsIDS = nodeIDs(fGraph,NodeType="POINT_XYZ");
    idx = randsample(height(ptsIDS'),int32(height(ptsIDS')*0.05));
    fixNode(fGraph,ptsIDS(idx));
    
    optimize(fGraph,factorGraphSolverOptions);
    
    fixNode(fGraph,ptsIDS(idx),false);
    
    % Extract the results of the factor graph optimization and re-package them
    poseIDs  = nodeIDs(fGraph,NodeType="POSE_SE3");
    fgposopt = nodeState(fGraph,poseIDs);
    
    initPose    = rigidtform3d(quat2rotm(fgposopt(1,4:7)),fgposopt(1,1:3));
    refinedPose = rigidtform3d(quat2rotm(fgposopt(2,4:7)),fgposopt(2,1:3));
    
    ViewId=[1;2];
    AbsolutePose=[initPose;refinedPose];
    
    refinedAbsPoses=table(ViewId,AbsolutePose);
    
    ptsIDS = nodeIDs(fGraph,NodeType="POINT_XYZ");
    refinedPoints = nodeState(fGraph,ptsIDS);
end


function [fGraph, viewToNode, velToNode, biasToNode, mapPoints, vSetKeyFrames] = helperAddNewKeyFrame(...
    fGraph, viewToNode, pointToNode, velToNode, biasToNode, mapPoints, vSetKeyFrames, imuMeasurements, ...
    cameraPose, currFeatures, currPoints, mapPointsIndices, featureIndices, keyFramesIndices, ...
    intrinsics, camToIMUTransform, imuParams, initIMU)

    N = length(mapPointsIndices);
    K = intrinsics.K;
    camInfo = ((K(1,1)/1.5)^2)*eye(2);
    
    viewId = vSetKeyFrames.Views.ViewId(end)+1;
    
    vSetKeyFrames = addView(vSetKeyFrames, viewId, cameraPose,...
        'Features', currFeatures.Features, ...
        'Points', currPoints);
    
    viewsAbsPoses = vSetKeyFrames.Views.AbsolutePose;
    
    % Generate the pose ID
    poseId = generateNodeID(fGraph,1);
    viewToNode = [viewToNode;poseId];
    
    for i = 1:numel(keyFramesIndices)
        localKeyFrameId = keyFramesIndices(i);
        [index3d, index2d] = findWorldPointsInView(mapPoints, localKeyFrameId);
        [~, index1, index2] = intersect(index3d, mapPointsIndices, 'stable');
        
        prePose   = viewsAbsPoses(localKeyFrameId);
        relPose = rigidtform3d(prePose.R' * cameraPose.R, ...
            (cameraPose.Translation-prePose.Translation)*prePose.R);
        
        if numel(index1) > 5
            vSetKeyFrames = addConnection(vSetKeyFrames, localKeyFrameId, viewId, relPose, ...
                Matches=[index2d(index1),featureIndices(index2)]);
    
            % Add a 3d pose factor to link both frames in the local map
            fgRelPose=double([relPose.Translation rotm2quat(relPose.R)]);
            fPose=factorTwoPoseSE3([viewToNode(localKeyFrameId) viewToNode(viewId)],Measurement=fgRelPose);
            addFactor(fGraph,fPose);
        end
    end
    
    mapPoints = addCorrespondences(mapPoints, viewId, mapPointsIndices, ...
        featureIndices);
    
    % Generate the IDs to be used in the factorCameraSE3AndPointXYZ
    ptsIds  = pointToNode(mapPointsIndices);
    poseIds = ones(N,1)*poseId;
    
    ptsInView = vSetKeyFrames.Views.Points{viewId,1}.Location;
    mesPts = ptsInView(featureIndices,:);
    
    % Add camera factor to the factor graph
    fCam = factorCameraSE3AndPointXYZ([poseIds ptsIds], K, Measurement=mesPts, ...
        Information=camInfo);
    addFactor(fGraph,fCam);
    
    % Add pose node to the factor graph
    pose = double([cameraPose.Translation rotm2quat(cameraPose.R)]);
    nodeState(fGraph,poseId,pose);
    
    % If the camera poses have been scaled properly, then add the IMU
    % factor to the factor graph
    if initIMU
        velId = generateNodeID(fGraph,1);
        velToNode=[velToNode;velId];
    
        biasId = generateNodeID(fGraph,1);
        biasToNode=[biasToNode;biasId];
    
        imuId = [viewToNode(end-1),velToNode(end-1),biasToNode(end-1), ...
                 viewToNode(end)  ,velToNode(end)  ,biasToNode(end)];
        fIMU = factorIMU(imuId,imuMeasurements.gyro,imuMeasurements.accel,imuParams,SensorTransform=camToIMUTransform);
        addFactor(fGraph,fIMU);
    end
end