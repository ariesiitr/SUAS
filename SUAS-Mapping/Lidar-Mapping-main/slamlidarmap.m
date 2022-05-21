
%taking and generating the dataset from trajectory, truth map and lidar
%scanned laz file
datafile = 'aerialMap.tar.gz';
wayPointsfile = 'gTruthWayPoints.mat';

% Generate a lidar scan at each waypoint using the helper function

[pClouds,orgMap,gTruthWayPts] = helperCreateDataset(datafile,wayPointsfile);

%Visualize the ground truth waypoints on the original map covered by the robot.
% Create a figure window to visualize the ground truth map and waypoints 
hFigGT = figure;
axGT = axes('Parent',hFigGT,'Color','black');

% Visualize the ground truth waypoints
%pcshow(gTruthWayPts,'red','MarkerSize',150,'Parent',axGT)
%hold on

% Visualize the original map covered by the robot
pcshow(orgMap,'MarkerSize',10,'Parent',axGT)
hold off

% Customize the axis labels
xlabel(axGT,'X (m)')
ylabel(axGT,'Y (m)')
zlabel(axGT,'Z (m)')
title(axGT,'Ground Truth Map And Robot Trajectory')

%Visualize the extracted lidar scans using the pcplayer object.
% Specify limits for the player
xlimits = [-90 90];
ylimits = [-90 90];
zlimits = [-625 -587];

% Create a pcplayer object to visualize the lidar scans
lidarPlayer = pcplayer(xlimits,ylimits,zlimits);

% Customize the pcplayer axis labels
xlabel(lidarPlayer.Axes,'X (m)')
ylabel(lidarPlayer.Axes,'Y (m)')
zlabel(lidarPlayer.Axes,'Z (m)')
title(lidarPlayer.Axes,'Lidar Scans')

% Loop over and visualize the data
for l = 1:length(pClouds)
    
    % Extract the lidar scan
    ptCloud = pClouds(l);
    
    % Update the lidar display
    view(lidarPlayer,ptCloud)
    pause(0.05)
end

%Set Up Tunable Parameters

skipFrames = 3;
%Downsampling lidar scans can improve algorithm speed

gridStep = 1.5; % in meters
neighbors = 60;

%Set the threshold and ratio for matching the FPFH descriptors, used to identify point correspondences.

matchThreshold = 0.1;
matchRatio = 0.97;

%Set the maximum distance and number of trails for relative pose estimation between successive scans.

maxDistance = 1;
maxNumTrails = 3000;

%Specify the percentage of inliers to consider for fine-tuning relative poses.

inlierRatio = 0.1;

%Specify the size of each cell of a box grid filter, used to create maps by aligning lidar scans.

alignGridStep = 1.2;

%Parameters for Loop Closure Estimation
%Specify the radius around the current robot location to search for loop closure candidates.
loopClosureSearchRadius = 7.9;
nScansPerSubmap = 3;
subMapThresh = 15;
loopClosureThreshold = 0.6;

%Specify the maximum acceptable root mean squared error (RMSE) for the estimation of relative pose 

rmseThreshold = 0.6;

%Create a pose graph, using a poseGraph3D object, to store estimated relative poses between accepted lidar scans.

pGraph = poseGraph3D;

% Default serialized upper-right triangle of a 6-by-6 Information Matrix
infoMat = [1 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 1];

%Preallocate and initialize the variables required for computation.
% Allocate memory to store submaps
subMaps = cell(floor(length(pClouds)/(skipFrames*nScansPerSubmap)),1);

% Allocate memory to store each submap pose
subMapPoses = zeros(round(length(pClouds)/(skipFrames*nScansPerSubmap)),3);

% Initialize variables to store accepted scans and their transforms for
% submap creation
pcAccepted = pointCloud.empty(0);
tformAccepted = rigid3d.empty(0);

% Initialize variable to store relative poses from the feature-based approach
% without pose graph optimization
fpfhTform = rigid3d.empty(0);

% Counter to track the number of scans added
count = 1;

%Create variables to visualize processed lidar scans and estimated trajectory.
% Set to 1 to visualize processed lidar scans during build process
viewPC = 0;

% Create a pcplayer object to view the lidar scans while
% processing them sequentially, if viewPC is enabled
if viewPC == 1
    pplayer = pcplayer(xlimits,ylimits,zlimits);
    
    % Customize player axis labels
    xlabel(pplayer.Axes,'X (m)')
    ylabel(pplayer.Axes,'Y (m)')
    zlabel(pplayer.Axes,'Z (m)')
    title(pplayer.Axes,'Processed Scans')
end

% Create a figure window to visualize the estimated trajectory
hFigTrajUpdate = figure;
axTrajUpdate = axes('Parent',hFigTrajUpdate,'Color','black');
title(axTrajUpdate,'Sensor Pose Trajectory')


%Trajectory Estimation and Refinement
for FrameIdx = 1:skipFrames:length(pClouds)
%Point Cloud Downsampling
% Downsample the current scan
    curScan = pcdownsample(pClouds(FrameIdx),'gridAverage',gridStep);
    if viewPC == 1
        
        % Visualize down sampled point cloud
        view(pplayer,curScan)
    end
    
    %Fine-tunes the relative pose using the pcregistericp function
    % Extract FPFH features
    curFeature = extractFPFHFeatures(curScan,'NumNeighbors',neighbors);
    
    if FrameIdx == 1
        
        % Update the acceptance scan and its tform
        pcAccepted(count,1) = curScan;
        tformAccepted(count,1) = rigid3d;
        
        % Update the initial pose to the first waypoint of ground truth for
        % comparison
        fpfhTform(count,1) = rigid3d(eye(3),gTruthWayPts(1,:));
    else
        
        % Identify correspondences by matching current scan to previous scan 
        indexPairs = pcmatchfeatures(curFeature,prevFeature,curScan,prevScan, ...
            'MatchThreshold',matchThreshold,'RejectRatio',matchRatio);
        matchedPrevPts = select(prevScan,indexPairs(:,2));
        matchedCurPts = select(curScan,indexPairs(:,1));
        
        % Estimate relative pose between current scan and previous scan 
        % using correspondences
        tform1 = estimateGeometricTransform3D(matchedCurPts.Location, ...
            matchedPrevPts.Location,'rigid','MaxDistance',maxDistance, ...
            'MaxNumTrials',maxNumTrails);

        % Perform ICP registration to fine-tune relative pose
        tform = pcregistericp(curScan,prevScan,'InitialTransform',tform1, ...
            'InlierRatio',inlierRatio);
%Convert the rigid transformation to an xyz-position and a quaternion orientation to add it to the pose graph. 
        relPose = [tform2trvec(tform.T') tform2quat(tform.T')];
        
        % Add relative pose to pose graph
        addRelativePose(pGraph,relPose);
%Store the accepted scans and their poses for submap creation.
        % Update counter and store accepted scans and their poses
        count = count + 1;
        pcAccepted(count,1) = curScan;
        accumPose = pGraph.nodes(height(pGraph.nodes));
        tformAccepted(count,1) = rigid3d((trvec2tform(accumPose(1:3)) * ...
            quat2tform(accumPose(4:7)))');
        
        % Update estimated poses
        fpfhTform(count) = rigid3d(tform.T * fpfhTform(count-1).T);
    end
%Submap Creation
currSubMapId = floor(count/nScansPerSubmap);
    if rem(count,nScansPerSubmap) == 0
        
        % Align an array of lidar scans to create a submap
        subMaps{currSubMapId} = pcalign(...
            pcAccepted((count - nScansPerSubmap + 1):count,1), ...
            tformAccepted((count - nScansPerSubmap + 1):count,1), ...
            alignGridStep);
        
        % Assign center scan pose as pose of submap
        subMapPoses(currSubMapId,:) = tformAccepted(count - ...
            floor(nScansPerSubmap/2),1).Translation;
    end 

    if currSubMapId > subMapThresh
        mostRecentScanCenter = pGraph.nodes(height(pGraph.nodes));
        
        % Estimate possible loop closure candidates by matching current
        % scan with submaps
        [loopSubmapIds,~] = helperEstimateLoopCandidates(subMaps,curScan, ...
            subMapPoses,mostRecentScanCenter,currSubMapId,subMapThresh, ...
            loopClosureSearchRadius,loopClosureThreshold);
        
        if ~isempty(loopSubmapIds)
            rmseMin = inf;
            
            % Estimate the best match for the current scan from the matching submap ids
            for k = 1:length(loopSubmapIds)
                
                % Check every scan within the submap
                for kf = 1:nScansPerSubmap
                    probableLoopCandidate = ...
                        loopSubmapIds(k)*nScansPerSubmap - kf + 1;
                    [pose_Tform,~,rmse] = pcregistericp(curScan, ...
                        pcAccepted(probableLoopCandidate));
                    
                    % Update the best loop closure candidate
                    if rmse < rmseMin
                        rmseMin = rmse;
                        matchingNode = probableLoopCandidate;
                        Pose = [tform2trvec(pose_Tform.T') ...
                            tform2quat(pose_Tform.T')];
                    end
                end
            end
            
            % Check if loop closure candidate is valid
            if rmseMin < rmseThreshold
                
                % Add relative pose of loop closure candidate to pose graph
                addRelativePose(pGraph,Pose,infoMat,matchingNode, ...
                    pGraph.NumNodes);
            end
        end
    end
    
    % Update previous point cloud and feature
    prevScan = curScan;
    prevFeature = curFeature;

    % Visualize the estimated trajectory from the accepted scan.
    show(pGraph,'IDs','off','Parent',axTrajUpdate);
    drawnow
end
%Pose Graph Optimization

pGraph = optimizePoseGraph(pGraph,'FirstNodePose',[gTruthWayPts(1,:) 1 0 0 0]);

%Get estimated trajectory from pose graph
pGraphTraj = pGraph.nodes;

% Get estimated trajectory from feature-based registration without pose
% graph optimization
fpfhEstimatedTraj = zeros(count,3);
for i = 1:count
    fpfhEstimatedTraj(i,:) = fpfhTform(i).Translation;
end

% Create a figure window to visualize the ground truth and estimated
% trajectories
hFigTraj = figure;
axTraj = axes('Parent',hFigTraj,'Color','black');
plot3(fpfhEstimatedTraj(:,1),fpfhEstimatedTraj(:,2),fpfhEstimatedTraj(:,3), ...
    'r*','Parent',axTraj)
hold on
plot3(pGraphTraj(:,1),pGraphTraj(:,2),pGraphTraj(:,3),'b.','Parent',axTraj)
plot3(gTruthWayPts(:,1),gTruthWayPts(:,2),gTruthWayPts(:,3),'go','Parent',axTraj)
hold off
axis equal
view(axTraj,2)
xlabel(axTraj,'X (m)')
ylabel(axTraj,'Y (m)')
zlabel(axTraj,'Z (m)')
title(axTraj,'Trajectory Comparison')
legend(axTraj,'Estimated trajectory without pose graph optimization', ...
    'Estimated trajectory with pose graph optimization', ...
    'Ground Truth Trajectory','Location','bestoutside')

%%Build and Visualize Generated Map
%%Transform and merge lidar scans using estimated poses to create an accumulated point cloud map.
% Get the estimated trajectory from poses
estimatedTraj = pGraphTraj(:,1:3);

% Convert relative poses to rigid transformations
estimatedTforms = rigid3d.empty(0);
for idx=1:pGraph.NumNodes
    pose = pGraph.nodes(idx);
    rigidPose = rigid3d((trvec2tform(pose(1:3)) * quat2tform(pose(4:7)))');
    estimatedTforms(idx,1) = rigidPose;
end

% Create global map from processed point clouds and their relative poses
globalMap = pcalign(pcAccepted,estimatedTforms,alignGridStep);

% Create a figure window to visualize the estimated map and trajectory
hFigTrajMap = figure;
axTrajMap = axes('Parent',hFigTrajMap,'Color','black');
pcshow(estimatedTraj,'red','MarkerSize',150,'Parent',axTrajMap)
hold on
pcshow(globalMap,'MarkerSize',10,'Parent',axTrajMap)
hold off

% Customize axis labels
xlabel(axTrajMap,'X (m)')
ylabel(axTrajMap,'Y (m)')
zlabel(axTrajMap,'Z (m)')
title(axTrajMap,'Estimated Robot Trajectory And Generated Map')

%Visualize the ground truth map and the estimated map.
% Create a figure window to display both the ground truth map and estimated map
hFigMap = figure('Position',[0 0 700 400]);
axMap1 = subplot(1,2,1,'Color','black','Parent',hFigMap);
axMap1.Position = [0.08 0.2 0.4 0.55];
pcshow(orgMap,'Parent',axMap1)
xlabel(axMap1,'X (m)')
ylabel(axMap1,'Y (m)')
zlabel(axMap1,'Z (m)')
title(axMap1,'Ground Truth Map')

axMap2 = subplot(1,2,2,'Color','black','Parent',hFigMap);
axMap2.Position = [0.56 0.2 0.4 0.55];
pcshow(globalMap,'Parent',axMap2);
xlabel(axMap2,'X (m)');
ylabel(axMap2,'Y (m)');
zlabel(axMap2,'Z (m)');
title(axMap2,'Estimated Map');


