function cameraRtC2W=RGBDsfm(sequenceName, frameIDs)

if ~exist('sequenceName','var')
    % load demo sequence
    %sequenceName = 'hotel_mr/scan1';
    sequenceName = 'hotel_umd/maryland_hotel3';
    %sequenceName = 'brown_bm_1/brown_bm_1';
    %sequenceName = 'home_md/home_md_scan9_2012_sep_30';
end
if ~exist('frameIDs','var')
    frameIDs = [];
end

%write2path = '/data/vision/torralba/sun3d/sfmNew/';
%write2path = '/Users/xj/local_data/';
SUN3Dpath = 'http://sun3d.csail.mit.edu/data/';

write2path = '/n/fs/sun3d/data/sfm/';
%SUN3Dpath = '/n/fs/sun3d/data/scene/scene_final/';

basicSetup

%% read data

data = loadSUN3D(sequenceName, frameIDs, SUN3Dpath);

% show the video
%{
figure
for cameraID=1:length(data.image)
    imshow(imread(data.image{cameraID}));
    title(sprintf('Frame %d',cameraID));
    drawnow;
end
%}

%% loop closure


%% loop closure detection

%length(data.image) = 10; % debug

BOWmodel = BOWtrain(data);
scores = BOWmodel.index.histograms' * BOWmodel.index.histograms;
% release memory
clear BOWmodel;

% smooth out
% worry about recall? ==> use max between several columns

% remove your self?

wDistance = tril(min(1,bwdist(eye(length(data.image)))/30),-1);
scores = double(wDistance) .* full(scores);
G = fspecial('gaussian',[5 5],2);
scores = imfilter(scores,G,'same');
clear G;

% non max suppression
scoresNMS = nonmaxsup(scores, 7);

ind = find(scoresNMS(:)>0.1); % threshold 

[~, perm]= sort(scoresNMS(ind),'descend');
ind = ind(perm);

% for time problem, make it no longer than the length(data.image)
if length(ind)>length(data.image)
    ind = ind(1:length(data.image));
end

[cameras_i, cameras_j] = ind2sub([length(data.image) length(data.image)],ind);

clear scores
clear scoresNMS

% visualization
%{
figure
for cameraID=1:length(cameras_i)
    im_i = imread(data.image{cameras_i(cameraID)});
    im_j = imread(data.image{cameras_j(cameraID)});
    
    im_ij(:,:,1)=[im_i(:,:,1) im_j(:,:,1)];
    im_ij(:,:,2)=[im_i(:,:,2) im_j(:,:,2)];
    im_ij(:,:,3)=[im_i(:,:,3) im_j(:,:,3)];
    
    clf
    subplot(3,1,1);
    imshow(im_ij);
    title(sprintf('Frame %d and %d: score = %f', cameras_i(cameraID), cameras_j(cameraID), full(scores(cameras_i(cameraID),cameras_j(cameraID)))));
    
    subplot(3,1,[2 3]);
    imagesc(scores);
    axis equal;
    hold on;
    siW = 50;
    plot([cameras_j(cameraID)-siW cameras_j(cameraID)+siW cameras_j(cameraID)+siW cameras_j(cameraID)-siW cameras_j(cameraID)-siW], ...    
    [cameras_i(cameraID)-siW cameras_i(cameraID)-siW cameras_i(cameraID)+siW cameras_i(cameraID)+siW cameras_i(cameraID)-siW], '-r');
    axis tight;
    axis off;
    drawnow;
    
    pause(0.2);
end
%}

% show the video
%{
figure
for cameraID=1:length(data.image)
    imshow(readImage(frames,cameraID));
    title(sprintf('Frame %d',cameraID));
    drawnow;
end
%}

%% run loop matching
MatchPairsLoop = cell(1,length(cameras_i));
parfor cameraID = 1:length(cameras_i)
    MatchPairsLoop{cameraID} = align2view(data, cameras_i(cameraID), cameras_j(cameraID));
end
minAcceptableSIFT = 25;

cntLoopEdge = 0;
for pairID=1:length(MatchPairsLoop)
    if size(MatchPairsLoop{pairID}.matches,2)>minAcceptableSIFT
        cntLoopEdge = cntLoopEdge+1;
    end
end
fprintf('found %d good loop edges\n',cntLoopEdge); clear cntLoopEdge;

%{
figure
for pairID=1:length(MatchPairsLoop)
    if size(MatchPairsLoop{pairID}.matches,2)>minAcceptableSIFT
        im_i = readImage(frames,MatchPairsLoop{pairID}.i);
        im_j = readImage(frames,MatchPairsLoop{pairID}.j);

        im_ij(:,:,1)=[im_i(:,:,1) im_j(:,:,1)];
        im_ij(:,:,2)=[im_i(:,:,2) im_j(:,:,2)];
        im_ij(:,:,3)=[im_i(:,:,3) im_j(:,:,3)];

        clf
        subplot(3,1,1);
        imshow(im_ij);
        title(sprintf('Frame %d and %d: matches = %d', MatchPairsLoop{pairID}.i, MatchPairsLoop{pairID}.j, size(MatchPairsLoop{pairID}.matches,2)));
        hold on;

        plot([MatchPairsLoop{pairID}.matches(1,:)' 640+MatchPairsLoop{pairID}.matches(6,:)']',[MatchPairsLoop{pairID}.matches(2,:)' MatchPairsLoop{pairID}.matches(7,:)']','-');

        subplot(3,1,[2 3]);
        imagesc(scores);
        axis equal;
        hold on;
        siW = 50;
        plot([MatchPairsLoop{pairID}.j-siW MatchPairsLoop{pairID}.j+siW MatchPairsLoop{pairID}.j+siW MatchPairsLoop{pairID}.j-siW MatchPairsLoop{pairID}.j-siW], ...    
        [MatchPairsLoop{pairID}.i-siW MatchPairsLoop{pairID}.i-siW MatchPairsLoop{pairID}.i+siW MatchPairsLoop{pairID}.i+siW MatchPairsLoop{pairID}.i-siW], '-r');
        axis tight;
        axis off;
        drawnow;

        pause(1);
    end
end
%}

%{
figure(100)
for cameraID=1:length(cameras_i)
    if size(MatchPairsLoop{cameraID}.matches,2)>minAcceptableSIFT
        hold on
        plot3(cameraCenters(1,[cameras_i(cameraID) cameras_j(cameraID)]),cameraCenters(2,[cameras_i(cameraID) cameras_j(cameraID)]),cameraCenters(3,[cameras_i(cameraID) cameras_j(cameraID)]),'-k');
    end
end
%}

clear cameras_i
clear cameras_j

%% time based reconstruction
MatchPairs = cell(1,length(data.image)-1);
parfor frameID = 1:length(data.image)-1
    MatchPairs{frameID} = align2view(data, frameID, frameID+1);
end

% naive approach: just put all results together
cameraRtC2W = repmat([eye(3) zeros(3,1)], [1,1,length(data.image)]);
for frameID = 1:length(data.image)-1
    cameraRtC2W(:,:,frameID+1) = [cameraRtC2W(:,1:3,frameID) * MatchPairs{frameID}.Rt(:,1:3) cameraRtC2W(:,1:3,frameID) * MatchPairs{frameID}.Rt(:,4) + cameraRtC2W(:,4,frameID)];
end

if ~exist(fullfile(write2path, sequenceName),'dir')
    mkdir(fullfile(write2path, sequenceName));
end
save(fullfile(write2path, sequenceName, 'cameraRt_RANSAC.mat'),'cameraRtC2W','MatchPairs','-v7.3');

fprintf('ransac all finished\n');


outputPly(fullfile(write2path, sequenceName, 'time.ply'), cameraRtC2W, data);



%{
% plot the pose graph
cameraCenters = reshape(cameraRtC2W(:,4,:),3,[]);
figure(100)
plot3(cameraCenters(1,:),cameraCenters(2,:),cameraCenters(3,:),'-');
axis equal;
hold on;
grid on;
plot3(cameraCenters(1,:),cameraCenters(2,:),cameraCenters(3,:),'.r', 'markersize', 0.1);
%}



%save('all_debug.mat','-v7.3');


%% bundle adjustment

wTimePoints = 0.1;
w3D = 100;

% link track
maxNumPoints = length(data.image)*1000;
pointObserved= sparse(length(data.image),maxNumPoints);
pointObservedValue = zeros(6,maxNumPoints);

pointCloud   = zeros(3,maxNumPoints);
pointCount = 0;
pointObservedValueCount = 0;

%% time based

% reduce the numbers for time matching (hack) ==> should be better
%for pairID = 1:length(MatchPairs)
%    MatchPairs{pairID}.matches = MatchPairs{pairID}.matches(:,1:5);
%end

doLongTrack = true;

if doLongTrack

    pointCount = size(MatchPairs{1}.matches,2);
    pointObservedValueCount = size(MatchPairs{1}.matches,2)*2;
    pointObservedValue(:,1:pointObservedValueCount) = [[MatchPairs{1}.matches(1:5,:) MatchPairs{1}.matches(6:10,:)]; -wTimePoints * ones(1,pointObservedValueCount)];
    pointObserved(1,1:pointCount)=1:pointCount;
    pointObserved(2,1:pointCount)=pointCount + (1:pointCount);
    previousIndex = 1:pointCount;
    
    pointCloud(:,1:pointCount) = MatchPairs{1}.matches(3:5,:);
    
    for frameID = 2:length(data.image)-1
        [~,iA,iB] = intersect(MatchPairs{frameID-1}.matches(6:7,:)',MatchPairs{frameID}.matches(1:2,:)','rows');
        
        
        alreadyExist = false(1,size(MatchPairs{frameID}.matches,2));
        alreadyExist(iB) = true;
        newCount = sum(~alreadyExist);
        
        
        currentIndex = zeros(1,size(MatchPairs{frameID}.matches,2));
        currentIndex(iB) = previousIndex(iA);
        currentIndex(~alreadyExist) = (pointCount+1):(pointCount+newCount);
        
        pointObservedValue(1:5,pointObservedValueCount+1:pointObservedValueCount+newCount+length(currentIndex)) = [MatchPairs{frameID}.matches(1:5,~alreadyExist) MatchPairs{frameID}.matches(6:10,:)];
        pointObservedValue(6,pointObservedValueCount+1:pointObservedValueCount+newCount+length(currentIndex)) = -wTimePoints;
        
        pointObserved(frameID  ,currentIndex(~alreadyExist)) = (pointObservedValueCount+1):(pointObservedValueCount+newCount);
        pointObservedValueCount = pointObservedValueCount + newCount;
        pointObserved(frameID+1,currentIndex) = (pointObservedValueCount+1):(pointObservedValueCount+length(currentIndex));
        pointObservedValueCount = pointObservedValueCount + length(currentIndex);
        
        
        pointCloud(:,pointCount+1:pointCount+newCount) = transformRT(MatchPairs{frameID}.matches(3:5,~alreadyExist), cameraRtC2W(:,:,frameID), false);
        
        pointCount = pointCount + newCount;
        
        previousIndex = currentIndex;
    end
    
else
    for pairID = 1:length(MatchPairs)
        n = size(MatchPairs{pairID}.matches,2);

        pointObservedValue(:,pointObservedValueCount+1:pointObservedValueCount+n*2) = [[MatchPairs{pairID}.matches(1:5,:) MatchPairs{pairID}.matches(6:10,:)]; -wTimePoints * ones(1,2*size(MatchPairs{pairID}.matches,2))];

        pointObserved(MatchPairs{pairID}.i,pointCount+1:pointCount+n)=pointObservedValueCount+1  :pointObservedValueCount+n;
        pointObserved(MatchPairs{pairID}.j,pointCount+1:pointCount+n)=pointObservedValueCount+1+n:pointObservedValueCount+n*2;

        pointCloud(:,pointCount+1:pointCount+n) = transformRT(MatchPairs{pairID}.matches(3:5,:), cameraRtC2W(:,:,MatchPairs{pairID}.i), false);

        pointObservedValueCount = pointObservedValueCount+n*2;
        pointCount = pointCount+n;

    end
end


%% loop closure
for pairID = 1:length(MatchPairsLoop)
    if size(MatchPairsLoop{pairID}.matches,2)>minAcceptableSIFT
        n = size(MatchPairsLoop{pairID}.matches,2);

        pointObservedValue(:,pointObservedValueCount+1:pointObservedValueCount+n*2) = [[MatchPairsLoop{pairID}.matches(1:5,:) MatchPairsLoop{pairID}.matches(6:10,:)]; -1 * ones(1,2*size(MatchPairsLoop{pairID}.matches,2))];

        pointObserved(MatchPairsLoop{pairID}.i,pointCount+1:pointCount+n)=pointObservedValueCount+1  :pointObservedValueCount+n;
        pointObserved(MatchPairsLoop{pairID}.j,pointCount+1:pointCount+n)=pointObservedValueCount+1+n:pointObservedValueCount+n*2;

        pointCloud(:,pointCount+1:pointCount+n) = transformRT(MatchPairsLoop{pairID}.matches(3:5,:), cameraRtC2W(:,:,MatchPairsLoop{pairID}.i), false);

        pointObservedValueCount = pointObservedValueCount+n*2;
        pointCount = pointCount+n;
    end
end


save(fullfile(write2path, sequenceName,'MatchPairs.mat'),'MatchPairs','MatchPairsLoop','-v7.3');
clear MatchPairsLoop
clear MatchPairs

% save some memory
pointCloud = pointCloud(:,1:pointCount);
pointObserved = pointObserved(:,1:pointCount);
pointObservedValue = pointObservedValue(:,1:pointObservedValueCount);

%% bundle adjustment
outputKeypointsPly(fullfile(write2path, sequenceName, 'time_key.ply'),pointCloud(:,reshape(find(sum(pointObserved~=0,1)>0),1,[])));

fprintf('bundle adjusting    ...\n');
tic
%[cameraRtC2W,pointCloud] = bundleAdjustment2D3DRobustFile(cameraRtC2W,pointCloud,pointObserved, pointObservedValue, frames.K, w3D, 3);
global objectLabel;
objectLabel.length = 0;
objectLabel.objectRtO2W = zeros(3,4,objectLabel.length);
objectLabel.objectSize = zeros(objectLabel.length,3);
objectLabel.optimizationWeight = zeros(1,objectLabel.length);
[cameraRtC2W,pointCloud] = bundleAdjustment2D3DBoxFile(cameraRtC2W,pointCloud,pointObserved, pointObservedValue, data.K, w3D, 3);
toc;

outputKeypointsPly(fullfile(write2path, sequenceName, 'BA_key.ply'),pointCloud(:,reshape(find(sum(pointObserved~=0,1)>0),1,[])));

outputPly(fullfile(write2path, sequenceName, 'BA.ply'), cameraRtC2W, data);

fprintf('rectifying scenes ...');
tic
cameraRtC2W = rectifyScene(cameraRtC2W, data);
toc

%% save all necessary works

% clean all unnecessary variables
clear frameID
clear frame_begin
clear frame_end
clear frame_interval
clear ind
clear jsonfiles
clear maxNumPoints
clear minAcceptableSIFT
clear n
clear pairID
clear perm

save(fullfile(write2path, sequenceName,'BA_variables.mat'),'-v7.3');


timeStamp = getTimeStamp();

%% output camera text file
outputCameraExtrinsics(SUN3Dpath, sequenceName, cameraRtC2W, timeStamp);

%% output thumbnail
outputThumbnail(SUN3Dpath, sequenceName, cameraRtC2W, timeStamp, data);

end

