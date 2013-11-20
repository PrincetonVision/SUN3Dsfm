function ObjectAdjustment(sequenceName, frameIDs)
%function cameraRtC2W=RGBDsfmObjectAdjustment(sequenceName, frameIDs)

if ~exist('sequenceName','var')
    % load demo sequence
    %sequenceName = 'hotel_mr/scan1';
    sequenceName = 'hotel_umd/maryland_hotel3';
    %sequenceName = 'brown_bm_1/brown_bm_1';
end
if ~exist('frameIDs','var')
    frameIDs = [];
end



basicSetup

%write2path = '/data/vision/torralba/sun3d/sfmNew/';
%write2path = './local_data/';
write2path = '/n/fs/sun3d/data/sfm/';
%SUN3Dpath = '/n/fs/sun3d/data/scene/scene_final/';
SUN3Dpath = 'http://sun3d.csail.mit.edu/data/';

%% read data
load(fullfile(write2path, sequenceName,'BA_variables.mat'));
data = loadSUN3D(sequenceName, frameIDs, SUN3Dpath);




subsampleFactor = 0.1;
subsampleMinPoints = 10;
outlierMargin = 0.02;


resizingScale = 1;
adjustSize = false;
normalizeWeight = true;
SIFTkeypointWeightTime = 30;
SIFTkeypointWeightLoop = 30;
objectWeightTotal = 1; %1;


% hack to rotate? for debugging
%{
if ~1
    preRT = [eye(3) [0;0;0]];
    ang=pi; %*45/180;
    preRT = [...
        cos(ang) -sin(ang) 0   0; ...
        sin(ang)  cos(ang) 0 1.7; ...
        0         0 1   0];
    
    for frameID=1:size(cameraRtC2W,3)
        cameraRtC2W(:,:,frameID) = [preRT(:,1:3) * cameraRtC2W(:,1:3,frameID) preRT(:,1:3) * cameraRtC2W(:,4,frameID) + preRT(:,4)];
    end
else
%}
    fprintf('rectifying scenes ...');
    tic
    cameraRtC2W = rectifyScene(cameraRtC2W, data);
    toc
    
    %{
    % visualize
    for frameID = 1 : 10 : size(cameraRtC2W,3)
        XYZcam = depth2XYZcamera(data.K, depthRead(data.depth{frameID}));
        isValid = find(reshape(XYZcam(:,:,4)~=0,1,[]));
        XYZ{frameID} = transformRT([XYZcam(isValid); XYZcam(640*480+isValid); XYZcam(640*480*2+isValid)], cameraRtC2W(:,:,frameID), false);
    end
    pts = cat(2, XYZ{:});
    inds = randsample(1:size(pts,2), 10000);
    plot3(pts(1,inds), pts(2,inds), pts(3,inds), '.', 'markersize', 0.5);
    %}
%end

% for display
cameraPts = [0 0 0; -240 -320 data.K(1,1); -240 320 data.K(1,1); 240 320 data.K(1,1); 240 -320 data.K(1,1)]'; 
cameraPts = cameraPts * 0.0001;


%{
figure(100);
clf
plot3(reshape(cameraRtC2W(1,4,:),1,[]),reshape(cameraRtC2W(2,4,:),1,[]),reshape(cameraRtC2W(3,4,:),1,[]),'.')
axis equal;
grid on;
xlabel('x')
ylabel('y')
zlabel('z')
set(gca, 'YDir', 'reverse');
%}

%% load everything
[pixelX,pixelY] = meshgrid(1:640,1:480);
pointCountOld = pointCount;

% parameter one: weighting of SIFT keypoints
indTime = pointObservedValue(6,:)==-0.1;
indLoop = pointObservedValue(6,:)==-1;

% normalize the weight
SIFTkeypointWeightTime = sqrt(1/sum(indTime) * SIFTkeypointWeightTime) ;
SIFTkeypointWeightLoop = sqrt(1/sum(indLoop) * SIFTkeypointWeightLoop);

pointObservedValue(6,indTime) = - SIFTkeypointWeightTime;
pointObservedValue(6,indLoop) = - SIFTkeypointWeightLoop;

uniqueW = unique(pointObservedValue(6,:));


%% now, bundle adjustment based on the labeling result
objectSize = reshape(textread('ObjectSize.csv','%s','delimiter', ','),5,[])';
objectSizeName = objectSize(:,1);
objectSizeSize = arrayfun(@(x) str2num(x{1}), objectSize(:,2:end));
%objectSizeSize = sort(objectSizeSize,2,'descend');


global objectLabel;
objectLabel.length = length(data.annotation.objects);
objectLabel.objectRtO2W = zeros(3,4,objectLabel.length);
objectLabel.optimizationWeight = zeros(1,objectLabel.length);
objectLabel_first = zeros(1,objectLabel.length);

objectLabel.objectSize = zeros(objectLabel.length,3);
for label=1:objectLabel.length
    if ~isempty(data.annotation.objects{label})
        texts = regexp(data.annotation.objects{label}.name,':','split');
        category = texts{1};
        
        id = find(ismember(objectSizeName,category));
        if ~isempty(id)
            objectLabel.objectSize(label,:) = objectSizeSize(id,1:3);
            objectLabel.objectType(label)   = objectSizeSize(id,4);
        else
            objectLabel.objectSize(label,:) = [100,100,100]; % unknown object = no constrains;
            objectLabel.objectType(label)   = 2;
            fprintf('object with unknown size (%d): %s\n',label,category);
        end
    else
        objectLabel.objectSize(label,:) = [100,100,100]; % unknown object = no constrains;
        objectLabel.objectType(label)   = 2;
        
    end
end

%% 
% prepare for gallery
%{
keyframeCNT =0;
for cameraID = 1:frames.length
    if ~isempty(readLabel(frames, cameraID))
        keyframeCNT = keyframeCNT+1;
    end
end
fprintf('number of keyframes = %d\n', keyframeCNT);
%}

%objectLabel.objectSize = objectLabel.objectSize * 100; %max(1,objectLabel.objectSize);

fnameList = cell(1,length(data.image));
for i=1:length(data.image)
    [~, fname] = fileparts(data.image{i});
    fnameList{i} = fname;
end

KeyframeID = [];
for i=1:length(data.annotation.frames)
    if ~isempty(data.annotation.frames{i})
        frameID = find(ismember(fnameList,data.annotation.fileList{i}(1:end-4)));
        KeyframeID = [KeyframeID frameID];
    end
end



objectLabel.optimizationWeight = zeros(1,objectLabel.length);

%figure(1); clf
for i=1:length(data.annotation.frames)
    if ~isempty(data.annotation.frames{i})
        frameID = find(ismember(fnameList,data.annotation.fileList{i}(1:end-4)));
        
        %image  = imread(data.image{frameID});
        XYZcam = depth2XYZcamera(data.K, depthRead(data.depth{frameID}));
        
        for polygonID = 1:length(data.annotation.frames{i}.polygon)
            
            label = data.annotation.frames{i}.polygon{polygonID}.object + 1;
            
                    
            texts = regexp(data.annotation.objects{label}.name,':','split');
            category = texts{1};
            
            % excluding mirrors
            if isempty(findstr(category,'mirror')) %&& label<=6 %for debugging
                
                mask = poly2mask(data.annotation.frames{i}.polygon{polygonID}.x, data.annotation.frames{i}.polygon{polygonID}.y, 480, 640);
                
                
                isValid = find(reshape(mask & XYZcam(:,:,4)~=0,1,[]));
                
                if ~isempty(isValid)
                    n = length(isValid);

                    % outlier rejection
                    ptCloud = [XYZcam(isValid); XYZcam(640*480+isValid); XYZcam(640*480*2+isValid)];

                    boxRtO2W = fitModel3D(ptCloud, label);

                    ptCloud = transformRT(ptCloud,boxRtO2W,true);



                    isInliers = true(1, n);
                    for dim=1:3
                        sortedV = sort(ptCloud(dim,:));
                        beginV = sortedV(max(1,min(n,round(n*outlierMargin))));
                        endV = sortedV(max(1,min(n,round(n*(1-outlierMargin)))));

                        minSize = max(abs(endV),abs(beginV))*2;

                        isInliers = isInliers & (ptCloud(dim,:) >= beginV) & (ptCloud(dim,:) <= endV);


                        if adjustSize && objectLabel.objectSize(label,dim)<minSize

                            objectLabel.objectSize(label,dim)=minSize + 0.2;

                            fprintf('Object %d: %s has new size (%f,%f,%f). Dim %d changed.\n', label, data.annotation.objects{label}.name, objectLabel.objectSize(label,1), objectLabel.objectSize(label,2), objectLabel.objectSize(label,3),dim);

                        end
                    end
                    isValid = isValid(isInliers);

                    if ~isempty(isValid)
                        n = length(isValid);


                        % subsample isValid
                        n = min(n,max(round(n* subsampleFactor),subsampleMinPoints));
                        isValid = isValid(randsample(length(isValid),n));


                        pointObservedValue(:,pointObservedValueCount+1:pointObservedValueCount+n) = [pixelX(isValid); pixelY(isValid); XYZcam(isValid); XYZcam(640*480+isValid); XYZcam(640*480*2+isValid); (label-1) * ones(1,n)];

                        pointObserved(frameID,pointCount+1:pointCount+n)=pointObservedValueCount+1:pointObservedValueCount+n;


                        pointCloud(:,pointCount+1:pointCount+n) = transformRT([XYZcam(isValid); XYZcam(640*480+isValid); XYZcam(640*480*2+isValid)], cameraRtC2W(:,:,frameID), false);

                        % fit a box
                        boxRt = fitModel3D(pointCloud(:,pointCount+1:pointCount+n), label);


                        % visualization for the point cloud
                        %{

                        colorNow = ObjectColor(label) * (0.5 + 0.5 * frameID / length(fnameList));
                        
                        plot3(pointCloud(1,pointCount+1:pointCount+n),pointCloud(2,pointCount+1:pointCount+n),pointCloud(3,pointCount+1:pointCount+n),...
                            '.', 'markersize', 0.5,'Color',colorNow); hold on;
                        hSize = objectLabel.objectSize(label,:)/2;
                        %hSize = min(10,hSize);
                        axisPoints = [-1 0 0; 1 0 0; 0 -1 0; 0 1 0; 0 0 -1; 0 0 1]';
                        axisPoints(1,:) = axisPoints(1,:) * hSize(1);
                        axisPoints(2,:) = axisPoints(2,:) * hSize(2);
                        axisPoints(3,:) = axisPoints(3,:) * hSize(3);
                        axisPoints = transformRT(axisPoints,boxRt,false);
                        plot3(axisPoints(1,[1 2]),axisPoints(2,[1 2]),axisPoints(3,[1 2]),'-r'); hold on;
                        plot3(axisPoints(1,[3 4]),axisPoints(2,[3 4]),axisPoints(3,[3 4]),'-g'); hold on;
                        plot3(axisPoints(1,[5 6]),axisPoints(2,[5 6]),axisPoints(3,[5 6]),'-b'); hold on;
                        boxPoints = [-1 -1 -1; -1 -1 +1; -1 +1 -1; -1 +1 +1; +1 -1 -1; +1 -1 +1; +1 +1 -1; +1 +1 +1]';
                        boxPoints(1,:) = boxPoints(1,:) * hSize(1);
                        boxPoints(2,:) = boxPoints(2,:) * hSize(2);
                        boxPoints(3,:) = boxPoints(3,:) * hSize(3);
                        boxPoints = transformRT(boxPoints,boxRt,false);
                        plot3(boxPoints(1,[1 2 4 3 1]),boxPoints(2,[1 2 4 3 1]),boxPoints(3,[1 2 4 3 1]),'-','Color',colorNow); hold on;
                        plot3(boxPoints(1,[1 2 4 3 1]+4),boxPoints(2,[1 2 4 3 1]+4),boxPoints(3,[1 2 4 3 1]+4),'-','Color',colorNow); hold on;
                        plot3(boxPoints(1,[1 5]),boxPoints(2,[1 5]),boxPoints(3,[1 5]),'-','Color',colorNow); hold on;
                        plot3(boxPoints(1,[1 5]+1),boxPoints(2,[1 5]+1),boxPoints(3,[1 5]+1),'-','Color',colorNow); hold on;
                        plot3(boxPoints(1,[1 5]+2),boxPoints(2,[1 5]+2),boxPoints(3,[1 5]+2),'-','Color',colorNow); hold on;
                        plot3(boxPoints(1,[1 5]+3),boxPoints(2,[1 5]+3),boxPoints(3,[1 5]+3),'-','Color',colorNow); hold on;
                        title(sprintf('Frame %d',frameID))
                        axis equal;
                        grid on;
                        %}

                        if objectLabel_first(label) < n
                            % initialize objectRtO2W
                            objectLabel_first(label) = n;
                            objectLabel.objectRtO2W(:,:,label) = boxRt;
                            
                            %{
                            colorNow = ObjectColor(label) * (0.5 + 0.5 * frameID / length(fnameList));

                            plot3(pointCloud(1,pointCount+1:pointCount+n),pointCloud(2,pointCount+1:pointCount+n),pointCloud(3,pointCount+1:pointCount+n),...
                                '.', 'markersize', 0.5,'Color',colorNow); hold on;
                            
                            %}
                            
                        end

                        % update counter
                        pointObservedValueCount = pointObservedValueCount+n;
                        pointCount = pointCount+n;

                        objectLabel.optimizationWeight(label) = objectLabel.optimizationWeight(label) + n;
                    end
                end                    
                    
            
            end
        end
        
        
    end
end

  
objectLabel.objectSize = objectLabel.objectSize * resizingScale;

cntPoints = objectLabel.optimizationWeight;

minPointsThreshold = 50;

if normalizeWeight
    objectLabel.optimizationWeight(cntPoints<minPointsThreshold) = 0;
    
    numberObject = sum(objectLabel.optimizationWeight>0);
    objectLabel.optimizationWeight = sqrt(objectWeightTotal)./sqrt(max(1,objectLabel.optimizationWeight * numberObject));
    
    %if max(objectLabel.optimizationWeight)>1
    %    maxWeight = max(objectLabel.optimizationWeight);
    %    objectLabel.optimizationWeight = sqrt(objectLabel.optimizationWeight/maxWeight);
    %end
else
    objectLabel.optimizationWeight = objectWeightTotal * ones(size(objectLabel.optimizationWeight));
end
objectLabel.optimizationWeight(cntPoints<minPointsThreshold) = 0;

[vv,ind]=sort(cntPoints,'descend');
for objectID=ind
    if cntPoints(objectID)>0
        fprintf('Object %d  CNT=%d  TYPE=%d  WEIGHT=%f  Size %.2fx%.2fx%.2f: %s\n',...
            objectID,cntPoints(objectID),objectLabel.objectType(objectID),objectLabel.optimizationWeight(objectID),...
            objectLabel.objectSize(objectID,1),objectLabel.objectSize(objectID,2),objectLabel.objectSize(objectID,3),...
            data.annotation.objects{objectID}.name);
    end
end

%{
%figure(100)
%clf
plot3(reshape(cameraRtC2W(1,4,:),1,[]),reshape(cameraRtC2W(2,4,:),1,[]),reshape(cameraRtC2W(3,4,:),1,[]),'.')
hold on;
axis equal;
for label=1:objectLabel.length
    if objectLabel.optimizationWeight(label)>0
        %if objectLabel.objectType(label)==4 || 
            hSize = objectLabel.objectSize(label,:)/2;
            
            boxPoints = [-1 -1 -1; -1 -1 +1; -1 +1 -1; -1 +1 +1; +1 -1 -1; +1 -1 +1; +1 +1 -1; +1 +1 +1]';
            boxPoints(1,:) = boxPoints(1,:) * hSize(1);
            boxPoints(2,:) = boxPoints(2,:) * hSize(2);
            boxPoints(3,:) = boxPoints(3,:) * hSize(3);
            boxPoints = transformRT(boxPoints,objectLabel.objectRtO2W(:,:,label),false);
            
            colorNow = ObjectColor(label) ;
            
            plot3(boxPoints(1,[1 2 4 3 1]),boxPoints(2,[1 2 4 3 1]),boxPoints(3,[1 2 4 3 1]),'-','Color',colorNow); hold on;
            plot3(boxPoints(1,[1 2 4 3 1]+4),boxPoints(2,[1 2 4 3 1]+4),boxPoints(3,[1 2 4 3 1]+4),'-','Color',colorNow); hold on;
            plot3(boxPoints(1,[1 5]),boxPoints(2,[1 5]),boxPoints(3,[1 5]),'-','Color',colorNow); hold on;
            plot3(boxPoints(1,[1 5]+1),boxPoints(2,[1 5]+1),boxPoints(3,[1 5]+1),'-','Color',colorNow); hold on;
            plot3(boxPoints(1,[1 5]+2),boxPoints(2,[1 5]+2),boxPoints(3,[1 5]+2),'-','Color',colorNow); hold on;
            plot3(boxPoints(1,[1 5]+3),boxPoints(2,[1 5]+3),boxPoints(3,[1 5]+3),'-','Color',colorNow); hold on;
            hold on;
            
            centerPoint = transformRT([0;0;0],objectLabel.objectRtO2W(:,:,label),false);

            text(centerPoint(1),centerPoint(2),centerPoint(3),sprintf('%d: %s',label, data.annotation.objects{label}.name),'Color',colorNow);
            hold on;
        %end
    end
end
axis equal;
grid on;

%}



%% visualization
%{
% per image
for frameID = 1:frames.length-1
    labelMap = readLabel(frames, frameID);
    if ~isempty(labelMap)
        
        figure(frameID)
        clf
        
        labels = unique(labelMap(:));
        labels = labels(labels~=0)';
        if ~isempty(labels)
            
            XYZcam=readDepth(frames,frameID);
            
            for label=labels
                isValid = find(reshape((labelMap == label) & XYZcam(:,:,4)~=0,1,[]));
                
                if ~isempty(isValid)
                    n = length(isValid);
                    
                    

                        % outlier rejection
                        ptCloud = [XYZcam(isValid); XYZcam(640*480+isValid); XYZcam(640*480*2+isValid)];
                    
                            boxRtO2W = fitModel3D(ptCloud, label);


%                         ptCloud = transformRT(ptCloud,boxRtO2W,true);
%                         
%                         
%                         isInliers = true(1, n);
%                         for dim=1:3
%                             sortedV = sort(ptCloud(dim,:));                        
%                             beginV = sortedV(max(1,min(n,round(n*outlierMargin))));
%                             endV = sortedV(max(1,min(n,round(n*(1-outlierMargin)))));
%                             
%                             isInliers = isInliers & (ptCloud(dim,:) >= beginV) & (ptCloud(dim,:) <= endV);
%                         end
%                         isValid = isValid(isInliers);
% 
%                         ptCloud = ptCloud(:,isValid);

                        
                    
                    % fit a box
                    boxRt = fitModel3D(pointCloud(:,pointCount+1:pointCount+n), label);
                    
                    % visualization for the point cloud
                    plot3(ptCloud(1,:),ptCloud(2,:),ptCloud(3,:),'.', 'markersize', 0.5,'Color',objectLabel.colorList(label,:)/255 ); hold on;
                    
                    
                    hSize = min(1,objectLabel.objectSize(label,:)/2);
                    axisPoints = [-1 0 0; 1 0 0; 0 -1 0; 0 1 0; 0 0 -1; 0 0 1]';
                    axisPoints(1,:) = axisPoints(1,:) * hSize(1);
                    axisPoints(2,:) = axisPoints(2,:) * hSize(2);
                    axisPoints(3,:) = axisPoints(3,:) * hSize(3);
                    axisPoints = transformRT(axisPoints,boxRt,false);
                    plot3(axisPoints(1,[1 2]),axisPoints(2,[1 2]),axisPoints(3,[1 2]),'-r'); hold on;
                    plot3(axisPoints(1,[3 4]),axisPoints(2,[3 4]),axisPoints(3,[3 4]),'-g'); hold on;
                    plot3(axisPoints(1,[5 6]),axisPoints(2,[5 6]),axisPoints(3,[5 6]),'-b'); hold on;
                    boxPoints = [-1 -1 -1; -1 -1 +1; -1 +1 -1; -1 +1 +1; +1 -1 -1; +1 -1 +1; +1 +1 -1; +1 +1 +1]';
                    boxPoints(1,:) = boxPoints(1,:) * hSize(1);
                    boxPoints(2,:) = boxPoints(2,:) * hSize(2);
                    boxPoints(3,:) = boxPoints(3,:) * hSize(3);
                    boxPoints = transformRT(boxPoints,boxRt,false);
                    plot3(boxPoints(1,[1 2 4 3 1]),boxPoints(2,[1 2 4 3 1]),boxPoints(3,[1 2 4 3 1]),'-','Color',objectLabel.colorList(label,:)/255); hold on;
                    plot3(boxPoints(1,[1 2 4 3 1]+4),boxPoints(2,[1 2 4 3 1]+4),boxPoints(3,[1 2 4 3 1]+4),'-','Color',objectLabel.colorList(label,:)/255); hold on;
                    plot3(boxPoints(1,[1 5]),boxPoints(2,[1 5]),boxPoints(3,[1 5]),'-','Color',objectLabel.colorList(label,:)/255); hold on;
                    plot3(boxPoints(1,[1 5]+1),boxPoints(2,[1 5]+1),boxPoints(3,[1 5]+1),'-','Color',objectLabel.colorList(label,:)/255); hold on;
                    plot3(boxPoints(1,[1 5]+2),boxPoints(2,[1 5]+2),boxPoints(3,[1 5]+2),'-','Color',objectLabel.colorList(label,:)/255); hold on;
                    plot3(boxPoints(1,[1 5]+3),boxPoints(2,[1 5]+3),boxPoints(3,[1 5]+3),'-','Color',objectLabel.colorList(label,:)/255); hold on;
                    title(sprintf('Frame %d',frameID))
                    axis equal;
                    grid on;
                    
                end
            end
        end
    end
end

% per object

clear labelMap
clear XYZcam
for cameraID = 1:frames.length-1
    labelMap{cameraID} = readLabel(frames, cameraID);
    if ~isempty(labelMap{cameraID})
        XYZcam{cameraID}=readDepth(frames,cameraID);
    end
end

for label = 1:objectLabel.length
for label = [3 67 23 35 48 47 33 42 12 38 58 71 24 68]
        figure(label);
        n = 1;
        for cameraID = 1:frames.length-1
            if ~isempty(labelMap{cameraID})
                labels = unique(labelMap{cameraID}(:));
                labels = labels(labels~=0)';
                if any(labels==label)

                    image = readImage(frames, cameraID);

                    image(:,:,1) = image(:,:,1) / 2;
                    vM = reshape(labelMap{cameraID} == label,1,[]);
                    image(vM) = image(vM)*2;
                    subplot(6,6,n); n = n+1;
                    imagesc(image); axis equal; axis off
                end
            end
        end
end

%}

%%
%{
LineWidth = 4;
for label = 1:objectLabel.length
    if ~isempty(data.annotation.objects{label})
        
        texts = regexp(data.annotation.objects{label}.name,':','split');
        category = texts{1};
        
        figure(label)
        fprintf('%s\n',data.annotation.objects{label}.name);
        fcnt = 0;
        %figure(label)
        for i=1:length(data.annotation.frames)
            if ~isempty(data.annotation.frames{i})
                for polygonID = 1:length(data.annotation.frames{i}.polygon)
                    if label == data.annotation.frames{i}.polygon{polygonID}.object+1
                        frameID = find(ismember(fnameList,data.annotation.fileList{i}(1:end-4)));
                        
                        fcnt= fcnt+1;
                        
                        try
                            subplot(4,4,fcnt);
                        catch
                        end
                        im = im2double(imread(data.image{frameID}));
                        X = data.annotation.frames{i}.polygon{polygonID}.x;
                        Y = data.annotation.frames{i}.polygon{polygonID}.y;
                        colorNow = ObjectColor(label);                        
                        
                        mask = poly2mask(X, Y, 480, 640);
                        
                        im = im * 0.7 + 0.3 * repmat(double(mask),[1,1,3]) .* repmat(reshape(colorNow,1,1,3),[480,640]);
                        
                        imshow(im); hold on;
                        plot([X X(1)],[Y Y(1)], 'LineWidth', LineWidth, 'Color', [0 0 0]); hold on;
                        plot([X X(1)],[Y Y(1)], 'LineWidth', LineWidth/2, 'Color', colorNow); hold on;
                        title(sprintf('%s (%d,%d)',data.annotation.objects{label}.name, i, frameID));
                    end
                end
            end
        end
        
    end
end

%% per object visualization
for label = 1:objectLabel.length
    
    if ~isempty(data.annotation.objects{label})
    
        texts = regexp(data.annotation.objects{label}.name,':','split');
        category = texts{1};

        if isempty(findstr(category,'mirror'))
            fprintf('%s = ',data.annotation.objects{label}.name);
            %figure(label)
            for i=1:length(data.annotation.frames)
                if ~isempty(data.annotation.frames{i})
                    for polygonID = 1:length(data.annotation.frames{i}.polygon)
                        if label == data.annotation.frames{i}.polygon{polygonID}.object+1
                            frameID = find(ismember(fnameList,data.annotation.fileList{i}(1:end-4)));
                            fprintf('(i%d f%d) ',i, frameID);

                            %image  = imread(data.image{frameID});
                            XYZcam = depth2XYZcamera(data.K, depthRead(data.depth{frameID}));


                            mask = poly2mask(data.annotation.frames{i}.polygon{polygonID}.x, data.annotation.frames{i}.polygon{polygonID}.y, 480, 640);


                            isValid = find(reshape(mask & XYZcam(:,:,4)~=0,1,[]));

                            if ~isempty(isValid)
                                n = length(isValid);


                                % outlier rejection
                                ptCloud = [XYZcam(isValid); XYZcam(640*480+isValid); XYZcam(640*480*2+isValid)];

                                boxRtO2W = fitModel3D(ptCloud, label);

                                ptCloud = transformRT(ptCloud,boxRtO2W,true);


                                isInliers = true(1, n);
                                for dim=1:3
                                    sortedV = sort(ptCloud(dim,:));
                                    beginV = sortedV(max(1,min(n,round(n*outlierMargin))));
                                    endV = sortedV(max(1,min(n,round(n*(1-outlierMargin)))));

                                    isInliers = isInliers & (ptCloud(dim,:) >= beginV) & (ptCloud(dim,:) <= endV);
                                end
                                isValid = isValid(isInliers);

                                %ptCloud = ptCloud(:,isValid);

                                %colorNow = ObjectColor(label) ;
                                colorNow = ObjectColor(label); % * (0.5 + 0.5 * frameID / length(fnameList));

                                ptCloud = transformRT([XYZcam(isValid); XYZcam(640*480+isValid); XYZcam(640*480*2+isValid)], cameraRtC2W(:,:,frameID), false);
                                plot3(ptCloud(1,1:10:end),ptCloud(2,1:10:end),ptCloud(3,1:10:end),'.', 'markersize', 0.5,'Color', colorNow); hold on;
                            end
                        end
                    end
                end
            end            
            title(data.annotation.objects{label}.name)

            % visualization for the point cloud

            %axisPoints = [-1 0 0; 1 0 0; 0 -1 0; 0 1 0; 0 0 -1; 0 0 1]';
            %axisPoints = transformRT(axisPoints,objectLabel.objectRtO2W(:,:,label),false);
            %plot3(axisPoints(1,[1 2]),axisPoints(2,[1 2]),axisPoints(3,[1 2]),'-r'); hold on;
            %plot3(axisPoints(1,[3 4]),axisPoints(2,[3 4]),axisPoints(3,[3 4]),'-g'); hold on;
            %plot3(axisPoints(1,[5 6]),axisPoints(2,[5 6]),axisPoints(3,[5 6]),'-b'); hold on;

            colorNow = ObjectColor(label) ;

            boxPoints = [-1 -1 -1; -1 -1 +1; -1 +1 -1; -1 +1 +1; +1 -1 -1; +1 -1 +1; +1 +1 -1; +1 +1 +1]';
            hSize = objectLabel.objectSize(label,:)/2;
            hSize = min(hSize, 4);
            boxPoints(1,:) = boxPoints(1,:) * hSize(1);
            boxPoints(2,:) = boxPoints(2,:) * hSize(2);
            boxPoints(3,:) = boxPoints(3,:) * hSize(3);
            boxPoints = transformRT(boxPoints,objectLabel.objectRtO2W(:,:,label),false);
            plot3(boxPoints(1,[1 2 4 3 1]),boxPoints(2,[1 2 4 3 1]),boxPoints(3,[1 2 4 3 1]),'-k','Color',colorNow); hold on;
            plot3(boxPoints(1,[1 2 4 3 1]+4),boxPoints(2,[1 2 4 3 1]+4),boxPoints(3,[1 2 4 3 1]+4),'-k','Color',colorNow); hold on;
            plot3(boxPoints(1,[1 5]),  boxPoints(2,[1 5]),  boxPoints(3,[1 5]),  '-k','Color',colorNow); hold on;
            plot3(boxPoints(1,[1 5]+1),boxPoints(2,[1 5]+1),boxPoints(3,[1 5]+1),'-k','Color',colorNow); hold on;
            plot3(boxPoints(1,[1 5]+2),boxPoints(2,[1 5]+2),boxPoints(3,[1 5]+2),'-k','Color',colorNow); hold on;
            plot3(boxPoints(1,[1 5]+3),boxPoints(2,[1 5]+3),boxPoints(3,[1 5]+3),'-k','Color',colorNow); hold on;



            %centerPoint = transformRT([0;0;0],objectLabel.objectRtO2W(:,:,label),false);
            %text(centerPoint(1),centerPoint(2),centerPoint(3),sprintf('%d: %s',label, data.annotation.objects{label}.name),'Color',colorNow);
            %hold on;


            axis equal
            grid on
            axis tight

            xlabel('x')
            ylabel('y')
            zlabel('z')
            
            fprintf('\n');

            drawnow
            %pause
        end
    end
end
%}

%% bundle adjustment again
fprintf('bundle adjusting    ...\n');
tic
[cameraRtC2W,pointCloud] = bundleAdjustment2D3DBoxFileType(cameraRtC2W,pointCloud,pointObserved, pointObservedValue, data.K, w3D, 3);
toc;
  
%% output result

% output keypoint ply


% output ply

outputKeyframePly(fullfile(write2path, sequenceName, 'box_BA_type_Keyframes.ply'), cameraRtC2W, data,fnameList);

outputPly(fullfile(write2path, sequenceName, 'box_BA_type.ply'), cameraRtC2W, data);


%outputKeypointsPly(sprintf('%s_box_key.ply',frames.sfmFileName),pointCloud(:,reshape(find(sum(pointObserved~=0,1)>0),1,[])));
%outputKeypointsPly(sprintf('%s_box_key.ply',frames.sfmFileName),pointCloud(:,1:pointCountOld));
%outputKeypointsPly(fullfile(write2path, sequenceName, 'box_key_type.ply'),pointCloud(:,1:pointCountOld));

%plot3(reshape(cameraRtC2W(1,4,:),1,[]),reshape(cameraRtC2W(2,4,:),1,[]),reshape(cameraRtC2W(3,4,:),1,[]),'-k'); hold on;
%plot3(reshape(cameraRtC2W(1,4,KeyframeID),1,[]),reshape(cameraRtC2W(2,4,KeyframeID),1,[]),reshape(cameraRtC2W(3,4,KeyframeID),1,[]),'*r'); hold on;


%{
% visualize camera
%plot3(reshape(cameraRtC2W(1,4,:),1,[]),reshape(cameraRtC2W(2,4,:),1,[]),reshape(cameraRtC2W(3,4,:),1,[]),'-k'); hold on;
for i=1:size(cameraRtC2W,3)
    cameraPtsNow = transformRT(cameraPts, cameraRtC2W(:,:,i));
    plot3(cameraPtsNow(1,[1 2 3 4 5 2]),cameraPtsNow(2,[1 2 3 4 5 2]),cameraPtsNow(3,[1 2 3 4 5 2]),'-'); hold on;
    plot3(cameraPtsNow(1,[1 3]),cameraPtsNow(2,[1 3]),cameraPtsNow(3,[1 3]),'-'); hold on;
    plot3(cameraPtsNow(1,[1 4]),cameraPtsNow(2,[1 4]),cameraPtsNow(3,[1 4]),'-'); hold on;
    plot3(cameraPtsNow(1,[1 5]),cameraPtsNow(2,[1 5]),cameraPtsNow(3,[1 5]),'-'); hold on;
    axis equal
    drawnow
    
    if i>1
        plot3(reshape(cameraRtC2W(1,4,[i-1 i]),1,[]),reshape(cameraRtC2W(2,4,[i-1 i]),1,[]),reshape(cameraRtC2W(3,4,[i-1 i]),1,[]),'-k'); hold on;
    end
    %pause
end



% visualize keyframe camera
for i=1:length(data.annotation.frames)
    if ~isempty(data.annotation.frames{i})
        frameID = find(ismember(fnameList,data.annotation.fileList{i}(1:end-4)));
        
        cameraPtsNow = transformRT(cameraPts, cameraRtC2W(:,:,frameID));
        plot3(cameraPtsNow(1,[1 2 3 4 5 2]),cameraPtsNow(2,[1 2 3 4 5 2]),cameraPtsNow(3,[1 2 3 4 5 2]),'-r'); hold on;
        plot3(cameraPtsNow(1,[1 3]),cameraPtsNow(2,[1 3]),cameraPtsNow(3,[1 3]),'-r'); hold on;
        plot3(cameraPtsNow(1,[1 4]),cameraPtsNow(2,[1 4]),cameraPtsNow(3,[1 4]),'-r'); hold on;
        plot3(cameraPtsNow(1,[1 5]),cameraPtsNow(2,[1 5]),cameraPtsNow(3,[1 5]),'-r'); hold on;
        axis equal
        drawnow
        %pause
    end
end
%}


% output 3D point cloud for each object?

% output inlier points

% output inlier points with TSDF + marching cube surface

% output bounding box points

% output bounding box + TSDF + marching cube surface

% output other shit

%save([frames.matFileName '_cameraRt.mat'], 'frames','cameraRtC2W','-v7.3');
%save(fullfile(write2path, sequenceName, 'cameraRt_type.mat'), 'data','cameraRtC2W','-v7.3');

%% rectify

%{
disp(frames.matFileName); % for andrew
addpath('/data/vision/torralba/sun3d/app/src/flow_v1/');
cameraRtC2W = rectify_cameras(cameraRtC2W, frames);
save([frames.matFileName '_cameraRt_rectify.mat'], 'frames','cameraRtC2W','-v7.3');
%}

%% output ply
%clusterID = ones(1,frames.length);
%for cluster=unique(clusterID)
%    outputPly(sprintf('%s_%d_%dframes.ply',frames.sfmFileName,cluster, sum(clusterID==cluster)), cameraRt, frames, find(clusterID==cluster));
%end
%outputPly(fullfile(fileparts(frames.sfmFileName),'index.ply'), cameraRtC2W, frames, 1:frames.length);

%% output bin
%for cluster=unique(clusterID)
%    outputBin(sprintf('%s_%d_%dframes',frames.sfmFileName,cluster, sum(clusterID==cluster)), cameraRt, frames, find(clusterID==cluster),sequenceName);
%end

%outputBin(fullfile(fileparts(frames.sfmFileName),'index'), cameraRtC2W, frames, 1:frames.length,sequenceName);

%% output thumbnail
%{
[X, ~, C] = world_pts(cameraRtC2W, frames, 1:frames.length, min(1,50/frames.length));
im = pt_im(X, C, [800 800]); % 500x500 image
imwrite(im,fullfile(fileparts(frames.sfmFileName),'thumb.jpg')); 
%}
%{
try
rmdir(fullfile(matPath,[hname '.json']));
catch
end
%}


timeStamp = getTimeStamp();

%% output camera text file
outputCameraExtrinsics(SUN3Dpath, sequenceName, cameraRtC2W, timeStamp);

%% output thumbnail
outputThumbnail(SUN3Dpath, sequenceName, cameraRtC2W, timeStamp, data);

return;
