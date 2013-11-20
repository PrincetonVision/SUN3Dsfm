function outputThumbnail(SUN3Dpath, sequenceName, cameraRtC2W, timeStamp, data)

outTo = fullfile(SUN3Dpath, sequenceName,'thumbnail');
if ~exist(outTo,'dir')
    mkdir(outTo);
end

heightCutoff = 2;

for frameID=1:length(data.image)
    IMcamera = imread(data.image{frameID});
    XYZcamera = depth2XYZcamera(data.K, depthRead(data.depth{frameID}));
    
    % pick the valid points with their color
    valid = logical(XYZcamera(:,:,4));  valid = valid(:)';
    XYZ = reshape(XYZcamera,[],4)';
    RGB = reshape(IMcamera,[],3)';
    XYZ = XYZ(1:3,valid);
    RGB = RGB(:,valid);
    
    RGB = double(RGB)/255;
    
    % transform to world coordinate
    XYZworld = transformPointCloud(XYZ,cameraRtC2W(:,:,frameID));
    
    % to get a top view
    XYZworld([2 3],:) = XYZworld([3 2],:);
    XYZworld(2,:) = -XYZworld(2,:);
    
    % remove ceiling
    valid = XYZworld(3,:) <heightCutoff ;
    XYZworld = XYZworld(:,valid);
    RGB = RGB(:,valid);
    
    if frameID == 1
        [image,depth, info]=point2render(XYZworld, RGB);
    else
        [image,depth]=point2render(XYZworld, RGB, image, depth, info);
    end
end

thumbnail = autoCropImage(image);

imwrite(thumbnail, fullfile(outTo,[timeStamp '.jpg']));