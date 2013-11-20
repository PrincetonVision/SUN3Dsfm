function outputPly(PLYfilename, cameraRtC2W, data)

fprintf('Writing ply point cloud file: ');

tic


frameInterval = 10;

pointCount = round(min(640*480*0.5,1000000/(length(data.image)/frameInterval)));


dataChunk = uint8([]);
for frameID=1:frameInterval:length(data.image)

    image = imread(data.image{frameID});
    XYZcam = depth2XYZcamera(data.K, depthRead(data.depth{frameID}));        
    
    XYZcam = reshape(XYZcam,640*480,4)';
    isValid = find(XYZcam(4,:));
    XYZcam = transformRT(XYZcam(1:3,:),cameraRtC2W(:,:,frameID));
    try
        selID = randsample(length(isValid),pointCount);
    catch
        selID = randsample(length(isValid),pointCount,true);
    end
    isValid = isValid(selID);
 
    RGB =  reshape(image, 640*480, 3)';
    
    
    dataChunk = [dataChunk [reshape(typecast(reshape(single(XYZcam(:,isValid)),1,[]),'uint8'),3*4,[]); RGB(:,isValid)]];
end




file = writePLYhead(PLYfilename, size(dataChunk,2));
fwrite(file, dataChunk,'uint8');
fclose(file);

toc;