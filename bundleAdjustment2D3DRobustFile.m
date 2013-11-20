function [cameraRtC2W,pointCloud] = bundleAdjustment2D3DRobustFile(cameraRtC2W,pointCloud,pointObserved, pointObservedValue, K, weight, mode)


[camID,ptsID,valID] = find(pointObserved);


nCam=uint32(size(cameraRtC2W,3));
nPts=uint32(size(pointCloud,2));
nObs=uint32(length(valID));
fx=K(1,1); 
fy=K(2,2); 
px=K(1,3); 
py=K(2,3);

cameraRtW2C = zeros(3,4,size(cameraRtC2W,3));
% transform the cameraRt
for cameraID=1:size(cameraRtC2W,3)    
    cameraRtW2C(:,:,cameraID) = transformCameraRt(cameraRtC2W(:,:,cameraID));
end

fname_inout = tempname;
fname_in = [fname_inout '.in'];
fname_out = [fname_inout '.out'];


fin = fopen(fname_in, 'wb');
fwrite(fin, nCam, 'uint32');
fwrite(fin, nPts, 'uint32');
fwrite(fin, nObs, 'uint32');
fwrite(fin, fx, 'double');
fwrite(fin, fy, 'double');
fwrite(fin, px, 'double');
fwrite(fin, py, 'double');

fwrite(fin, cameraRtW2C, 'double');
fwrite(fin, pointCloud, 'double');

% write observation  
ptsObservedIndex = uint32([camID,ptsID]-1)';
ptsObservedValue = pointObservedValue(:,valID);
fwrite(fin, ptsObservedIndex, 'uint32');
fwrite(fin, ptsObservedValue, 'double');

fclose(fin);

cmd = sprintf('./ba2D3D %d %f %s %s', mode, weight, fname_in, fname_out);
fprintf('%s\n',cmd);
system(cmd);

% read the result back;
fout = fopen(fname_out, 'rb');
nCam=fread(fout,1,'uint32');
nPts=fread(fout,1,'uint32');
cameraRtW2C = fread(fout,12*nCam,'double');
pointCloud = fread(fout,3*nPts,'double');
fclose(fout);

cameraRtW2C = reshape(cameraRtW2C,3,4,[]);
pointCloud = reshape(pointCloud,3,[]);

% transform the cameraRt back
for cameraID=1:size(cameraRtW2C,3)
    cameraRtC2W(:,:,cameraID) = transformCameraRt(cameraRtW2C(:,:,cameraID));
end

delete(fname_in);
delete(fname_out);
