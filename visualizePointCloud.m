
function visualizePointCloud(XYZ,RGB, subsampleGap)
    if ~exist('subsampleGap','var')
        subsampleGap = 50;
    end
    XYZ = XYZ(:,1:subsampleGap:end);
    RGB = RGB(:,1:subsampleGap:end);
    scatter3(XYZ(1,:),XYZ(2,:),XYZ(3,:),ones(1,size(XYZ,2)),double(RGB)'/255,'filled');
    axis equal
    axis tight
end