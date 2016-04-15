function [agreeRatio, averageDistance, goodpixels, pt]=depthConsistencyParallel(Rt, threshold, XYZcam_i, XYZcam_j, K)


P3D =  [reshape(XYZcam_i(:,:,1),1,[]);
        reshape(XYZcam_i(:,:,2),1,[]);
        reshape(XYZcam_i(:,:,3),1,[])];
P3D = P3D(:,reshape(XYZcam_i(:,:,4)~=0,1,[]));
P3D = transformRT(P3D,Rt,true);
P2D = K * P3D;
P2D = round(P2D(1:2,:)./P2D([3 3],:));
Pvalid = 0< P2D(1,:) & P2D(1,:)<640 & 0< P2D(2,:) & P2D(2,:)<480 & ~isnan(P2D(1,:)) & ~isnan(P2D(2,:));

if ~any(Pvalid)
    agreeRatio = 0;
    averageDistance = 0;
    goodpixels = 0;
    pt = Inf;
    return;
end

linearInd = sub2ind([480 640], P2D(2,Pvalid), P2D(1,Pvalid));
DepthMap = DepthMex(480,640,double(linearInd-1),double(P3D(3,Pvalid)));
compMap = logical((DepthMap~=0) .* XYZcam_j(:,:,4));
diffMap = abs(DepthMap - XYZcam_j(:,:,3));

%thresholdMap = depthUncertainty(XYZcam(:,:,3,cluster_j), 0.075, 570) * threshold;



agreeMap = (diffMap<threshold) & compMap;
goodpixels = sum(compMap(:));
agreeRatio = sum(agreeMap(:))/goodpixels;
averageDistance=mean(diffMap(agreeMap(:)));
%goodpixels = goodpixels / (640*480);

goodpixels = goodpixels / sum(reshape(logical(XYZcam_j(:,:,4)),1,[]));


%{
figure(1)
imagesc(DepthMap); axis equal; axis tight; axis off;

figure(2)
imagesc(XYZcam_j(:,:,3)); axis equal; axis tight; axis off;

figure(3)
imagesc(diffMap.*compMap); axis equal; axis tight; axis off;

figure(4)
imagesc(agreeMap); axis equal; axis tight; axis off;

figure(5)
clf
hist(reshape(diffMap.*compMap,1,[]),0:0.01:1)
hold on
plot([threshold threshold],[0 640*480/10],'-r');


drawnow
%}

diffMap = diffMap(compMap);
%pt = prctile(diffMap,[50,60,70,80,90]);
pt = prctile(diffMap,[70]);
