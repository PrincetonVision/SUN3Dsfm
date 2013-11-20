function cameraRt = rectifyScene(cameraRt, data, frameSubset)
% Rectify cameraRt using the points and cameras from the frameSubset (or sample them if it's not provided)
SAMPLE_SIZE = 1000;

if nargin < 3
  if size(cameraRt, 3) > SAMPLE_SIZE
    frameSubset = randsample(1 : size(cameraRt, 3), SAMPLE_SIZE);
  else
    frameSubset = 1 : size(cameraRt, 3);
  end
end

X = worldPts(cameraRt, data, frameSubset, 0.25);
[R world_center] = dominantAxes(cameraRt(:, :, frameSubset), X, data, frameSubset);

for i = 1 : size(cameraRt, 3)
  cameraRt(:, 1 : 3, i) = R * cameraRt(:, 1 : 3, i);
  cameraRt(:, 4, i) = R*(cameraRt(:, 4, i) - world_center); % + world_center;
end


disp('Rectifying with rotation matrix:');
disp(R);

% Shift the cameras so that the floor is at y = 0
X_new = worldPts(cameraRt, data, frameSubset, 0.25);
floor_y = prctile(X_new(2,:), 3);
cameraRt(:, 4, :) = bsxfun(@minus, cameraRt(:, 4, :), [median(X_new(1, :)); floor_y; median(X_new(3, :))]);


function [R world_center] = dominantAxes(cameraRt, pts, data, frame_sub)
sample_rate = 100;

normals = {};
for fi = 1 : length(frame_sub)
  XYZ = readDepthScale(data, frame_sub(fi), 0.25);
  S = 10;
  V1 = XYZ(1 + S : end, 1 : end - S, 1:3) - XYZ(1 : end - S, 1 : end - S, 1:3);
  V2 = XYZ(1 : end - S, 1 + S : end, 1:3) - XYZ(1 : end - S, 1 : end - S, 1:3);

  ok = XYZ(1 : end - S, 1 : end - S, 4) > 0;
  C = cross(V1, V2);
  
  ok = reshape(ok(:), 1, []);
  nrm = reshape(C(:), [], 3)';
  nrm = nrm(:, ok);
  nrm = nrm(:, ~all(nrm == 0, 1));
  nrm = nrm ./ repmat(sqrt(sum(nrm .^ 2, 1)), [3 1]);
  nrm = cameraRt(:, 1:3, fi) * nrm;

  normals{fi} = nrm;
end
normals = cell2mat(normals);


% approximately 1300 bins
sphere = icosahedron2sphere(4)';
bins = sphere(:, sphere(1, :) >= 0);

NSAMPLE = 1e5;
normals = normals(:, randsample(1 : size(normals, 2), min(size(normals, 2), NSAMPLE)));
[D, B] = max(abs(bins' * normals), [], 1);

H = accumarray(cat(2, B', repmat(1, [length(B) 1])), repmat(1, [length(B) 1]));

A = zeros(3);
[~, I] = sort(-H);
for j = 1 : 3
  b = I(1);

  % choose mean normal that falls into the biggest bin
  in_bin = normals(:, B == b);
  % flip mirrored normals
  dots = sum(in_bin .* repmat(bins(:, b), [1 size(in_bin, 2)]), 1);
  in_bin(:, (dots < 0)) = -in_bin(:, (dots < 0));
  v = mean(in_bin, 2);
  v = v / norm(v);
  
  A(:, j) = v;

  %fprintf('Bin: %d, Normal: %f %f %f. Contains %d points. Mean vector: %f %f %f\n', b, bins(:, b), H(b), v);
  % remove bins that are not ~90 degrees away
  dots = sum(bins(:, I) .* repmat(v, [1 length(I)]), 1);
  I = I((dots >= cos(deg2rad(100))) & (dots <= cos(deg2rad(80))));
end


% assume most cameras were upright, and map each camera's local axis to world coordinates
% camera y-axis in world coordinates: cameraRt(1:3, :, j) * [0 -1 0] = -cameraRt(1:3, 2, j) 
cam_y = squeeze(-cameraRt(1:3, 2, :));
% consider both + and - versions of axes
dots = [A'; -A'] * cam_y;
% use abs, since the estimated y axis might be upside down
[~, I] = max(dots, [], 1);
counts = accumarray(cat(2, I', repmat(1, [length(I) 1])), repmat(1, [length(I) 1]));
[~, yi] = max(counts);
if yi > 3
  yi = yi - 3;
  A(:, yi) = -A(:, yi);
end
Y = A(:, yi);

% choose X
if yi > 3
  if yi == 4
    X = A(:, 5);
  else
    X = A(:, 4);
  end
else
  if yi == 1
    X = A(:, 2);
  else
    X = A(:, 1);
  end
end

% remove the component of X that's parallel to Y, so that we have a valid rotation matrix
%fprintf('Degree difference between X and Y: %f before making orthogonal\n', rad2deg(acos(X'*Y)));
X = X - (Y'*X)*Y;
X = X/norm(X);

Z = cross(X, Y);
Z = Z/norm(Z);

R = [X Y Z]';
world_center = mean(pts, 2);


function [XYZ_all ij] = worldPts(cameraRt, data, frame_subset, im_scale)
if nargin < 4
  im_scale = 1.;
end

XYZ_all = zeros(3, 0);
%colors = uint8(zeros(3, 0));

for i = 1 : length(frame_subset)
  frame = frame_subset(i);
  XYZcam=readDepthScale(data, frame, im_scale);
  sz = size(XYZcam);
  sz = sz(1:2);
  Xcam = XYZcam(:,:,1);
  Ycam = XYZcam(:,:,2);
  Zcam = XYZcam(:,:,3);  
  XYZ1world = cameraRt(:,1:3,frame) * [Xcam(:)'; Ycam(:)'; Zcam(:)'] + repmat(cameraRt(:,4,frame),1,sz(1)*sz(2));
  XYZworld (:,:,1) = reshape(XYZ1world(1,:),sz);
  XYZworld (:,:,2) = reshape(XYZ1world(2,:),sz);
  XYZworld (:,:,3) = reshape(XYZ1world(3,:),sz);
  XYZworld (:,:,4) = XYZcam (:,:,4);    

  Xworld = XYZworld(:,:,1);
  Yworld = XYZworld(:,:,2);
  Zworld = XYZworld(:,:,3);
  validM = logical(XYZworld(:,:,4));
  XYZworldframe = [Xworld(:)'; Yworld(:)'; Zworld(:)'];
  valid = validM(:)';
  XYZworldframe = XYZworldframe(:,valid);

  sz = size(XYZworld);
  assert(numel(valid) == size(XYZworld, 1)*size(XYZworld, 2));
  [i j] = ind2sub(sz(1:2), find(valid));
  ij = [i; j];

  assert(size(ij, 2) == size(XYZworldframe, 2));
  XYZworldframe = double(XYZworldframe);
  XYZ_all = [XYZ_all XYZworldframe];
  
  % im = imresize(readImage(data, frame), im_scale);
  % colors = [colors zip3(im, ij)];
end

function xyz = readDepthScale(data, frameID, scale)
if scale == 1
  xyz = depth2XYZcamera(data.K, depthRead(data.depth{frameID}));%readDepth(frames, frame);
else
  xyz = depth2XYZcamera(data.K, depthRead(data.depth{frameID}));
  xyz = imresize(xyz, scale, 'nearest');
  xyz(:,:,4) = logical(xyz(:,:,4));
end

function rad = deg2rad(deg)

rad = deg*pi/180;

return;

function [coor,tri] = icosahedron2sphere(level)

% copyright by Jianxiong Xiao http://mit.edu/jxiao
% this function use a icosahedron to sample uniformly on a sphere
%{
Please cite this paper if you use this code in your publication:
J. Xiao, T. Fang, P. Zhao, M. Lhuillier, and L. Quan
Image-based Street-side City Modeling
ACM Transaction on Graphics (TOG), Volume 28, Number 5
Proceedings of ACM SIGGRAPH Asia 2009
%}


a= 2/(1+sqrt(5));
M=[
    0 a -1 a 1 0 -a 1 0
    0 a 1 -a 1 0 a 1 0
    0 a 1 0 -a 1 -1 0 a
    0 a 1 1 0 a 0 -a 1
    0 a -1 0 -a -1 1 0 -a
    0 a -1 -1 0 -a 0 -a -1
    0 -a 1 a -1 0 -a -1 0
    0 -a -1 -a -1 0 a -1 0
    -a 1 0 -1 0 a -1 0 -a
    -a -1 0 -1 0 -a -1 0 a
    a 1 0 1 0 -a 1 0 a
    a -1 0 1 0 a 1 0 -a
    0 a 1 -1 0 a -a 1 0
    0 a 1 a 1 0 1 0 a
    0 a -1 -a 1 0 -1 0 -a
    0 a -1 1 0 -a a 1 0
    0 -a -1 -1 0 -a -a -1 0
    0 -a -1 a -1 0 1 0 -a
    0 -a 1 -a -1 0 -1 0 a
    0 -a 1 1 0 a a -1 0
    ];

coor = reshape(M',3,60)';
%[M(:,[1 2 3]); M(:,[4 5 6]); M(:,[7 8 9])];


[coor, ~, idx] = unique(coor,'rows');

tri = reshape(idx,3,20)';

%{
for i=1:size(tri,1)
    x(1)=coor(tri(i,1),1);
    x(2)=coor(tri(i,2),1);
    x(3)=coor(tri(i,3),1);
    y(1)=coor(tri(i,1),2);
    y(2)=coor(tri(i,2),2);
    y(3)=coor(tri(i,3),2);
    z(1)=coor(tri(i,1),3);
    z(2)=coor(tri(i,2),3);
    z(3)=coor(tri(i,3),3);
    patch(x,y,z,'r');
end

axis equal
axis tight
%}

% extrude
coor = coor ./ repmat(sqrt(sum(coor .* coor,2)),1, 3);

for i=1:level
    m = 0;
    for t=1:size(tri,1)
        n = size(coor,1);
        coor(n+1,:) = ( coor(tri(t,1),:) + coor(tri(t,2),:) ) / 2;
        coor(n+2,:) = ( coor(tri(t,2),:) + coor(tri(t,3),:) ) / 2;
        coor(n+3,:) = ( coor(tri(t,3),:) + coor(tri(t,1),:) ) / 2;
        
        triN(m+1,:) = [n+1     tri(t,1)    n+3];
        triN(m+2,:) = [n+1     tri(t,2)    n+2];
        triN(m+3,:) = [n+2     tri(t,3)    n+3];
        triN(m+4,:) = [n+1     n+2         n+3];
        
        n = n+3;
        
        m = m+4;
        
    end
    tri = triN;
    
    % uniquefy
    [coor, ~, idx] = unique(coor,'rows');
    tri = idx(tri);
    
    % extrude
    coor = coor ./ repmat(sqrt(sum(coor .* coor,2)),1, 3);
end

% vertex number: 12  42  162  642
