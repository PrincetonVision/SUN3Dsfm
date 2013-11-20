% Projects the depth values onto the RGB image.
%
% Usage: 
%   imgDepth = imread('raw_clips/bedroom_0001/d-12942.665769-31455701.pgm');
%   imgDepth = swapbytes(imgDepth);
%   rgb = imread('raw_clips/bedroom_0001/r-12941.6324869-2938947.ppm');
%
%   [depthOut, rgbOut] = project_depth_map(imgDepth, rgb);
%   
%
%
% Args:
%   imgDepth - the original uint16 output from the Kinect. The bytes MUST
%              have already been swapped via swapbytes.m
%   rgb - the original uint8 RGB image.
%
% Returns:
%   depthOut - the depth image output in meters (double).
%   rgb_out - the distorted RGB.
function [depthOut, rgbUndistorted] = project_depth_map(imgDepth, rgb)
  camera_params;
  [H, W] = size(imgDepth);
  
  kc_d = [k1_d, k2_d, p1_d, p2_d, k3_d];
  fc_d = [fx_d,fy_d];
  cc_d = [cx_d,cy_d]; 
  
  
  fc_rgb = [fx_rgb,fy_rgb];
  cc_rgb = [cx_rgb,cy_rgb]; 
  kc_rgb = [k1_rgb,k2_rgb,p1_rgb,p2_rgb,k3_rgb];
  
  rgbUndistorted = zeros(size(rgb));
    
  for ii = 1 : size(rgb,3)
    rgbUndistorted(:,:,ii) = undistort(double(rgb(:,:,ii)), ...
        fc_rgb, cc_rgb, kc_rgb, 0);
  end
  
  rgbUndistorted = uint8(rgbUndistorted);

  noiseMask = 255 * double(imgDepth == max(imgDepth(:)));

  % Undistort the noise mask.
  noiseMask = undistort(noiseMask, fc_d, cc_d, kc_d, 0);
  noiseMask = noiseMask > 0;

  imgDepth = undistort(double(imgDepth),fc_d,cc_d,kc_d,0);

  % Fix issues introduced by distortion.
  imgDepth(imgDepth < 600) = 2047;
  imgDepth(noiseMask) = 2047;

  depth2 = depth_rel2depth_abs(imgDepth);
  points3d = depth_plane2depth_world(depth2);
  points3d = depth_world2rgb_world(points3d);
  
  [xProj, yProj] = rgb_world2rgb_plane(points3d);

  % Finally, project back onto the RGB plane.
  xProj = round(xProj);
  yProj = round(yProj);
  
  goodInds = find(xProj(:) > 0 &  xProj(:) < W & ...
                  yProj(:) > 0 &  yProj(:) < H);
  
  depthOut = zeros(size(imgDepth));
  [depthSorted, order] = sort(-depth2(goodInds));
  depthSorted = -depthSorted;
  
  % Z-buffer projection
  for ii = 1:length(order)
    depthOut(yProj(goodInds(order(ii))), xProj(goodInds(order(ii)))) = ...
        depthSorted(ii);
  end
  
  % Fix weird values...
  depthOut(depthOut > maxDepth) = maxDepth;
  depthOut(depthOut < 0) = 0;
  depthOut(isnan(depthOut)) = 0;
end