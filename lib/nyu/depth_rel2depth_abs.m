% Projects the given depth image to world coordinates. Note that this 3D
% coordinate space is defined by a horizontal plane made from the X and Z
% axes and the Y axis points up.
%
% Args:
%   imgDepthOrig - 480x640 raw depth image from the Kinect. Note that the
%                  bytes of the original uint16 image must have already
%                  been swapped via swapbytes.m
%
% Returns:
%   imgDepthAbs - 480x640 depth image whose values are in meters, rather
%                 than the internal depth values used by the kinect.
function imgDepthAbs = depth_rel2depth_abs(imgDepthOrig)
  assert(isa(imgDepthOrig, 'double'));

  [H, W] = size(imgDepthOrig);
  assert(H == 480);
  assert(W == 640);

  camera_params;

  imgDepthAbs = depthParam1 ./ (depthParam2 - imgDepthOrig);
  
  imgDepthAbs(imgDepthAbs > maxDepth) = maxDepth;
  imgDepthAbs(imgDepthAbs < 0) = 0;
end
