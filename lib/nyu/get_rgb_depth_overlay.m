% Returns an overlay of RGB and Depth frames to evaluate the alignment.
%
% Args:
%   imgRgb - the RGB image, an HxWx3 matrix of type uint8.
%   imgDepthAbs - the absolute-depth image, an HxW matrix of type double
%                 whose values indicate depth in meters.
%
% Returns:
%   imgOverlay - an image visualizing RGB and Depth alignment.
function imgOverlay = get_rgb_depth_overlay(imgRgb, imgDepthAbs)
  % Check dims.
  assert(ndims(imgRgb) == 3);
  assert(ndims(imgDepthAbs) == 2);

  % Check sizes.
  [H, W, D] = size(imgRgb);
  assert(D == 3);
  assert(all(size(imgDepthAbs) == [H, W]));
  
  % Check types.
  assert(isa(imgRgb, 'uint8'));
  assert(isa(imgDepthAbs, 'double'));
  
  imgDepth = imgDepthAbs - min(imgDepthAbs(:));
  imgDepth = imgDepth ./ max(imgDepth(:));
  imgDepth = uint8(imgDepth * 255);
  
  imgOverlay = reshape(imgRgb, [H*W 3]);
  imgOverlay(:,3) = imgOverlay(:,2);
  imgOverlay(:,2) = imgDepth(:);
  
  imgOverlay = reshape(imgOverlay, [H, W, 3]);
end