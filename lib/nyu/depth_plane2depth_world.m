% Projects the given depth image to world coordinates. Note that this 3D
% coordinate space is defined by a horizontal plane made from the X and Z
% axes and the Y axis points up.
%
% Args:
%   imgDepthAbs - 480x640 depth image whose values indicate depth in
%                 meters.
%
% Returns:
%   points3d - Nx3 matrix of 3D world points (X,Y,Z).
function points3d = depth_plane2depth_world(imgDepthAbs)
  [H, W] = size(imgDepthAbs);
  assert(H == 480);
  assert(W == 640);

  camera_params;

  [xx,yy] = meshgrid(1:W, 1:H);
  
  X = (xx - cx_d) .* imgDepthAbs / fx_d;
  Y = (yy - cy_d) .* imgDepthAbs / fy_d;
  Z = imgDepthAbs;
  
  points3d = [X(:) Y(:) Z(:)];
end
