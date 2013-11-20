% Performs the camera projection from the RGB-world coordinate frame onto
% the RGB plane.
%
% Args:
%   points3d - Nx3 matrix of (X,Y,Z) points in the RGB-world coordinate
%              frame.
% 
% Returns:
%   X_plane - the X coordinates in the RGB plane.
%   Y_plane - the Y coordiantes in the RGB plane.
function [X_plane, Y_plane] = rgb_world2rgb_plane(points3d)
  camera_params;

  X_world = points3d(:,1);
  Y_world = points3d(:,2);
  Z_world = points3d(:,3);
  
  X_plane = (X_world .* fx_rgb ./ Z_world) + cx_rgb;
  Y_plane = (Y_world .* fy_rgb ./ Z_world) + cy_rgb;
end