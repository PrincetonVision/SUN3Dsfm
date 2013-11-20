% Performs the affine transformation between the Depth-world coordinate
% frame and the RGB-world coordinate frame.
%
% Args:
%   points3d - the 3D points in the depth camera's world coordinate frame,
%              an Nx3 matrix where N=480*640.
%
% Returns:
%   points3d - the 3D points in the RGB camera's world coordinate frame,
%              an Nx3 matrix where N=480*640.
function points3d = depth_world2rgb_world(points3d)
  camera_params;
  
  T = [t_x; t_z; t_y];
  points3d = R * points3d' + T * ones(1, size(points3d,1));
  points3d = points3d';
end