% Returns the accelerometer data coming from the kinect.
%
% Args:
%   filename - the filename of the .dump file.
%
% Returns:
%   data - a 4x1 vector where the first three elements are the x, y, and z
%          components of the accelerometer and the fourth element is the
%          tilt angle.
%
% data = get_accel_data(filename);