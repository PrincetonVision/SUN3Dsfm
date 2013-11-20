function R = coolquat2mat(q)

% COOLQUAT2MAT  Convert Quaternion to rotation matrix as in CoolQuaternion
%               ...

% Author: Andrew Fitzgibbon <awf@robots.ox.ac.uk>
% Date: 23 Mar 99

x2 = q(1) * q(1);
y2 = q(2) * q(2);
z2 = q(3) * q(3);
r2 = q(4) * q(4);

R(1,1) = r2 + x2 - y2 - z2;		% fill diagonal terms
R(2,2) = r2 - x2 + y2 - z2;
R(3,3) = r2 - x2 - y2 + z2;

xy = q(1) * q(2);
yz = q(2) * q(3);
zx = q(3) * q(1);
rx = q(4) * q(1);
ry = q(4) * q(2);
rz = q(4) * q(3);

R(1,2) = 2 * (xy + rz);			% fill off diagonal terms
R(1,3) = 2 * (zx - ry);
R(2,3) = 2 * (yz + rx);
R(2,1) = 2 * (xy - rz);
R(3,1) = 2 * (zx + ry);
R(3,2) = 2 * (yz - rx);
