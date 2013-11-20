function rotationMatrix = rotationMatrixFromPointCloud(ptCloud)


if size(ptCloud,2)<3
    rotationMatrix = eye(3);
    return;
end

rotationMatrix = princomp(ptCloud');

if abs(det(rotationMatrix)-1)>0.001 || sum(sum(abs(rotationMatrix'-inv(rotationMatrix))))>0.001
    rotationMatrix(:,3) = -rotationMatrix(:,3);
    if abs(det(rotationMatrix)-1)>0.001 || sum(sum(abs(rotationMatrix'-inv(rotationMatrix))))>0.001
        error('Rotation matrix is not rotation');
    end
end
