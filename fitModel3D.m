function objectRTO2W = fitModel3D(ptCloud, label)

global objectLabel;

if size(ptCloud,2)<3
    objectRTO2W = [eye(3) zeros(3,1)];
    return;
end

switch objectLabel.objectType(label)
    case 0 % a floor
        objectRTO2W = [eye(3) zeros(3,1)];
    case 1 % an axis y align object, eg. a ceiling, or a wall
        rotationMatrix = princomp(ptCloud([1 3],:)');
        
        if abs(det(rotationMatrix)-1)>0.000001 || sum(sum(abs(rotationMatrix'-inv(rotationMatrix))))>0.000001
            rotationMatrix(:,2) = -rotationMatrix(:,2);
            if abs(det(rotationMatrix)-1)>0.000001 || sum(sum(abs(rotationMatrix'-inv(rotationMatrix))))>0.000001
                error('Rotation matrix is not rotation');
            end
        end
       
        objectRTO2W = [[rotationMatrix(1,1),0,rotationMatrix(1,2); 0,1,0; rotationMatrix(2,1),0,rotationMatrix(2,2)] median(ptCloud,2)];
    case 2 % a general object
        objectRTO2W = [rotationMatrixFromPointCloud(ptCloud) median(ptCloud,2)];
    case 3 % an unwanted object, such as mirror
        objectRTO2W = [eye(3) zeros(3,1)];
    case 4 % no rotation, only translation
        objectRTO2W = [eye(3) median(ptCloud,2)];
end
