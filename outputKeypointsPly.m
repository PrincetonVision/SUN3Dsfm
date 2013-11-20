function outputKeypointsPly(PLYfilename,pointCloud)


count = sum(~isnan(pointCloud(1,:)));

%fprintf('Writing ply point cloud file.\n');
file = fopen(PLYfilename,'w');
fprintf (file, 'ply\n');
%fprintf (file, 'format ascii 1.0\n');
fprintf (file, 'format binary_little_endian 1.0\n');
fprintf (file, 'element vertex %d\n', count);
fprintf (file, 'property float x\n');
fprintf (file, 'property float y\n');
fprintf (file, 'property float z\n');
fprintf (file, 'end_header\n');

for i=find(~isnan(pointCloud(1,:)))
        fwrite(file, pointCloud(1,i),'float');
        fwrite(file, pointCloud(2,i),'float');
        fwrite(file, pointCloud(3,i),'float');
end
fclose(file);
