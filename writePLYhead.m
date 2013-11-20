function file = writePLYhead(PLYfilename, pointCount)

file = fopen(PLYfilename,'w');
fprintf (file, 'ply\n');
%fprintf (file, 'format ascii 1.0\n');
fprintf (file, 'format binary_little_endian 1.0\n');
fprintf (file, 'element vertex %d\n', pointCount);
fprintf (file, 'property float x\n');
fprintf (file, 'property float y\n');
fprintf (file, 'property float z\n');
fprintf (file, 'property uchar red\n');
fprintf (file, 'property uchar green\n');
fprintf (file, 'property uchar blue\n');
fprintf (file, 'end_header\n');