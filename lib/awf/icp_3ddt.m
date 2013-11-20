function D = icp_3ddt(M)

% ICP_3DDT      Compute 3D distance transform from Mask array M
% Author: Andrew Fitzgibbon <awf@robots.ox.ac.uk>
% Date: 30 Aug 01

% Changed by Jianxiong Xiao Mar 31 2013
% compile by: g++ -o icp_3ddt icp_3ddt.cxx

N = size(M,1);

tmpfile = tempname;

infile = [tmpfile '_in.dat'];
outfile = [tmpfile '_3ddt.dat'];

disp('3ddt distance tranform')

f = fopen(infile, 'wb');
fwrite(f, M(:), 'float32');
fclose(f); 

tic
path = [fileparts(which('icp_3ddt.m')) filesep];
unix(sprintf('%sicp_3ddt %d %s %s ', path, N, infile, outfile))
toc

f = fopen(outfile, 'rb'); 
D = fread(f, N^3, 'float32'); 
fclose(f); 
D = reshape(D,  N,N,N);

delete(infile);
delete(outfile);
