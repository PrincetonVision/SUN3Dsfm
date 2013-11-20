function Rtparams = run_icp3d(Model, Data, ROBUST_THRESH, initial_p)

% RUN_ICP3D     A function
%               ...

% Author: Andrew Fitzgibbon <awf@robots.ox.ac.uk>
% Date: 31 Aug 01

global run_icp3d_iter

if nargin < 3
    ROBUST_THRESH = 1000; % i.e. non-robust
end

N = 100;
icp.ROBUST_THRESH = ROBUST_THRESH;

% Center model
ModelCenter = mean(Model);
Model = awf_translate_pts(Model, -ModelCenter);

% Center data
DataCenter = mean(Data);
Data = awf_translate_pts(Data, -DataCenter);

bbox_min = min(min(Model), min(Data));
bbox_max = max(max(Model), max(Data));

maxdiff = max(bbox_max - bbox_min);


% Map to integer grid, but keep EDGE_DIST pixels off the edge
% Map min -> 3
% Map max -> N - 3
EDGE_DIST = 10;

% x = (x - bbox_min) * (N-2*EDGE_DIST)/maxdiff + EDGE_DIST
%   = x * (N-2*EDGE_DIST)/maxdiff - bbox_min * (N-2*EDGE_DIST)/maxdiff + EDGE_DIST
scale = (N-2*EDGE_DIST)/maxdiff;
t = EDGE_DIST - bbox_min * scale;

Data = awf_translate_pts(Data * scale, t);
Model = awf_translate_pts(Model * scale, t);

bbox_min = min(min(Model), min(Data));
bbox_max = max(max(Model), max(Data));

clf
BOX = [
  0 0 0;
  0 1 0;
  1 1 0;
  1 0 0;
  0 0 0;
  0 0 1;
  0 1 1; 0 1 0; 0 1 1;
  1 1 1; 1 1 0; 1 1 1;
  1 0 1; 1 0 0; 1 0 1;
  0 0 1;
  ];

scatter(BOX * (N+1), 'b-')
hold on
set(scatter(Model(:,[2 1 3]), 'r.'), 'markersize', 0.5)
camlight
hold on
h = scatter(Data(:,[2 1 3]), '.');
if size(Data,1) > 1000
  disp('small markers');
  set(h, 'markersize', 0.1);
end
view(3)
axis equal
drawnow


% Make model DT
B = round(Model);
M = zeros(N,N,N);
M(sub2ind(size(M), B(:,1), B(:,2), B(:,3))) = 1;
D = icp_3ddt(M);

% Compute derivatives of D
dDdX = convn(D, [-1,0,1]/2, 'same');
dDdY = convn(D, [-1,0,1]'/2, 'same');
dDdZ = convn(D, cat(3, -1, 0, 1)/2, 'same');

%ISO_SURF_DIST = 10;
%p = patch(isosurface(D, ISO_SURF_DIST), 'facecolor', 'none', 'edgecolor', 'red');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
%% Run minimization
%%

icp.Data = Data;
icp.ModelDistanceTransform = D;
icp.ModelDx = dDdX;
icp.ModelDy = dDdY;
icp.ModelDz = dDdZ;
icp.handle = h;

DO_LM = 1;
if DO_LM
  run_icp3d_iter = 1;
  options = optimset('lsqnonlin');
  options.TypicalX = [1 1 1 1 1 1 1];
  options.TolFun = 0.0001;
  options.TolX = 0.00001;
  options.DiffMinChange = .001;
  options.LargeScale = 'on';
  options.maxFunEvals = 1000;
  options.Jacobian = 'on';
  options.DerivativeCheck = 'off';
  params = [0 0 0 1 0 0 0]; % quat, tx, ty, tz
  if nargin > 3
    params = initial_p;
  end
  
  % test_derivatives([0 0 0 .5 0 0 0], icp);
  % test_derivatives([.1 .1 .1 .9 .01 .01 .01], icp);
  % test  with some outside:
  % test_derivatives([.1 .1 .1 .9 .3 .01 .01], icp);
  
  [params, resnorm, residual, exitflag, output] =...
      lsqnonlin(@icp_error_with_derivs, params, [], [], options, icp);
else
  do_icp(icp)
end

[R,t] = icp_deparam(params);
Rtparams = [R t];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function test_derivatives(p, icp)
% Test derivatives
delta = .0001;

disp('Computing FD derivatives');
[err0, J0] = icp_error_with_derivs(p, icp);
for k=1:length(p)
  p0 = p; p0(k) = p0(k) - delta;
  p1 = p; p1(k) = p1(k) + delta;
  h = p1(k) - p0(k);
  f0 = icp_error_with_derivs(p0, icp);
  f1 = icp_error_with_derivs(p1, icp);
  FDJac(:,k) = (f1 - f0) / h;
end
J0
FDJac

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [dists, J] = icp_error_with_derivs(params, icp)
% In:
% icp.Data                    Nx7 data points
% icp.ModelDistanceTransform  DT of model points
%
% Out:
%   dists = Nx1
%       J = Nx7
global run_icp3d_iter

%% Derivations:
%
%          TX := T(params, icp.Data(i))
% dists(i) = DT(TX);
%  J(i, j) = d[ DT(TX) ]/d[params(j)]
%          = ddX[ DT ](TX) * d[TX]/d[params(j)]
%
% [TX, Jx, Jy, Jz] = icp_3d_err_transformed(params, icp.Data);
%      Jx = N x 7
%  T(i,j) = Jx .* Dx(TX) + Jy .* Dy(TX) +  Jz .* Dz(TX);

%% Code:
%
fprintf('Iter %3d ', run_icp3d_iter);
run_icp3d_iter = run_icp3d_iter + 1;

fprintf('%5.2f ', params);

PARAMSCALE = [1 1 1 1 100 100 100];

% SCALE up
params = params .* PARAMSCALE;

[R,t] = icp_deparam(params);

% Compute transformed data points and Jacobians
[Tdata, Jx, Jy, Jz] = icp_3d_err_transformed(params, icp.Data);
fprintf('trans ');

if 1
  set(icp.handle, ...
      'xdata', Tdata(:,2), ...
      'ydata', Tdata(:,1), ...
      'zdata', Tdata(:,3));
  drawnow
  fprintf('draw ');
end

% Index each row of Tdata into the DT.  It's oddly ordered...
i = (Tdata(:,2));
j = (Tdata(:,1));
k = (Tdata(:,3));

N = size(icp.ModelDistanceTransform, 1);

distpenalty = zeros(size(Tdata,1), 1);
Grad_distpenalty = zeros(size(Tdata,1), 3);

LO = 3; % .. don't use derivatives near the boundary
HI = N-3;

% i < 1 .. clip and add pythagorean penalty
I = find(i < LO);
if ~isempty(I)
  distpenalty(I) = distpenalty(I) + (LO - i(I)).^2;
  Grad_distpenalty(I, 2) = Grad_distpenalty(I,2) + 2 * (LO - i(I));
  i(I) = LO;
end

% j < LO
I = find(j < LO);
if ~isempty(I)
  distpenalty(I) = distpenalty(I) + (LO - j(I)).^2;
  Grad_distpenalty(I, 1) = Grad_distpenalty(I,1) + 2 * (LO - j(I));
  j(I) = LO;
end

% k < LO
I = find(k < LO);
if ~isempty(I)
  distpenalty(I) = distpenalty(I) + (LO - k(I)).^2;
  Grad_distpenalty(I, 3) = Grad_distpenalty(I,3) + 2 * (LO - k(I));
  k(I) = LO;
end

% i >= HI .. clip and add pythagorean penalty
I = find(i >= HI);
if ~isempty(I)
  distpenalty(I) = distpenalty(I) + (i(I) - HI).^2;
  Grad_distpenalty(I, 2) = Grad_distpenalty(I,2) + 2 * (i(I) - HI);
  i(I) = HI;
end

% j >= HI
I = find(j >= HI);
if ~isempty(I)
  distpenalty(I) = distpenalty(I) + (j(I) - HI).^2;
  Grad_distpenalty(I, 1) = Grad_distpenalty(I,1) + 2 * (j(I) - HI);
  j(I) = HI;
end

% k >= HI
I = find(k >= HI);
if ~isempty(I)
  distpenalty(I) = distpenalty(I) + (k(I) - HI).^2;
  Grad_distpenalty(I, 3) = Grad_distpenalty(I,3) + 2 * (k(I) - HI);
  k(I) = HI;
end

fprintf(' n=%d ', length(find(distpenalty)))
% Now all in range.  Dists^2 = dist at closest 
% pt on DT boundary^2 + pythagpenalty^2

fprintf('interp ');
dists = interp3(icp.ModelDistanceTransform, i, j, k, 'linear');
ddists_dx = interp3(icp.ModelDy, i, j, k, 'linear');% x,y swapped
ddists_dy = interp3(icp.ModelDx, i, j, k, 'linear');
ddists_dz = interp3(icp.ModelDz, i, j, k, 'linear');
Grad_dists = [ddists_dx ddists_dy ddists_dz];

% Add penalties to distances, so points outside the DT get
% an upper-bound distance to the model.
dists = dists + 1e-4;
pdists = sqrt(dists.^2 + distpenalty);
c = 1./(2*pdists);
dpdists_dx = c .* (2*dists.* ddists_dx + Grad_distpenalty(:,1));
dpdists_dy = c .* (2*dists.* ddists_dy + Grad_distpenalty(:,2));
dpdists_dz = c .* (2*dists.* ddists_dz + Grad_distpenalty(:,3));
Grad_pdists = [dpdists_dx dpdists_dy dpdists_dz];

%% Robustness... Just a clipped quadratic here, a Lorentzian would be
%% better, but I haven't worked out the derivatives.
ROBUST_THRESH = icp.ROBUST_THRESH;
fprintf(' #outliers=%d ', sum(dists > ROBUST_THRESH));
dists(dists > ROBUST_THRESH) = ROBUST_THRESH;
Grad_pdists(dists > ROBUST_THRESH,:) = 0;

% Grad_pdists - [ddists_dx ddists_dy ddists_dz]

dists = dists / 100;
Grad_pdists = Grad_pdists / 100;

% d/dp f(T(p)) = [d/dx f(x)](T(p)) d/dp T(p)
%    1 x 7             1 x 3       3 x 7

L = ones(1,7);
J = Grad_pdists(:, L*1) .* Jx + ...
    Grad_pdists(:, L*2) .* Jy + ...
    Grad_pdists(:, L*3) .* Jz;

% Scale down J
J = J .* PARAMSCALE(ones(size(J,1), 1),:);

% Correct sign, don't know where I dropped it...
% (Prob in the convolutions to get Dx I guess)
J = -J;

fprintf('err %g\n', norm(dists));

% keyboard




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function dists = icp_error(params, icp)
% In:
% icp.Data                    Nx7 data points
% icp.ModelDistanceTransform  DT of model points
global run_icp3d_iter

fprintf('Iter %3d ', run_icp3d_iter);
run_icp3d_iter = run_icp3d_iter + 1;

[R,t] = icp_deparam(params);
fprintf('%6.3f ', params);

% Compute transformed data points and Jacobians
% [Tdata, Jx, Jy, Jz] = icp_3d_err_transformed(params, icp.Data);
Tdata = icp_3d_err_transformed(params, icp.Data);
fprintf('trans ');

if 1
  set(icp.handle, ...
      'xdata', Tdata(:,2), ...
      'ydata', Tdata(:,1), ...
      'zdata', Tdata(:,3));
  drawnow
  fprintf('draw ');
end

% Index each row of Tdata into the DT.  It's oddly ordered...
i = round(Tdata(:,2));
j = round(Tdata(:,1));
k = round(Tdata(:,3));

N = size(icp.ModelDistanceTransform, 1);

distpenalty = 0*i;

% i < 1 .. clip and add pythagorean penalty
I = find(i < 1);
distpenalty(I) = distpenalty(I) + (1 - i(I)).^2;
i(I) = 1;

% j < 1
I = find(j < 1);
distpenalty(I) = distpenalty(I) + (1 - j(I)).^2;
j(I) = 1;

% k < 1
I = find(k < 1);
distpenalty(I) = distpenalty(I) + (1 - k(I)).^2;
k(I) = 1;

% i >= N .. clip and add pythagorean penalty
I = find(i >= N);
distpenalty(I) = distpenalty(I) + (i(I) - N).^2;
i(I) = N;

% j >= N
I = find(j >= N);
distpenalty(I) = distpenalty(I) + (j(I) - N).^2;
j(I) = N;

% k >= N
I = find(k >= N);
distpenalty(I) = distpenalty(I) + (k(I) - N).^2;
k(I) = N;

fprintf(' n %d ', length(find(distpenalty)))
% Now all in range.  Dists^2 = dist at closest 
% pt on DT boundary^2 + pythagpenalty^2

fprintf('interp ');
dists = interp3(icp.ModelDistanceTransform, i, j, k, 'linear');

% Add penalties to distances, so points outside the DT get
% an upper-bound distance to the model.
dists = sqrt(dists.^2 + distpenalty);

%% Robustness..
ROBUST_THRESH = 1000;
fprintf(' out %d ', sum(dists > ROBUST_THRESH));
dists(dists > ROBUST_THRESH) = ROBUST_THRESH;

dists = dists / 1000;

fprintf('err %g\n', norm(dists));

% keyboard
