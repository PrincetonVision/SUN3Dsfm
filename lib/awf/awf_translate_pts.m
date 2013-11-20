function xpt = awf_translate_pts(x, t)

% AWF_TRANSLATE_PTS x(i,:) = x(i,:) + t

% Author: Andrew Fitzgibbon <awf@robots.ox.ac.uk>
% Date: 31 Aug 01

xpt = x + t(ones(size(x,1),1),:);
