function [ids, scores, matches] = visualindex_query(model, im, runGeometric)
% VISUALINDEX_QUERY  Search index for matching images
%   [IDS, SCORES, MATCHES] = VISUALINDEX_QUERY(MODEL, IM) searches the
%   index MODEL for images matching the query image IM. It returns
%   a list of imge IDS and SCORES by descending confidence.
%
%   The function first matches images based on the visual words
%   histograms and then geometrically verifies the top
%   MODEL.RERANKDEPTH results, reranking those.  The function also
%   returns MATCHES, an array of structures with the geometric
%   verification `certificates' for these top matches.

% Author: Andrea Vedaldi

if ~exist('runGeometric','var')
    runGeometric = false;
end

% reranking depth cannot be larger than the number of indexed images
depth = min(model.rerankDepth, numel(model.index.ids)) ;

% extract the features, visual words, and histogram for the query images
[frames, descrs] = visualindex_get_features(model, im) ;
words = visualindex_get_words(model, descrs) ;
histogram = visualindex_get_histogram(model, words) ;

% compute histogram-based score
scores = histogram' * model.index.histograms ;

% apply geometric verification to the top matches
[scores, perm] = sort(scores, 'descend') ;
ids = model.index.ids(perm) ;

if runGeometric
    for t = 1:depth
      words0 = model.index.words{perm(t)} ;
      frames0 = model.index.frames{perm(t)} ;
      [scores(t), matches{t}] = verify(frames0, words0, frames, words, size(im)) ;
    end
    [scores(1:depth), reperm] = sort(scores(1:depth), 'descend') ;
    matches = matches(reperm) ;
    perm(1:depth) = perm(reperm) ;
    ids = model.index.ids(perm) ;
end

% --------------------------------------------------------------------
function [score, matches] = verify(f1,w1,f2,w2,s2)
% --------------------------------------------------------------------
% The geometric verfication is a simple RANSAC affine matcher. It can
% be significantly improved.

% find the features that are mapped to the same visual words
[drop,m1,m2] = intersect(w1,w2) ;
numMatches = length(drop) ;

% get the 2D coordinates of these features in homogeneous notation
X1 = f1(1:2, m1) ;
X2 = f2(1:2, m2) ;
X1(3,:) = 1 ;
X2(3,:) = 1 ;

thresh = max(max(s2)*0.02, 10) ;

% RANSAC
randn('state',0) ;
rand('state',0) ;
numRansacIterations = 500 ;
for t = 1:numRansacIterations
  fprintf('RANSAC iteration %d of %d\r', t, numRansacIterations) ;
  % select a subset of 3 matched features at random
  subset = vl_colsubset(1:numMatches, 3) ;

  % subtract the first one from the other two and then compute affine
  % transformation (note the small regularization term).
  u1 = X1(1:2,subset(3)) ;
  u2 = X2(1:2,subset(3)) ;
  A{t} = (X2(1:2,subset(1:2)) - [u2 u2]) / ...
         (X1(1:2,subset(1:2)) - [u1 u1] + eye(2)*1e-5) ;
  T{t} = u2 - A{t} * u1 ;

  % the score is the number of inliers
  X2_ = [A{t} T{t} ; 0 0 1] * X1 ;
  delta = X2_ - X2 ;
  ok{t} = sum(delta.*delta,1) < thresh^2 ;
  score(t) = sum(ok{t}) ;
end

[score, best] = max(score) ;
matches.A = A{best} ;
matches.ok = ok{best} ;
matches.f1 = f1(:, m1(matches.ok)) ;
matches.f2 = f2(:, m2(matches.ok)) ;
