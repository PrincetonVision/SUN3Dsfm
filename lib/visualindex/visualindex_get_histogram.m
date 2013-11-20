function v = visualindex_get_histogram(model, words)
% VISUALINDEX_GET_HISTOGRAM  Get visual word histogram
%   V = VISUALINDEX_GET_HISTOGRAM(MODEL, WORDS) returns the histogram
%   of visual words V given the visual words WORDS.

% Author: Andrea Vedaldi

v = sparse(double(words),1,...
           ones(length(words),1), ...
           model.vocab.size,1) ;
v = v .*  model.vocab.weights ;
v = v / norm(v) ;
