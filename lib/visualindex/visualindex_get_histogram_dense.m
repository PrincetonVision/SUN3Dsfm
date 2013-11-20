function v = visualindex_get_histogram_dense(model, words)
% VISUALINDEX_GET_HISTOGRAM  Get visual word histogram
%   V = VISUALINDEX_GET_HISTOGRAM(MODEL, WORDS) returns the histogram
%   of visual words V given the visual words WORDS.

% Author: Andrea Vedaldi


v = hist(words,1:model.vocab.size); 
v = v .*  model.vocab.weights';
v = v / norm(v) ;
