function [words,dists] = visualindex_get_words(model, descrs)
% VISUALINDEX_GET_WORDS  Convert visual descriptors into visual words
%   [WORDS, DISTS] = VISUALINDEX_GET_WORDS(MODEL, DESCRS) quantizes
%   the descriptors DESCRS and returns the corresponding WORDS, along
%   with the distances DISTS from each descriptor to the corresponding
%   visual word center.

% Author: Andrea Vedaldi

[words,dists] = vl_kdtreequery(model.vocab.tree, ...
                               model.vocab.centers, ...
                               descrs, ...
                               'maxcomparisons', 500) ;
