function [f,d] = visualindex_get_features(model, im)
% VISUALINDEX_GET_FEATURES  Extract features from an image
%   [F,D] = VISUALINDEX_GET_FEATURES(MODEL, IM) extracts the SIFT
%   frames F and descriptors D from image IM for indexing based on the
%   specified MODEL.

% Auhtor: Andrea Vedaldi

area = size(im,1) * size(im,2) ;
firstOctave = max(ceil(.5 * log2(area / 1024^2)), -1) ;
[f,d] = vl_sift(im2single(rgb2gray(im)), ...
                'firstoctave', firstOctave,  ...
                'floatdescriptors') ;
