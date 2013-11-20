function model = visualindex_build(images, ids, runGeometric, varargin)
% VISUALINDEX_BUILD  Build an index for a set of images
%   MODEL = VISUALDINDEX_BUILD(IMAGES, IDS) indexes the specified
%   IMAGES associating to them the given IDS. IMAGES is a cell array
%   of strings containitn path to the images and IDS are unique
%   numeric identifiers (in DOUBLE class).
%
%   VISUALINDEX_BUILD(..., 'numWords', K, 'numKMeansIterations', T)
%   allows specifying the number of visual words and K-means
%   iterations used to build the visual words vocabulary.
%
%   MODEL is a structure with the following fields
%
%   model.vocab.size:
%      Size of the visual vocabulary (number of visual wrods).
%
%   model.vocab.centers:
%      Visual words (quantized SIFT features).
%
%   model.vocab.tree:
%      KD-Tree index of the visual words (see VL_KDTREEBUILD()).
%
%   model.vocab.weights:
%      TF-IDF weights of the visual words.
%
%   model.index.frames
%   model.index.descrs
%   model.index.words
%      Cell array of frames (keypoints), descriptors, and visual words
%      of the SIFT features of each image.
%
%   model.index.histograms
%      Cell array of histogram of visual words for each image.

% Author: Andrea Vedaldi

opts.numWords = 10000 ;
opts.numKMeansIterations = 20 ;
opts = vl_argparse(opts, varargin) ;

randn('state',0) ;
rand('state',0) ;

model.rerankDepth = 40 ;
model.vocab.size = opts.numWords ;
model.index = struct ;

% --------------------------------------------------------------------
%                                              Extract visual features
% --------------------------------------------------------------------
% Extract SIFT features from each image.

% read features
frames = cell(1,numel(images)) ;
descrs = cell(1,numel(images)) ;
parfor i = 1:length(images)
  fprintf('Adding image %s (%d of %d)\n', ...
          images{i}, i, numel(images)) ;
  im = imread(images{i}) ;
  [frames{i},descrs{i}] = visualindex_get_features(model, im) ;
end
model.index.frames = frames ;
model.index.descrs = descrs ;
model.index.ids = ids ;
clear frames descrs ids ;

% --------------------------------------------------------------------
%                                                  Large scale k-means
% --------------------------------------------------------------------
% Quantize the SIFT features to obtain a visual word vocabulary.
% Implement a fast approximate version of K-means by using KD-Trees
% for quantization.

E = [] ;
assign = []  ;
descrs = vl_colsubset(cat(2,model.index.descrs{:}), opts.numWords * 15) ;
dist = inf(1, size(descrs,2)) ;

[model.vocab.centers, model.vocab.tree] = ...
    annkmeans(descrs, opts.numWords, 'verbose', true) ;

% --------------------------------------------------------------------
%                                                           Histograms
% --------------------------------------------------------------------
% Compute a visual word histogram for each image, compute TF-IDF
% weights, and then reweight the histograms.

words = cell(1, numel(model.index.ids)) ;
histograms = cell(1,numel(model.index.ids)) ;
for t = 1:length(model.index.ids)
  words{t} = visualindex_get_words(model, model.index.descrs{t}) ;
  histograms{t} = sparse(double(words{t}),1,...
                         ones(length(words{t}),1), ...
                         model.vocab.size,1) ;
end
model.index.words = words ;
model.index.histograms = cat(2, histograms{:}) ;
clear words histograms ;

% compute IDF weights
model.vocab.weights = log((size(model.index.histograms,2)+1) ...
                          ./  (max(sum(model.index.histograms > 0,2),eps))) ;

% weight and normalize histograms
for t = 1:length(model.index.ids)
  h = model.index.histograms(:,t) .*  model.vocab.weights ;
  model.index.histograms(:,t) = h / norm(h) ;
end

% release memory
if runGeometric
    model.index.frames = [];
    model.index.descrs = [];
end
