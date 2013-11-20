function [scores] = visualindex_query_fast(model, im)


% extract the features, visual words, and histogram for the query images
[~, descrs] = visualindex_get_features(model, im) ;
words = visualindex_get_words(model, descrs) ;
histogram = visualindex_get_histogram_dense(model, words) ;

% compute histogram-based score
scores = histogram * model.index.histograms ;
