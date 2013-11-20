function [ids, scores] = BOWquery(frames, frameID, model)

im = readImage(frames,frameID);
[ids, scores] = visualindex_query(model, im) ;

%{
%% visualization
figure(1) ; clf ;
imagesc(im) ; title('query') ;
axis image off ; drawnow ;

figure(2) ; clf ;
for k = 1:min(6, length(ids))
    vl_tightsubplot(6,k,'box','outer') ;
    imagesc(readImage(frames,ids(k)));
    axis image off ;
    title(sprintf('rank:%d score:%g id:%d', k, full(scores(k)), ids(k))) ;
end
%}

%sz = [size(im,2); size(im,1)] ;
%figure(4) ; clf ;
%visualindex_plot_matches(model, matches{1}, thumb_{1}, im, sz_{1}, sz) ;
