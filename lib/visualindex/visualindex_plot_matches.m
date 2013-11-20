function visualindex_plot_matches(model, matches, im1, im2, sz1, sz2)

h1 = size(im1,1) ; w1 = size(im1,2) ;
h2 = size(im2,1) ; w2 = size(im2,2) ;

if ~exist('sz1','var'), sz1 = [w1;h1] ; end
if ~exist('sz2','var'), sz2 = [w2;h2] ; end
s1 = w1 / sz1(1) ;
s2 = w2 / sz2(1) ;

if size(im1,3) == 1, im1 = cat(3,im1,im1,im1) ; end
if size(im2,3) == 1, im2 = cat(3,im2,im2,im2) ; end

im3 = zeros(max(h1,h2),w1+w2,3,'uint8') ;
im3(1:h1,1:w1,:) = im1 ;
im3(1:h2,w1+(1:w2),:) = im2 ;

S1 = diag([s1 s1 s1 1]) ;
S2 = diag([s2 s2 s2 1]) ;

f1 = S1 * matches.f1 ;
f2 = S2 * matches.f2 ;
f2(1,:) = f2(1,:) + w1 ;

cla ;
imagesc(im3) ;
axis off image ;  hold on ;
vl_plotframe([f1 f2]) ;
line([f1(1,:) ; f2(1,:)], ...
     [f1(2,:) ; f2(2,:)], 'color', 'r', 'marker', '.') ;
title(sprintf('inliers: %d', size(matches.f1,2))) ;

