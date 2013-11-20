function cim = nonmaxsup(m, radius)
  if (nargin == 1) radius = 1; end
  % Extract local maxima by performing a grey scale morphological
  % dilation and then finding points in the corner strength image that
  % match the dilated image and are also greater than the threshold.
  sze = 2 * radius + 1; % Size of mask.
  
  %areaM = ones(sze);
  
  [rr cc] = meshgrid(1:sze);  
  areaM = sqrt((rr-(radius+1)).^2+(cc-(radius+1)).^2)<=sze/2;  
  
  mx = ordfilt2(m, sum(sum(areaM)), areaM); % Grey-scale dilate.
  
  selectMap = (m == mx);
  
  selectMap = ordfilt2(selectMap, 9, ones(3,3));
  
  
  % grow neighborhood of max threshold;
  
  cim = sparse(m .* selectMap);
  