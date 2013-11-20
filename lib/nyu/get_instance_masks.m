% Returns a series of masks for each object instance in the given scene.
%
% Args:
%   imgObjectLabels - HxW label map. 0 indicates a missing label.
%   imgInstances - HxW instance map.
%
% Returns:
%   instanceMasks - binary masks of size HxWxN where N is the number of
%                   total objects in the room.
%   instanceLabels - Nx1 vector of class labels for each instance mask.
function [instanceMasks, instanceLabels] = get_instance_masks(...
    imgObjectLabels, imgInstances)
  
  [H, W] = size(imgObjectLabels);  

  pairs = unique([imgObjectLabels(:), uint16(imgInstances(:))], 'rows');
  pairs(sum(pairs, 2) == 0, :) = [];
  
  N = size(pairs, 1);
  
  instanceMasks = false(H, W, N);
  instanceLabels = zeros(N, 1);
  for ii = 1 : N
    instanceMasks(:,:,ii) = imgObjectLabels == pairs(ii,1) & imgInstances == pairs(ii,2);
    instanceLabels(ii) = pairs(ii,1);
  end
end