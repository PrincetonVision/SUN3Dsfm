% Crops the given image to use only the portion where the projected depth
% image exists.
%
% Args:
%   img - either a HxW image or a HxWxD image.
%
% Returns:
%   img - a cropped version of the image.
function img = crop_image(img)
  [mask, sz] = get_projection_mask();
  switch ndims(img)
    case 2
      img = reshape(img(mask), sz);
    case 3
      img = reshape(img, [480*640 3]);
      img = reshape(img(mask,:), [sz 3]);
    otherwise
      error('not supported');
  end
end