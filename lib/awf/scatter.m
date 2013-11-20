function h_out = scatter(v,style)

% SCATTER Plot a nx2 matrix as a set of points
%
if nargin == 1, style = '+'; end

if isempty(v),
	return
end

if ~isstr(style)
  set_h = style;
else
  set_h = [];
end

[r,c]=size(v);
if c == 2
  if set_h
    set(set_h, 'xdata', v(:,1), 'ydata', v(:,2));
  else
    h = plot(v(:,1),v(:,2),style);
  end
elseif c == 3
  if set_h
    set(set_h, 'xdata', v(:,1), 'ydata', v(:,2), 'zdata', v(:,3));
  else
    switch style
    case 'poly'
      h = patch(v(:,1), v(:,2), v(:,3), [1 1 1]/2);
    otherwise
      h = plot3(v(:,1), v(:,2), v(:,3), style);
    end
  end
else
  error('c != 2 or 3');
end

if nargout > 0
  h_out = h;
end
