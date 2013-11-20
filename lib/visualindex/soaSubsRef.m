function a = soaSubsRef(a, subs)
% SOASUBREF Subscript refernece of structure-of-arrays
%  B = SOASUBSREF(A, SUBS) subreferences a structore-of-arrays A and
%  reutrns tue specified subset in B.
%
%  Author:: Andrea Vedaldi

% AUTORIGHTS
% Copyright (C) 2008-09 Andrea Vedaldi
%
% This file is part of the VGG MKL Class and VGG MKL Det code packages,
% available in the terms of the GNU General Public License version 2.

names = fieldnames(a)' ;

for name = names
  name = char(name) ;
  a.(name) = a.(name)(:,subs) ;
end
