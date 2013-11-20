% Extracts the timestamp from the filename.
%
% Example usage:
%   filename = [CLIPS_DIR '/r-1339729868.166858-2965701968.ppm']
%   matlabTime = get_timestamp_from_filename(filename);
%   disp(datestr(matlabTime, 'mm/dd/yy HH:MM:SS.FFF'));
% 
%
%
% Args:
%   filename - the path to the raw kinect output file.
%
% Returns:
%   matlabTime - the matlab time which can be passed to datestr
function matlabTime = get_timestamp_from_filename(filename)
  parts = regexp(filename(3:end), '-', 'split');
  millis = str2double(parts{2});

  % Time since the epoch, correcting for offset between UTC and EST
  % (technically a good chunk of the film was shot in CST, but se la vi).
  unixEpoch = datenum(1969,12,31,20,0,0);
  matlabTime = millis ./ 86400 + unixEpoch;
end