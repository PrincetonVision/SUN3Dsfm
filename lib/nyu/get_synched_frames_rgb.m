% Returns a struct with synchronized RGB and depth frames, as well as the
% accelerometer data. Note that this script considers the depth frames as
% 'primary' in the sense that it keeps every depth frame and matches the
% nearest RGB frame.
%
% Args:
%   sceneDir - the directory containing the raw kinect dump for a
%   particular scene.
%
% Returns:
%   frameList - a struct containing a list of frames.
function frameList = get_synched_frames_rgb(sceneDir)

  depthImages = struct();
  frameList = struct();
  accelRecs = struct();
  
  % Faster than matlab's Dir function for big directories and slow
  % distributed file systems...
  files = regexp(ls(sceneDir), '(\s+|\n)', 'split');
  files(end) = [];
  
  files = sort(files);
  
  % Count the number of files of each type found in the scene.
  numDepth = 0;
  numRgb = 0;
  numAccel = 0;
  
  for ii = 1 : numel(files)
    if ~isempty(regexp(files{ii}, '^d-*', 'once'))
      numDepth = numDepth + 1;
      depthImages(numDepth).name = files{ii};
    elseif ~isempty(regexp(files{ii}, '^r-*', 'once'))
      numRgb = numRgb + 1;
      frameList(numRgb).rawRgbFilename = files{ii};
    elseif ~isempty(regexp(files{ii}, '^a-*', 'once'))
      numAccel = numAccel + 1;
      accelRecs(numAccel).name = files{ii};
    end
  end
  
  fprintf('Found %d depth, %d rgb images, and %d accel dumps.\n', ...
      numDepth, numRgb, numAccel);
  
  % Now, go through both images sets, grabbing the rgb and accelerometer
  % data that is nearest to the current timestamp.
  
  jj = 1; % Current RGB pointer.
  for ii = 1 : numRgb
    fprintf('Matching depth image %d/%d\n', ii, numRgb);
    
    % Parse the timestamp.
    timePartsDepth = regexp(depthImages(jj).name(3:end), '-', 'split');
    timePartsRgb = regexp(frameList(ii).rawRgbFilename(3:end), '-', 'split');
    
    tDepth = str2double(timePartsDepth{1});
    tRgb = str2double(timePartsRgb{1});

    
    tDiff = abs(tDepth-tRgb);
    % Advance the curInd until the difference in times gets worse.
    while jj < numDepth
      timePartsDepth = regexp(depthImages(jj+1).name(3:end), '-', 'split');
      tDepth = str2double(timePartsDepth{1});
      
      tmpDiff = abs(tDepth-tRgb);
      if tmpDiff > tDiff
        break;
      end
      tDiff = tmpDiff;
      
      % Otherwise, its better! and we should update jj
      jj = jj + 1;
    end
    frameList(ii).timeDiff = tDiff;
    frameList(ii).rawDepthFilename = depthImages(jj).name;

    
    fprintf('Matched rgb %d to depth %d.\n', ii, jj);

  end
  fprintf('\n');
end
