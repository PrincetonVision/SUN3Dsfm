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
function frameList = get_synched_frames(sceneDir)

  rgbImages = struct();
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
      frameList(numDepth).rawDepthFilename = files{ii};
    elseif ~isempty(regexp(files{ii}, '^r-*', 'once'))
      numRgb = numRgb + 1;
      rgbImages(numRgb).name = files{ii};
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
  kk = 1; % Current Accel pointer.
  for ii = 1 : numDepth
    fprintf('Matching depth image %d/%d\n', ii, numDepth);
    
    % Parse the timestamp.
    timePartsDepth = regexp(frameList(ii).rawDepthFilename(3:end), '-', 'split');
    timePartsRgb = regexp(rgbImages(jj).name(3:end), '-', 'split');
    timePartsAccel = regexp(accelRecs(kk).name(3:end), '-', 'split');
    
    tDepth = str2double(timePartsDepth{1});
    tRgb = str2double(timePartsRgb{1});
    tAccel = str2double(timePartsAccel{1});
    
    tDiff = abs(tDepth-tRgb);
    % Advance the curInd until the difference in times gets worse.
    while jj < numRgb
      timePartsRgb = regexp(rgbImages(jj+1).name(3:end), '-', 'split');
      tRgb = str2double(timePartsRgb{1});
      
      tmpDiff = abs(tDepth-tRgb);
      if tmpDiff > tDiff
        break;
      end
      tDiff = tmpDiff;
      
      % Otherwise, its better! and we should update jj
      jj = jj + 1;
    end
    frameList(ii).timeDiff = tDiff;
    
    %%%%%%% ACCEL %%%%%%
    tDiff = abs(tDepth-tAccel);
    % Advance the curInd until the difference in times gets worse.
    while kk < numAccel
      timePartsAccel = regexp(accelRecs(kk+1).name(3:end), '-', 'split');
      tAccel = str2double(timePartsAccel{1});
      
      tmpDiff = abs(tDepth-tAccel);
      if tmpDiff > tDiff
        break;
      end
      tDiff = tmpDiff;
      
      % Otherwise, its better! and we should update kk
      kk = kk + 1;
    end
    
    fprintf('Matched depth %d to rgb %d and accel %d.\n', ii, jj, kk);
    
    % Now save the current RGB filename and ACCEL filename.
    frameList(ii).rawRgbFilename = rgbImages(jj).name;
    frameList(ii).accelFilename = accelRecs(kk).name;
  end
  fprintf('\n');
end
