function [conf, imdb] = db_helper()
% DB_HELPER

  rootDir = fileparts(mfilename('fullpath')) ;

  switch 1
    case 1
      conf.imageDir = fullfile(rootDir, 'data','oxbuild_images') ;
      conf.gtDir = fullfile(rootDir, 'data', 'oxbuild_gt') ;
      conf.numWords = 50000 ;
      buildDb = @() parseOxfordDb(conf.imageDir, conf.gtDir) ;
  end

  conf.dataDir = [conf.imageDir '-index'] ;
  conf.thumbDir = [conf.imageDir] ;
  conf.imdbPath = fullfile(conf.dataDir, 'imbd.mat') ;
  conf.modelPath = fullfile(conf.dataDir, 'model.mat') ;

  if exist(conf.imdbPath,'file')
    imdb = load(conf.imdbPath) ;
  else
    imdb = buildDb() ;
    vl_xmkdir(fileparts(conf.imdbPath)) ;
    save(conf.imdbPath, '-STRUCT', 'imdb') ;
  end
end

% --------------------------------------------------------------
function imdb = parseOxfordDb(imageDir, gtDir, varargin)
% --------------------------------------------------------------

  imdb.dir = imageDir ;
  imdb.sets.TRAIN = uint8(1) ;
  imdb.sets.TEST = uint8(2) ;

  if ~exist(gtDir, 'dir')
    fprintf('Downloading and unpacking Oxford building datset gt to %s\n', gtDir) ;
    vl_xmkdir(gtDir) ;
    untar('http://www.robots.ox.ac.uk/~vgg/data/oxbuildings/gt_files_170407.tgz',gtDir) ;
  end
  if ~exist(imageDir, 'dir')
    fprintf('Downloading and unpacking Oxford building datset images to %s\n', imageDir) ;
    vl_xmkdir(imageDir) ;
    untar('http://www.robots.ox.ac.uk/~vgg/data/oxbuildings/oxbuild_images.tgz',imageDir) ;
  end

  files = dir(fullfile(gtDir, '*.txt')) ;
  files = {files(~[files.isdir]).name} ;

  classes = {} ;
    function c = decodeClass(cl)
    cl = char(cl) ;
    c = find(strcmp(cl,classes)) ;
    if ~isempty(c), return ; end
    classes{end+1} = cl ;
    c = length(classes) ;
  end

  t = 0 ;
  for i = 1:length(files)
    a = regexp(files{i}, '([\w_]+)_[0-9]+_query.txt', 'tokens') ;
    if ~isempty(a),
      cl= a{1} ;
      instances = textread(fullfile(gtDir, files{i}), ...
                           '%s %*f %*f %*f %*f') ;
      for j = 1:length(instances)
        t = t + 1 ;
        imdb.images.id(t) = t ;
        imdb.images.name{t} = [instances{j}(6:end) '.jpg'] ;
        imdb.images.set(t) = imdb.sets.TEST ;
        imdb.images.size(:,t) = [0;0] ;
        imdb.images.class(t) = decodeClass(cl) ;
      end
    end
    a = regexp(files{i}, '([\w_]+)_[0-9]+_good.txt', 'tokens') ;
    if ~isempty(a),
      cl = a{1} ;
      instances = textread(fullfile(gtDir, files{i}), ...
                           '%s') ;
      for j = 1:length(instances)
        t = t + 1 ;
        imdb.images.id(t) = t ;
        imdb.images.name{t} = [instances{j} '.jpg'] ;
        imdb.images.set(t) = imdb.sets.TRAIN ;
        imdb.images.size(:,t) = [0;0] ;
        imdb.images.class(t) = decodeClass(cl) ;
      end
    end
  end
  imdb.classNames = classes ;

  % remove duplicates
  [~,sel] = unique(imdb.images.name) ;
  imdb.images = soaSubsRef(imdb.images, sel) ;

  for k = 1:length(imdb.images.id)
    imdb.images.id(k) = k ;
    imageName = imdb.images.name{k} ;
    info = imfinfo(fullfile(imdb.dir, imageName)) ;
    imdb.images.size(:,k) = [info.Width ; info.Height] ;
  end
end

