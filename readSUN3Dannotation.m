function [K, frameCount, frameImage, frameDepth] = readSUN3Dannotation(sequenceName, frameIDs)

    if ~exist('sequenceName','var')
        % load demo sequence        
        %sequenceName = 'hotel_mr/scan1';
        sequenceName = 'hotel_umd/maryland_hotel3';
        %sequenceName = 'brown_bm_1/brown_bm_1';
    end

    % the root path of SUN3D
    % change it to local
    %SUN3Dpath = '/data/vision/torralba/sun3d/record/scene_final';
    SUN3Dpath = 'http://sun3d.csail.mit.edu/data/';

    % read intrinsic
    K = reshape(readValuesFromTxt(fullfile(SUN3Dpath,sequenceName,'intrinsics.txt')),3,3)';
    
    % file list
    imageFiles = dirSmart(fullfile(SUN3Dpath,sequenceName,'image/'),'jpg');
    depthFiles = dirSmart(fullfile(SUN3Dpath,sequenceName,'depth/'),'png');
    extrinsicsFiles = dirSmart(fullfile(SUN3Dpath,sequenceName,'extrinsics/'),'txt');
       
    
    % read the latest version of extrinsic parameters for cameras
    extrinsicsC2W = permute(reshape(readValuesFromTxt(fullfile(SUN3Dpath,sequenceName,'extrinsics',extrinsicsFiles(end).name)),4,3,[]),[2 1 3]);

    % read time stamp
    imageFrameID = zeros(1,length(imageFiles));
    imageTimestamp = zeros(1,length(imageFiles));
    for i=1:length(imageFiles)
        id_time = sscanf(imageFiles(i).name, '%d-%d.jpg');
        imageFrameID(i) = id_time(1);
        imageTimestamp(i) = id_time(2);
    end
    depthFrameID = zeros(1,length(depthFiles));
    depthTimestamp = zeros(1,length(depthFiles));
    for i=1:length(depthFiles)
        id_time = sscanf(depthFiles(i).name, '%d-%d.png');
        depthFrameID(i) = id_time(1);
        depthTimestamp(i) = id_time(2);
    end
    
    % synchronize: find a depth for each image
    frameCount = length(imageFiles);
    IDimage2depth = zeros(1,frameCount);
    for i=1:frameCount
        [~, IDimage2depth(i)]=min(abs(double(depthTimestamp)-double(imageTimestamp(i))));
    end
    
    % read annotation:
    annotation = annotationRead(fullfile(SUN3Dpath,sequenceName,'annotation/index.json'));
    
       
    %plot(double(imageTimestamp)-double(depthTimestamp(IDimage2depth)))
    
    if ~exist('frameIDs','var')
        frameIDs = 1:frameCount;
    end
    
    cnt = 0;
    for frameID=frameIDs
        image = imageRead(fullfile(fullfile(SUN3Dpath,sequenceName,'image',imageFiles(frameID).name)));
        depth = depthRead(fullfile(fullfile(SUN3Dpath,sequenceName,'depth',depthFiles(IDimage2depth(frameID)).name)));
        
        cnt = cnt+1;
        frameImage(:,:,:,cnt) = image;
        frameDepth(:,:,cnt) = depth;
        
        
        XYZcamera = depth2XYZcamera(K, depth);

        % pick the valid points with their color
        valid = logical(XYZcamera(:,:,4));  valid = valid(:)';        
        XYZ = reshape(XYZcamera,[],4)';
        RGB = reshape(image,[],3)';
        XYZ = XYZ(1:3,valid);
        RGB = RGB(:,valid);
        
        % plot in camera coordinate
        % visualizePointCloud(XYZ,RGB); disp('check the point cloud.'); pause; 
        
        
        % transform to world coordinate
        XYZworld = transformPointCloud(XYZ,extrinsicsC2W(:,:,frameID));
        
        
        if frameID==frameIDs(1); clf; end
        
        subplot(1,2,1); 
        hold off;
        imshow(image);
        title(sprintf('Image and Annotation for Frame %d',frameID));
        
        % draw the annotation if the frame is a keyframe
        keyframeID = find(ismember(cellstr(annotation.fileList), imageFiles(frameID).name));
        if ~isempty(keyframeID)
            if ~isempty(annotation.frames{keyframeID})
                hold on
                LineWidth = 4;
                for polygonID = 1:length(annotation.frames{keyframeID}.polygon)
                    
                    X = annotation.frames{keyframeID}.polygon(polygonID).x;
                    Y = annotation.frames{keyframeID}.polygon(polygonID).y;
                    objectID = annotation.frames{keyframeID}.polygon(polygonID).object;
                    
                    color = ObjectColor(objectID);
                    
                    plot([X X(1)],[Y Y(1)], 'LineWidth', LineWidth, 'Color', [0 0 0]);
                    hold on;
                    
                    plot([X X(1)],[Y Y(1)], 'LineWidth', LineWidth/2, 'Color', color);
                    hold on;
                    
                    xx = mean(X);
                    yy = mean(Y);
                    ht=text(xx,yy, annotation.objects{objectID+1}.name, 'horizontalAlignment', 'center', 'verticalAlignment', 'bottom');
                    set(ht, 'color', color, 'fontsize', 10);
                    
                end
                
            end
        end
        
        % plot in world coordinate
        subplot(1,2,2) 
        visualizePointCloud(XYZworld,RGB,100); hold on; 
        title(sprintf('World Coordinate using Frame 1-%d',frameID));
        
        disp('check the point cloud, and press any key to continue.'); pause; 
    end
    
end

function color = ObjectColor(objectID)

% same color as the online annotator

objectColors = {'#1f77b4', '#aec7e8', '#ff7f0e', '#ffbb78', '#2ca02c', '#98df8a', '#d62728', '#ff9896', '#9467bd', '#c5b0d5', '#8c564b', '#c49c94', '#e377c2', '#f7b6d2', '#7f7f7f', '#c7c7c7', '#bcbd22', '#dbdb8d', '#17becf', '#9edae5', '#8dd3c7', '#ffffb3', '#bebada', '#fb8072', '#80b1d3', '#fdb462', '#b3de69', '#fccde5', '#d9d9d9', '#bc80bd', '#ccebc5', '#ffed6f', '#e41a1c', '#377eb8', '#4daf4a', '#984ea3', '#ff7f00', '#ffff33', '#a65628', '#f781bf', '#999999', '#621e15', '#e59076', '#128dcd', '#083c52', '#64c5f2', '#61afaf', '#0f7369', '#9c9da1', '#365e96', '#983334', '#77973d', '#5d437c', '#36869f', '#d1702f', '#8197c5', '#c47f80', '#acc484', '#9887b0', '#2d588a', '#58954c', '#e9a044', '#c12f32', '#723e77', '#7d807f', '#9c9ede', '#7375b5', '#4a5584', '#cedb9c', '#b5cf6b', '#8ca252', '#637939', '#e7cb94', '#e7ba52', '#bd9e39', '#8c6d31', '#e7969c', '#d6616b', '#ad494a', '#843c39', '#de9ed6', '#ce6dbd', '#a55194', '#7b4173', '#000000', '#0000FF'};

objectID = mod(objectID,length(objectColors)) + 1;

color = objectColors{objectID};

color = [hex2dec(color(2:3)) hex2dec(color(4:5)) hex2dec(color(6:7))]/255;

end

function values = readValuesFromTxt(filename)
    try
        values = textscan(urlread(filename),'%f');
    catch
        fid = fopen(filename,'r');
        values = textscan(fid,'%f');
        fclose(fid);
    end
    values = values{1};
end

function visualizePointCloud(XYZ,RGB, subsampleGap)
    if ~exist('subsampleGap','var')
        subsampleGap = 50;
    end
    XYZ = XYZ(:,1:subsampleGap:end);
    RGB = RGB(:,1:subsampleGap:end);
    scatter3(XYZ(1,:),XYZ(2,:),XYZ(3,:),ones(1,size(XYZ,2)),double(RGB)'/255,'filled');
    axis equal
    axis tight
end

function XYZcamera = depth2XYZcamera(K, depth)
    [x,y] = meshgrid(1:640, 1:480);
    XYZcamera(:,:,1) = (x-K(1,3)).*depth/K(1,1);
    XYZcamera(:,:,2) = (y-K(2,3)).*depth/K(2,2);
    XYZcamera(:,:,3) = depth;
    XYZcamera(:,:,4) = depth~=0;
end

function XYZtransform = transformPointCloud(XYZ,Rt)
    XYZtransform = Rt(1:3,1:3) * XYZ + repmat(Rt(1:3,4),1,size(XYZ,2));
end

function depth = depthRead(filename)
    depth = imread(filename);
    depth = bitor(bitshift(depth,-3), bitshift(depth,16-3));
    depth = single(depth)/1000;
end
%{
    % test to make sure it is correct
    for i=0:65535
        depth = uint16(i);
        code =bitor(bitshift(depth,3),bitshift(depth,3-16));
        recoverDepth = bitor(bitshift(code,-3), bitshift(code,16-3));
        if (depth~=recoverDepth)
            fprintf('error + %d\n',i);
        end
    end
%}

function image = imageRead(filename)
    image = imread(filename);
end

function files = dirSmart(page, tag)
    [files, status] = urldir(page, tag);
    if status == 0
        files = dir(fullfile(page, ['*.' tag]));
    end
end

function [files, status] = urldir(page, tag)
    if nargin == 1
        tag = '/';
    else
        tag = lower(tag);
        if strcmp(tag, 'dir')
            tag = '/';
        end
        if strcmp(tag, 'img')
            tag = 'jpg';
        end
    end
    nl = length(tag);
    nfiles = 0;
    files = [];

    % Read page
    page = strrep(page, '\', '/');
    [webpage, status] = urlread(page);

    if status
        % Parse page
        j1 = findstr(lower(webpage), '<a href="');
        j2 = findstr(lower(webpage), '</a>');
        Nelements = length(j1);
        if Nelements>0
            for f = 1:Nelements
                % get HREF element
                chain = webpage(j1(f):j2(f));
                jc = findstr(lower(chain), '">');
                chain = deblank(chain(10:jc(1)-1));

                % check if it is the right type
                if length(chain)>length(tag)-1
                    if strcmp(chain(end-nl+1:end), tag)
                        nfiles = nfiles+1;
                        chain = strrep(chain, '%20', ' '); % replace space character
                        files(nfiles).name = chain;
                        files(nfiles).bytes = 1;
                    end
                end
            end
        end
    end
end


function annotation = annotationRead(filename)

    try
        str = urlread(filename);
    catch
        str = file2string(filename);
    end
    
    annotation = loadjson(str);
end

function fileStr = file2string(fname)
    fileStr = '';
    fid = fopen(fname,'r');
    tline = fgetl(fid);
    while ischar(tline)
        fileStr = [fileStr ' ' tline];
        tline = fgetl(fid);
    end
    fclose(fid);
end


%% the rest of this file is all from JSON lab for reading a JSON file. 
% http://www.mathworks.com/matlabcentral/fileexchange/33381
% version: JSONLAB v1.0 alpha (Optimus) is updated on 08/23/2013. 

function data = loadjson(fname,varargin)
%
% data=loadjson(fname,opt)
%    or
% data=loadjson(fname,'param1',value1,'param2',value2,...)
%
% parse a JSON (JavaScript Object Notation) file or string
%
% authors:Qianqian Fang (fangq<at> nmr.mgh.harvard.edu)
%            date: 2011/09/09
%         Nedialko Krouchev: http://www.mathworks.com/matlabcentral/fileexchange/25713
%            date: 2009/11/02
%         François Glineur: http://www.mathworks.com/matlabcentral/fileexchange/23393
%            date: 2009/03/22
%         Joel Feenstra:
%         http://www.mathworks.com/matlabcentral/fileexchange/20565
%            date: 2008/07/03
%
% $Id: loadjson.m 394 2012-12-18 17:58:11Z fangq $
%
% input:
%      fname: input file name, if fname contains "{}" or "[]", fname
%             will be interpreted as a JSON string
%      opt: a struct to store parsing options, opt can be replaced by 
%           a list of ('param',value) pairs. The param string is equivallent
%           to a field in opt.
%
% output:
%      dat: a cell array, where {...} blocks are converted into cell arrays,
%           and [...] are converted to arrays
%
% license:
%     BSD, see LICENSE_BSD.txt files for details 
%
% -- this function is part of jsonlab toolbox (http://iso2mesh.sf.net/cgi-bin/index.cgi?jsonlab)
%

global pos inStr len  esc index_esc len_esc isoct arraytoken

if(regexp(fname,'[\{\}\]\[]','once'))
   string=fname;
elseif(exist(fname,'file'))
   fid = fopen(fname,'rt');
   string = fscanf(fid,'%c');
   fclose(fid);
else
   error('input file does not exist');
end

pos = 1; len = length(string); inStr = string;
isoct=exist('OCTAVE_VERSION');
arraytoken=find(inStr=='[' | inStr==']' | inStr=='"');
jstr=regexprep(inStr,'\\\\','  ');
escquote=regexp(jstr,'\\"');
arraytoken=sort([arraytoken escquote]);

% String delimiters and escape chars identified to improve speed:
esc = find(inStr=='"' | inStr=='\' ); % comparable to: regexp(inStr, '["\\]');
index_esc = 1; len_esc = length(esc);

opt=varargin2struct(varargin{:});
jsoncount=1;
while pos <= len
    switch(next_char)
        case '{'
            data{jsoncount} = parse_object(opt);
        case '['
            data{jsoncount} = parse_array(opt);
        otherwise
            error_pos('Outer level structure must be an object or an array');
    end
    jsoncount=jsoncount+1;
end % while

jsoncount=length(data);
if(jsoncount==1 && iscell(data))
    data=data{1};
end

if(~isempty(data))
      if(isstruct(data)) % data can be a struct array
          data=jstruct2array(data);
      elseif(iscell(data))
          data=jcell2array(data);
      end
end
end

%%
function newdata=parse_collection(id,data,obj)

if(jsoncount>0 && exist('data','var')) 
    if(~iscell(data))
       newdata=cell(1);
       newdata{1}=data;
       data=newdata;
    end
end
end

%%
function newdata=jcell2array(data)
len=length(data);
newdata=data;
for i=1:len
      if(isstruct(data{i}))
          newdata{i}=jstruct2array(data{i});
      elseif(iscell(data{i}))
          newdata{i}=jcell2array(data{i});
      end
end
end
%%-------------------------------------------------------------------------
function newdata=jstruct2array(data)
fn=fieldnames(data);
newdata=data;
len=length(data);
for i=1:length(fn) % depth-first
    for j=1:len
        if(isstruct(getfield(data(j),fn{i})))
            newdata(j)=setfield(newdata(j),fn{i},jstruct2array(getfield(data(j),fn{i})));
        end
    end
end
if(~isempty(strmatch('x0x5F_ArrayType_',fn)) && ~isempty(strmatch('x0x5F_ArrayData_',fn)))
  newdata=cell(len,1);
  for j=1:len
    ndata=cast(data(j).x0x5F_ArrayData_,data(j).x0x5F_ArrayType_);
    iscpx=0;
    if(~isempty(strmatch('x0x5F_ArrayIsComplex_',fn)))
        if(data(j).x0x5F_ArrayIsComplex_)
           iscpx=1;
        end
    end
    if(~isempty(strmatch('x0x5F_ArrayIsSparse_',fn)))
        if(data(j).x0x5F_ArrayIsSparse_)
            if(~isempty(strmatch('x0x5F_ArraySize_',fn)))
                dim=data(j).x0x5F_ArraySize_;
                if(iscpx && size(ndata,2)==4-any(dim==1))
                    ndata(:,end-1)=complex(ndata(:,end-1),ndata(:,end));
                end
                if isempty(ndata)
                    % All-zeros sparse
                    ndata=sparse(dim(1),prod(dim(2:end)));
                elseif dim(1)==1
                    % Sparse row vector
                    ndata=sparse(1,ndata(:,1),ndata(:,2),dim(1),prod(dim(2:end)));
                elseif dim(2)==1
                    % Sparse column vector
                    ndata=sparse(ndata(:,1),1,ndata(:,2),dim(1),prod(dim(2:end)));
                else
                    % Generic sparse array.
                    ndata=sparse(ndata(:,1),ndata(:,2),ndata(:,3),dim(1),prod(dim(2:end)));
                end
            else
                if(iscpx && size(ndata,2)==4)
                    ndata(:,3)=complex(ndata(:,3),ndata(:,4));
                end
                ndata=sparse(ndata(:,1),ndata(:,2),ndata(:,3));
            end
        end
    elseif(~isempty(strmatch('x0x5F_ArraySize_',fn)))
        if(iscpx && size(ndata,2)==2)
             ndata=complex(ndata(:,1),ndata(:,2));
        end
        ndata=reshape(ndata(:),data(j).x0x5F_ArraySize_);
    end
    newdata{j}=ndata;
  end
  if(len==1)
      newdata=newdata{1};
  end
end
end
%%-------------------------------------------------------------------------
function object = parse_object(varargin)
    parse_char('{');
    object = [];
    if next_char ~= '}'
        while 1
            str = parseStr(varargin{:});
            if isempty(str)
                error_pos('Name of value at position %d cannot be empty');
            end
            parse_char(':');
            val = parse_value(varargin{:});
            eval( sprintf( 'object.%s  = val;', valid_field(str) ) );
            if next_char == '}'
                break;
            end
            parse_char(',');
        end
    end
    parse_char('}');
end
%%-------------------------------------------------------------------------

function object = parse_array(varargin) % JSON array is written in row-major order
global pos inStr isoct
    parse_char('[');
    object = cell(0, 1);
    dim2=[];
    if next_char ~= ']'
        [endpos e1l e1r maxlevel]=matching_bracket(inStr,pos);
        arraystr=['[' inStr(pos:endpos)];
        arraystr=regexprep(arraystr,'"_NaN_"','NaN');
        arraystr=regexprep(arraystr,'"([-+]*)_Inf_"','$1Inf');
        arraystr(find(arraystr==sprintf('\n')))=[];
        arraystr(find(arraystr==sprintf('\r')))=[];
        %arraystr=regexprep(arraystr,'\s*,',','); % this is slow,sometimes needed
        if(~isempty(e1l) && ~isempty(e1r)) % the array is in 2D or higher D
            astr=inStr((e1l+1):(e1r-1));
            astr=regexprep(astr,'"_NaN_"','NaN');
            astr=regexprep(astr,'"([-+]*)_Inf_"','$1Inf');
            astr(find(astr==sprintf('\n')))=[];
            astr(find(astr==sprintf('\r')))=[];
            astr(find(astr==' '))='';
            if(isempty(find(astr=='[', 1))) % array is 2D
                dim2=length(sscanf(astr,'%f,',[1 inf]));
            end
        else % array is 1D
            astr=arraystr(2:end-1);
            astr(find(astr==' '))='';
            [obj count errmsg nextidx]=sscanf(astr,'%f,',[1,inf]);
            if(nextidx>=length(astr)-1)
                object=obj;
                pos=endpos;
                parse_char(']');
                return;
            end
        end
        if(~isempty(dim2))
            astr=arraystr;
            astr(find(astr=='['))='';
            astr(find(astr==']'))='';
            astr(find(astr==' '))='';
            [obj count errmsg nextidx]=sscanf(astr,'%f,',inf);
            if(nextidx>=length(astr)-1)
                object=reshape(obj,dim2,numel(obj)/dim2)';
                pos=endpos;
                parse_char(']');
                return;
            end
        end
        arraystr=regexprep(arraystr,'\]\s*,','];');
        try
           if(isoct && regexp(arraystr,'"','once'))
                error('Octave eval can produce empty cells for JSON-like input');
           end
           object=eval(arraystr);
           pos=endpos;
        catch
         while 1
            val = parse_value(varargin{:});
            object{end+1} = val;
            if next_char == ']'
                break;
            end
            parse_char(',');
         end
        end
    end
    if(jsonopt('SimplifyCell',0,varargin{:})==1)
      try
        oldobj=object;
        object=cell2mat(object')';
        if(iscell(oldobj) && isstruct(object) && numel(object)>1 && jsonopt('SimplifyCellArray',1,varargin{:})==0)
            object=oldobj;
        elseif(size(object,1)>1 && ndims(object)==2)
            object=object';
        end
      catch
      end
    end
    parse_char(']');
end
%%-------------------------------------------------------------------------

function parse_char(c)
    global pos inStr len
    skip_whitespace;
    if pos > len || inStr(pos) ~= c
        error_pos(sprintf('Expected %c at position %%d', c));
    else
        pos = pos + 1;
        skip_whitespace;
    end
end
%%-------------------------------------------------------------------------

function c = next_char
    global pos inStr len
    skip_whitespace;
    if pos > len
        c = [];
    else
        c = inStr(pos);
    end
end
%%-------------------------------------------------------------------------

function skip_whitespace
    global pos inStr len
    while pos <= len && isspace(inStr(pos))
        pos = pos + 1;
    end
end
%%-------------------------------------------------------------------------
function str = parseStr(varargin)
    global pos inStr len  esc index_esc len_esc
 % len, ns = length(inStr), keyboard
    if inStr(pos) ~= '"'
        error_pos('String starting with " expected at position %d');
    else
        pos = pos + 1;
    end
    str = '';
    while pos <= len
        while index_esc <= len_esc && esc(index_esc) < pos
            index_esc = index_esc + 1;
        end
        if index_esc > len_esc
            str = [str inStr(pos:len)];
            pos = len + 1;
            break;
        else
            str = [str inStr(pos:esc(index_esc)-1)];
            pos = esc(index_esc);
        end
        nstr = length(str); switch inStr(pos)
            case '"'
                pos = pos + 1;
                if(~isempty(str))
                    if(strcmp(str,'_Inf_'))
                        str=Inf;
                    elseif(strcmp(str,'-_Inf_'))
                        str=-Inf;
                    elseif(strcmp(str,'_NaN_'))
                        str=NaN;
                    end
                end
                return;
            case '\'
                if pos+1 > len
                    error_pos('End of file reached right after escape character');
                end
                pos = pos + 1;
                switch inStr(pos)
                    case {'"' '\' '/'}
                        str(nstr+1) = inStr(pos);
                        pos = pos + 1;
                    case {'b' 'f' 'n' 'r' 't'}
                        str(nstr+1) = sprintf(['\' inStr(pos)]);
                        pos = pos + 1;
                    case 'u'
                        if pos+4 > len
                            error_pos('End of file reached in escaped unicode character');
                        end
                        str(nstr+(1:6)) = inStr(pos-1:pos+4);
                        pos = pos + 5;
                end
            otherwise % should never happen
                str(nstr+1) = inStr(pos), keyboard
                pos = pos + 1;
        end
    end
    error_pos('End of file while expecting end of inStr');
end
%%-------------------------------------------------------------------------

function num = parse_number(varargin)
    global pos inStr len isoct
    currstr=inStr(pos:end);
    numstr=0;
    if(isoct~=0)
        numstr=regexp(currstr,'^\s*-?(?:0|[1-9]\d*)(?:\.\d+)?(?:[eE][+\-]?\d+)?','end');
        [num, one] = sscanf(currstr, '%f', 1);
        delta=numstr+1;
    else
        [num, one, err, delta] = sscanf(currstr, '%f', 1);
        if ~isempty(err)
            error_pos('Error reading number at position %d');
        end
    end
    pos = pos + delta-1;
end
%%-------------------------------------------------------------------------

function val = parse_value(varargin)
    global pos inStr len
    true = 1; false = 0;

    switch(inStr(pos))
        case '"'
            val = parseStr(varargin{:});
            return;
        case '['
            val = parse_array(varargin{:});
            return;
        case '{'
            val = parse_object(varargin{:});
            return;
        case {'-','0','1','2','3','4','5','6','7','8','9'}
            val = parse_number(varargin{:});
            return;
        case 't'
            if pos+3 <= len && strcmpi(inStr(pos:pos+3), 'true')
                val = true;
                pos = pos + 4;
                return;
            end
        case 'f'
            if pos+4 <= len && strcmpi(inStr(pos:pos+4), 'false')
                val = false;
                pos = pos + 5;
                return;
            end
        case 'n'
            if pos+3 <= len && strcmpi(inStr(pos:pos+3), 'null')
                val = [];
                pos = pos + 4;
                return;
            end
    end
    error_pos('Value expected at position %d');
end
%%-------------------------------------------------------------------------

function error_pos(msg)
    global pos inStr len
    poShow = max(min([pos-15 pos-1 pos pos+20],len),1);
    if poShow(3) == poShow(2)
        poShow(3:4) = poShow(2)+[0 -1];  % display nothing after
    end
    msg = [sprintf(msg, pos) ': ' ...
    inStr(poShow(1):poShow(2)) '<error>' inStr(poShow(3):poShow(4)) ];
    error( ['JSONparser:invalidFormat: ' msg] );
end
%%-------------------------------------------------------------------------

function str = valid_field(str)
global isoct
% From MATLAB doc: field names must begin with a letter, which may be
% followed by any combination of letters, digits, and underscores.
% Invalid characters will be converted to underscores, and the prefix
% "x0x[Hex code]_" will be added if the first character is not a letter.
    pos=regexp(str,'^[^A-Za-z]','once');
    if(~isempty(pos))
        if(~isoct)
            str=regexprep(str,'^([^A-Za-z])','x0x${sprintf(''%X'',unicode2native($1))}_','once');
        else
            str=sprintf('x0x%X_%s',char(str(1)),str(2:end));
        end
    end
    if(isempty(regexp(str,'[^0-9A-Za-z_]', 'once' ))) return;  end
    if(~isoct)
        str=regexprep(str,'([^0-9A-Za-z_])','_0x${sprintf(''%X'',unicode2native($1))}_');
    else
        pos=regexp(str,'[^0-9A-Za-z_]');
        if(isempty(pos)) return; end
        str0=str;
        pos0=[0 pos(:)' length(str)];
        str='';
        for i=1:length(pos)
            str=[str str0(pos0(i)+1:pos(i)-1) sprintf('_0x%X_',str0(pos(i)))];
        end
        if(pos(end)~=length(str))
            str=[str str0(pos0(end-1)+1:pos0(end))];
        end
    end
    %str(~isletter(str) & ~('0' <= str & str <= '9')) = '_';
end
%%-------------------------------------------------------------------------
function endpos = matching_quote(str,pos)
len=length(str);
while(pos<len)
    if(str(pos)=='"')
        if(~(pos>1 && str(pos-1)=='\'))
            endpos=pos;
            return;
        end        
    end
    pos=pos+1;
end
error('unmatched quotation mark');
end
%%-------------------------------------------------------------------------
function [endpos e1l e1r maxlevel] = matching_bracket(str,pos)
global arraytoken
level=1;
maxlevel=level;
endpos=0;
bpos=arraytoken(arraytoken>=pos);
tokens=str(bpos);
len=length(tokens);
pos=1;
e1l=[];
e1r=[];
while(pos<=len)
    c=tokens(pos);
    if(c==']')
        level=level-1;
        if(isempty(e1r)) e1r=bpos(pos); end
        if(level==0)
            endpos=bpos(pos);
            return
        end
    end
    if(c=='[')
        if(isempty(e1l)) e1l=bpos(pos); end
        level=level+1;
        maxlevel=max(maxlevel,level);
    end
    if(c=='"')
        pos=matching_quote(tokens,pos+1);
    end
    pos=pos+1;
end
if(endpos==0) 
    error('unmatched "]"');
end

end

function opt=varargin2struct(varargin)
%
% opt=varargin2struct('param1',value1,'param2',value2,...)
%   or
% opt=varargin2struct(...,optstruct,...)
%
% convert a series of input parameters into a structure
%
% authors:Qianqian Fang (fangq<at> nmr.mgh.harvard.edu)
% date: 2012/12/22
%
% input:
%      'param', value: the input parameters should be pairs of a string and a value
%       optstruct: if a parameter is a struct, the fields will be merged to the output struct
%
% output:
%      opt: a struct where opt.param1=value1, opt.param2=value2 ...
%
% license:
%     BSD, see LICENSE_BSD.txt files for details 
%
% -- this function is part of jsonlab toolbox (http://iso2mesh.sf.net/cgi-bin/index.cgi?jsonlab)
%

len=length(varargin);
opt=struct;
if(len==0) return; end
i=1;
while(i<=len)
    if(isstruct(varargin{i}))
        opt=mergestruct(opt,varargin{i});
    elseif(ischar(varargin{i}) && i<len)
        opt=setfield(opt,varargin{i},varargin{i+1});
        i=i+1;
    else
        error('input must be in the form of ...,''name'',value,... pairs or structs');
    end
    i=i+1;
end

end

function val=jsonopt(key,default,varargin)
%
% val=jsonopt(key,default,optstruct)
%
% setting options based on a struct. The struct can be produced
% by varargin2struct from a list of 'param','value' pairs
%
% authors:Qianqian Fang (fangq<at> nmr.mgh.harvard.edu)
%
% $Id: loadjson.m 371 2012-06-20 12:43:06Z fangq $
%
% input:
%      key: a string with which one look up a value from a struct
%      default: if the key does not exist, return default
%      optstruct: a struct where each sub-field is a key 
%
% output:
%      val: if key exists, val=optstruct.key; otherwise val=default
%
% license:
%     BSD, see LICENSE_BSD.txt files for details
%
% -- this function is part of jsonlab toolbox (http://iso2mesh.sf.net/cgi-bin/index.cgi?jsonlab)
% 

val=default;
if(nargin<=2) return; end
opt=varargin{1};
if(isstruct(opt) && isfield(opt,key))
    val=getfield(opt,key);
end

end