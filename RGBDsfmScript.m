% ssh ionic.cs.princeton.edu
% cd /n/fs/sun3d/code/RGBDsfm_v6.0/
% /n/fs/vision/ionic/starter.sh RGBDsfmScript 12000mb 23:00:00 6 1 1 /n/fs/sun3d/code/ionic_log/

% to check jobs: showq
% qstat
% to kill jobs: qdel

function RGBDsfmScript(i)

    if matlabpool('size') ==0
        try
            matlabpool(6);
        catch
        end
    end

    load('/n/fs/sun3d/data/SUN3Dv1.mat');

    if ~exist(fullfile('/n/fs/sun3d/data/sfm',SUN3D{i}),'dir')
        try
            disp(i);
            [~,hname] = system('hostname');
            fprintf('RGBDsfmScript(%d: %s) at %s\n',i,SUN3D{i},hname);

            % do something
            RGBDsfm(SUN3D{i});

            fprintf('%d done\n',i);
        catch
            fprintf('%d error\n',i);
        end
    else
        fprintf('%d exists\n',i);
    end
end
