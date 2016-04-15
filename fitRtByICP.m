function RtICP = fitRtByICP(XYZcam_i, XYZcam_j)

params.ICP_WORST_REJECTION = .1;
params.ICP_MAXPOINTS_PER_SCAN = 10000;
params.ICP_MAXITER = 100;

MAXPOINTS = params.ICP_MAXPOINTS_PER_SCAN;


M =  [  reshape(XYZcam_i(:,:,1),1,[]);
        reshape(XYZcam_i(:,:,2),1,[]);
        reshape(XYZcam_i(:,:,3),1,[])];
M = M(:,reshape(XYZcam_i(:,:,4)~=0,1,[]));
subs = randperm(size(M,2));
subs = subs(1:min(length(subs),MAXPOINTS));
M = M(:,subs);


D =  [  reshape(XYZcam_j(:,:,1),1,[]);
        reshape(XYZcam_j(:,:,2),1,[]);
        reshape(XYZcam_j(:,:,3),1,[])];
D = D(:,reshape(XYZcam_j(:,:,4)~=0,1,[]));
subs = randperm(size(D,2));
subs = subs(1:min(length(subs),MAXPOINTS));
D = D(:,subs);


[Ricp, Ticp, ER] = icp(params,M, D, 'iter', params.ICP_MAXITER, 'Matching','kDtree', ...
                     'Minimize','plane','WorstRejection',params.ICP_WORST_REJECTION);
ER = ER(end);


RtICP = [Ricp  Ticp];
