% motion callback, event "every" mouse movement
function motion_callback(src, event)
    % retrieve the current point
    P = get(gcf,'CurrentPoint');
    % retrieve window geometry
    HGEOM = get( src, 'Position');
    % transform in sphere coordinates (3,4) = (WIDTH, HEIGHT)
    P = point_on_sphere( P, HGEOM(3), HGEOM(4) );
    
    % workaround condition (see point_on_sphere)
    if isnan(P)
       return; 
    end
    
    %%%%% ARCBALL COMPUTATION %%%%%
    if strcmp(mousestatus, 'buttondown')
        % compute angle and rotation axis
        rot_dir = cross( START, P ); rot_dir = rot_dir / norm( rot_dir );
        rot_ang = acos( dot( P, START ) );
       
        % convert direction in model coordinate system
        M_tr = inv( M_previous );
        rot_dir = M_tr*[rot_dir,0]';
        rot_dir = rot_dir(1:3);
        rot_dir = rot_dir / norm( rot_dir ); % renormalize
        % construct matrix
        R_matrix = makehgtform('axisrotate',rot_dir,rot_ang);
        % set hgt matrix
        set(h,'Matrix',M_previous*R_matrix);
        % refresh drawing
        drawnow;
    end
end