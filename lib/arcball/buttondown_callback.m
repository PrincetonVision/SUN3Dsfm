% only 1 event on click
function buttondown_callback( src, evnt )
    % change status
    mousestatus = 'buttondown';
    % retrieve the current point
    P = get(gcf,'CurrentPoint');
    % retrieve window geometry
    HGEOM = get( src, 'Position');
    % SET START POSITION
    START = point_on_sphere( P, HGEOM(3), HGEOM(4) );
    % SET START MATRIX
    M_previous = get( h, 'Matrix' );    
end