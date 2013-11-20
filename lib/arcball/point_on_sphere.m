
%%%%%%%%%%%% UTILITY FUNCTION %%%%%%%%%%%%%
function P = point_on_sphere( P, width, height )
    P(3) = 0;
    
    % determine radius of the sphere
    R = min(width, height)/2;
    
    % TRANSFORM the point in window coordinate into 
    % the coordinate of a sphere centered in middle window
    ORIGIN = [width/2, height/2, 0];
    P = P - ORIGIN;
    
    % normalize position to [-1:1] WRT unit sphere 
    % centered at the origin of the window
    P = P / R;
    
    % if position is out of sphere, normalize it to  
    % unit length
    L = sqrt( P*P' );
    if L > 1
       % P = nan; % workaround to stop evaluation
       % disp('out of sphere');
       
       P = P / L; 
       P(3) = 0;
    else
       % add the Z coordinate to the scheme
       P(3) = sqrt( 1 - P(1)^2 - P(2)^2 );
    end
end