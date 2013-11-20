function p = icp_param(R, t)


q = dcm2quat(R(1:3,1:3));
q = [q(2:4) q(1)];
p = [q t'];


    

[RR,tt] = icp_deparam(p);
if (sum(sum(abs(RR-R))) + sum(abs(tt-t))) == 0
    error('icp_param');
end
