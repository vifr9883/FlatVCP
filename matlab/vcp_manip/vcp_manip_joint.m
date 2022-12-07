function r = vcp_manip_joint(theta, j, data)
%VCP_MANIP_JOINT  Computes the position of the j-th joint given theta. 
%   r = VCP_MANIP_JOINT(theta, j, data) returns the position of the j-th joint.
%
%   Copyright (c) 2022, University of Colorado Boulder

assert(j <= data.n,"Invalid joint: j <= n")
r = zeros(2, size(theta,2));
for k = 1:size(theta,2)
  for i = 1:j
    r(1,k) = r(1,k) + data.l(i)*cos(sum(theta(1:i,k)));
    r(2,k) = r(2,k) + data.l(i)*sin(sum(theta(1:i,k)));
  end
end