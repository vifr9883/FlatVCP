function data = vcp_manip_data()
%VCP_MANIP_DATA  Generates a default structure with
%              data for the FQP problem.
%   data = VCP_MANIP_DATA() returns the struct with fields:
%       - x_0 [n+2,1] Initial State [m, m, m/s, rad]
%       - x_f [4,1] Final State [m, m, m/s, rad]
%       - v_max [1,1] Maximum velocity [m/s]
%       - a_max [1,1] Maximum acceleration [m/s^2]
%       - gamma_max [1,1] Maximum steering angle [rad]
%       - nu [1,1] Weight factor for t_f (high nu -> fast traj)
%       - L [1,1] Wheelbase length [m]
%
%   Copyright (c) 2022, University of Colorado Boulder

data.n = 4; % Number of links
data.l = ones(data.n,1); % Link lengths
data.tf = 10; % Final time

% Extremes
th_0 = zeros(data.n,1); % Initial joing config
th_f = deg2rad(10)*ones(data.n,1); % Final joing config
data.x_0 = [vcp_manip_joint(th_0, data.n, data); th_0]; % Initial state
data.x_f = [vcp_manip_joint(th_f, data.n, data); th_f]; % Final state

% Constraints
data.th_lb = deg2rad(0)*ones(data.n,1); % theta lower-bound vector
data.th_ub = deg2rad(10)*ones(data.n,1); % theta upper-bound vector
data.u_lim = deg2rad(2); % Actuation effor limit


