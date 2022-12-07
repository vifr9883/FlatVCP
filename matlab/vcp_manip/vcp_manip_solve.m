function sol = vcp_manip_solve(data)
%VCP_MANIP_SOLVE  Yalmip Optimizer for FQP.
%   vcp = VCP_MANIP_FQP() returns a struct with a YALMIP optimizer
%   object modeling FQP.
%
%   The struct has fields:
%   - d [1,1] B-spline degree
%   - N [1,1] Number of control points (N+1)
%   - tau [N+d+1,1] Knot vector, clamped, uniform
%   - n [1,1] number of links
%   - opt [] optimizer object
%
%   The optimizer object is [vcp.opt] and it has:
%   Inputs:
%   - x_0 [n+2,1] initial state
%   - x_f [n+2,1] final state
%   - th_lb [n,1] theta lower-bound vector
%   - th_ub [n,1] theta upper-bound vector
%   - u_lim [1,1] Actuation effor limit
%
%   Outputs:
%   - P [n,N+1] control points
%
%   Copyright (c) 2022, University of Colorado Boulder

%% Setup Problem
% Setup B-spline
vcp.d = 2; % degree
vcp.N = 20; % Number of control points
vcp.tau = bspline_knots(data.tf,vcp.N,vcp.d); % Knot vector
int_res = 50; % Integral approximation resolution

% Variables
P = sdpvar(data.n,vcp.N+1); % Control Points

% Init problem
obj = 0;
cst = [];

%% Objective
t_sig_obj =  linspace(vcp.tau(1),vcp.tau(end),int_res)'; % approx integral
dtheta = sdpvar(size(P,1),length(t_sig_obj)); % dtheta(t)
for k = 1:length(t_sig_obj)
  cst = [cst, dtheta(:,k) == bspline_curve(t_sig_obj(k),P,1,vcp.d,vcp.tau)];
  obj = obj + dtheta(:,k)'*dtheta(:,k); % \|dtheta(t)\|^2;
end

%% Constraints
% p_0 = theta_0
cst = [cst, P(:,1) == data.x_0(3:end)];
% p_N = theta_f
cst = [cst, P(:,end) == data.x_f(3:end)];
% Joint limits
for j = 0:vcp.N
  cst = [cst, data.th_lb <= P(:,j+1) <= data.th_ub];
end
% Input limit
for j = 1:vcp.N
  cst = [cst, norm(bspline_vcp(1,P,j,vcp.d,vcp.tau),inf) <= data.u_lim];
end

%% Options
options = sdpsettings('verbose',0,'solver','mosek','debug',0);

%% Solve FQP
% Solve IG
sol = optimize(cst,obj,options);
sol.P = value(P);
sol.d = vcp.d;
sol.N = vcp.N;
