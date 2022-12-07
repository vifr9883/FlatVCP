function vcp = vcp_manip_fqp()
%VCP_MANIP_FQP  Yalmip Optimizer for FQP.
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
vcp.n = 4; % Number of links
vcp.tf = 10; % Final time

% Setup B-spline
vcp.d = 2; % degree
vcp.N = 20; % Number of control points
vcp.tau = bspline_knots(vcp.tf,vcp.N,vcp.d); % Knot vector
int_res = 50; % Integral approximation resolution

% Parameters
x_0 = sdpvar(vcp.n+2,1); % Initial state
x_f = sdpvar(vpc.n+2,1); % Final state
th_lb = sdpvar(vcp.n,1); % theta lower-bound vector
th_ub = sdpvar(vcp.n,1); % theta upper-bound vector
u_lim = sdpvar(1,1); % Actuation effor limit


% Variables
P = sdpvar(vcp.n,vcp.N+1); % Control Points

% Init problem
obj = 0;
cst = [];

%% Objective
t_sig_obj =  linspace(vcp.tau(1),vcp.tau(end),int_res)'; % approx integral
dtheta = sdpvar(vcp.n,length(t_sig_obj)); % dtheta(t)
for k = 1:length(t_sig_obj)
  cst = [cst, dtheta(:,k) == bspline_curve(t_sig_obj(k),P,1,vcp.d,vcp.tau)];
  obj = obj + dtheta(:,k)'*dtheta(:,k); % \|dtheta(t)\|^2;
end

%% Constraints
% \theta(0) = r_0
cst = [cst, bspline_curve(0,P,0,vcp.d,vcp.tau) == x_0(1:2)];
% \theta(1) = r_f
cst = [cst, bspline_curve(1,P,0,vcp.d,vcp.tau) == x_f(1:2)];
% \theta'(0) = ubv_th*[cos\\sin]
cst = [cst, bspline_curve(0,P,1,vcp.d,vcp.tau) == ubv_th*[cos(x_0(4));sin(x_0(4))]];
% \theta'(1) = ubv_th*[cos\\sin]
cst = [cst, bspline_curve(1,P,1,vcp.d,vcp.tau) == ubv_th*[cos(x_f(4));sin(x_f(4))]];

% \|\theta'\|_2 <= ubv_th
r = 1;
for j = r:vcp.N
  cst = [cst, cone(bspline_vcp(r,P,j,vcp.d,vcp.tau),ubv_th)];
end



% \|\theta''\|_2 <= uba_th
r = 2;
for j = r:vcp.N
  cst = [cst, cone(bspline_vcp(r,P,j,vcp.d,vcp.tau),uba_th)];
end

% |\gamma| <= gamma_max
% \|\theta'\|_2 >= lbv_th
cst_gamma = [cst_gamma, lbv_th >= 0];
for j = 1:vcp.N
    d_hat = (x_f(1:2)-x_0(1:2))/distance;
    cst_gamma = [cst_gamma, d_hat'*bspline_vcp(1,P,j,vcp.d,vcp.tau) >= lbv_th];
end
% Quadratic lower bount
cst_gamma = [cst_gamma, beta >=0];
gamma_tilde = tan(gamma_max)/L;
cst_gamma = [cst_gamma, cone([2*alpha; 4*gamma_tilde*beta-1],... 
       4*gamma_tilde*beta + 1)];
cst_gamma = [cst_gamma, uba_th <= alpha*lbv_th - beta];


%% Options
opt = sdpsettings('verbose',0,'solver','mosek','debug',0);

%% Setup Optimizer
% Unconstraned gamma max
vcp.opt_ucst = optimizer(cst,obj,opt,{x_0,x_f,distance,gamma_max,alpha,L},{P, ubv_th, uba_th});
% Constrained gamma max
vcp.opt = optimizer([cst, cst_gamma],obj+obj_gamma,opt,{x_0,x_f,distance,gamma_max,alpha,L},{P, ubv_th, uba_th});
