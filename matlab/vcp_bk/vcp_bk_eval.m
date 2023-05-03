function [x, u] = vcp_bk_eval(sol, t)
%VCP_BK_EVAL  Produce state x(t) and input u(t) from FlatVCP sol.
%   [x, u] = VCP_BK_INFLATE(sol,t) returns x(t), u(t) state and input
%                   vectors evaluated at time t. The state
%                   vector x is [4, 1] and the input vector u is
%                   [3, 1].
%
%   see vcp_bk_data() and vcp_bk_solve()
%
%
%   Copyright (c) 2023, University of Colorado Boulder

%% Vectors to return
x = zeros(4,1);
u = zeros(3,1);

%% Compute speed profile
s = bspline_curve(t,sol.s.P,0,sol.s.d,bspline_knots(sol.t_f,sol.s.N,sol.s.d));
s_dot = bspline_curve(t,sol.s.P,1,sol.s.d,bspline_knots(sol.t_f,sol.s.N,sol.s.d));
s_ddot = bspline_curve(t,sol.s.P,2,sol.s.d,bspline_knots(sol.t_f,sol.s.N,sol.s.d));

%% Compute path
theta = bspline_curve(s,sol.theta.P,0,sol.theta.d,bspline_knots(1,sol.theta.N,sol.theta.d));
theta_p = bspline_curve(s,sol.theta.P,1,sol.theta.d,bspline_knots(1,sol.theta.N,sol.theta.d));
theta_pp = bspline_curve(s,sol.theta.P,2,sol.theta.d,bspline_knots(1,sol.theta.N,sol.theta.d));

%% Compute state
x(1:2) = theta;
x(3) = vecnorm(theta_p).*s_dot;
x(4) = atan2(theta_p(2,:),theta_p(1,:));
psi_p = (theta_pp(2,:).*theta_p(1,:) - theta_pp(1,:).*theta_p(2,:))...
        ./vecnorm(theta_p).^2;
u(1) = s_ddot.*vecnorm(theta_p) + s_dot.^2.*(dot(theta_p,theta_pp))./vecnorm(theta_p);
u(2) = s_dot.*psi_p; % psi_dot 
u(3) = atan2(sol.L.*psi_p,vecnorm(theta_p)); % gamma
