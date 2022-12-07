function [x, u, t] = vcp_manip_inflate(data, sol, Ts)
%VCP_MANIP_INFLATE  Produce state x and input u trajectories from FlatVCP sol.
%   [x, u, t] = VCP_MANIP_INFLATE(data, sol,Ts) returns x, u state and input
%                   trajectories with a sample time of Ts. The state
%                   trajectory x is [n+2, N] and the input trajectory u is
%                   [n, N]. The number N is determined as floor(t_f/Ts).
%                   A sequence of times t [1,N] is also returned.
%
%   see vcp_manip_data() and vcp_manip_solve()
%
%
%   Copyright (c) 2022, University of Colorado Boulder

%% Matrices to return
N = max(1,floor(data.tf/Ts));
t = linspace(0,data.tf,N);
x = zeros(data.n+2,N);


%% Compute B-spline
theta = bspline_curve(t,sol.P,0,sol.d,bspline_knots(data.tf,sol.N,sol.d));
u = bspline_curve(t,sol.P,1,sol.d,bspline_knots(data.tf,sol.N,sol.d));

%% Compute state
x = [vcp_manip_joint(theta, data.n, data); theta];

