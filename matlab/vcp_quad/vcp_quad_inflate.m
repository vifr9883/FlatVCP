function [x, u, t] = vcp_quad_inflate(sol, Ts)
%VCP_QUAD_INFLATE  Produce state x and input u traj from FlatVCP sol.
%   [x, u, t] = VCP_QUAD_INFLATE(sol,Ts) returns x, u state and input
%                   trajectories with a sample time of Ts. The state
%                   trajectory x is [9, N] and the input trajectory u is
%                   [4, N]. The number N is determined as floor(t_f/Ts).
%                   A sequence of times t [1,N] is also returned.
%
%   see vcp_quad_data() and vcp_quad_solve()
%
%
%   Copyright (c) 2022, University of Wisconsin-Madison

%% Matrices to return
N = max(1,floor(sol.t_f/Ts));
t = linspace(0,sol.t_f,N);
x = zeros(9,N);
u = zeros(4,N);

%% Compute B-spline flat trajectory
sig = bspline_curve(t, sol.P, 0, sol.d, bspline_knots(sol.t_f,sol.N,sol.d));
sig_dot = bspline_curve(t, sol.P, 1, sol.d, bspline_knots(sol.t_f,sol.N,sol.d));
sig_ddot = bspline_curve(t, sol.P, 2, sol.d, bspline_knots(sol.t_f,sol.N,sol.d));
sig_dddot = bspline_curve(t, sol.P, 3, sol.d, bspline_knots(sol.t_f,sol.N,sol.d));

%% Compute state-space trajectory
x(1:3,:) = sig; % x y z
x(7:9,:) = sig_dot; % dx dy dz
u(1,:) = vecnorm(sig_ddot + sol.g*[0;0;1]); % Thrust
% Angular
z_B = (sig_ddot + sol.g*[0;0;1])./u(1,:);
y_C = [-sin(0);cos(0);0]*ones(1,length(t)); % Assume psi(t) = 0
x_B = cross(y_C,z_B)./vecnorm(cross(y_C,z_B));
y_B = cross(z_B,x_B);
x(5,:) = -asin(x_B(3,:)); % theta
x(4,:) = asin(y_B(3,:)./cos(x(5,:))); % phi
% Angular Velocity
h_Omega = (1./u(1,:)).*(sig_dddot - dot(z_B,sig_dddot).*z_B);
u(2,:) = -dot(y_B,h_Omega); % p
u(3,:) = dot(x_B,h_Omega); % q
u(4,:) = dot(z_B,zeros(1,length(t)).*[0;0;1]); % r ( Assume psi_dot(t) = 0 )  