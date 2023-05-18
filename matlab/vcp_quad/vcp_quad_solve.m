function [sol, err] = vcp_quad_solve(socp,data)
%VCP_QUAD_SOLVE  Solve the Quadcopter Trajectory Generation Problem.
%   [sol,err] = VCP_QUAD_SOLVE(socp,data) returns the parameters of the
%                   safe B-spline trajectory. 
%                   Infeasibility is described in err code as per YALMIP.
%                   The inputs are the compiled SOCP problem in the socp
%                   struct, and the problem data structure.
%
%   see vcp_quad_data() and vcp_quad_compile()
%
%
%   Copyright (c) 2022, University of Wisconsin-Madison

%% Options
debug = false; % Switch to true to return extra data in sol struct


%% Struct to return
sol.t_f = data.t_f;
sol.d = socp.d;
sol.N = socp.N;
sol.g = socp.g;
sol.v_max = data.v_max;
sol.epsilon = data.epsilon;
sol.T_lb = data.T_lb;
sol.T_ub = data.T_ub;
sol.omega_max = data.omega_max;

%% Check data

%% Get constraint code from data (VATO)
V_act = logical(data.v_max);
A_act = logical(data.epsilon);
T_act = logical(data.T_ub);
O_act = logical(data.omega_max);
sol.code = num2str([V_act, A_act, T_act, O_act]);
cst_code = bin2dec(sol.code);

%% Compute parameters
tau = bspline_knots(data.t_f,socp.N,socp.d);
% Objective
t = linspace(0,data.t_f,socp.I);
Lam_obj = bspline_lamvec(t,socp.r_obj,socp.d,tau);
B_r = bspline_bmat(socp.r_obj,socp.d,tau);
BLam = B_r*Lam_obj;
% socps
B_1 = bspline_bmat(1,socp.d,tau);
B_2 = bspline_bmat(2,socp.d,tau);
B_3 = bspline_bmat(3,socp.d,tau);
% Euler Ext
z_B_0 = [cos(data.x_0(4))*sin(data.x_0(5)); 
         -sin(data.x_0(4));
         cos(data.x_0(4))*cos(data.x_0(5))];
z_B_f = [cos(data.x_f(4))*sin(data.x_f(5)); 
         -sin(data.x_f(4));
         cos(data.x_f(4))*cos(data.x_f(5))];
TzB = [data.T_0*z_B_0, data.T_f*z_B_f];
% Flight Volume
fvol = [data.r_lb, data.r_ub];
% Waypoints
t_wp = linspace(0,data.t_f,size(data.P_wp,2)+2);
t_wp = t_wp(2:end-1);
Lam_wp = bspline_lamvec(t_wp,0,socp.d,tau);
% Max velocity
v_max = data.v_max;
% Angular bounds
cot_eps = cot(data.epsilon);
% Thrust bounds
T_b = [data.T_lb, data.T_ub];
% Angular velocity bounds
omega_max = data.omega_max;

%% Solve FLAT-VCP: Safe B-spline Trajectory
tic
[sol_socp, err] = socp.opts{cst_code+1}(data.x_0,data.x_f, ...
                    BLam,B_1,B_2,B_3,TzB, fvol, ... % Objective and fvol
                    data.d_wp,data.P_wp,Lam_wp, ... % Waypoints
                    v_max, cot_eps, T_b, omega_max); % Optional cst
sol.solvetime = toc;
[sol.P, sol.zeta] =  sol_socp{:};

if err ~= 0
    return % Exit if not feasible
end

% Process the solution of FLAT-VCP
if debug
    sol.s.kappa_bar = kappa_bar;
    sol.s.epsilon_bar = epsilon_bar;
end