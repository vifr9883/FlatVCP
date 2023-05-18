function vcp = vcp_quad_socp()
%VCP_QUAD_CSOCP  Yalmip Optimizer for QUAD-SOCP.
%   vcp = VCP_QUAD_SOCP() returns a struct with a YALMIP optimizer
%   object modeling QUAD-SOCP.
%
%   The struct has fields:
%   - d [1,1] B-spline degree
%   - N [1,1] Number of control points (N+1)
%   - I [1,1] Integral approximation resolution
%   - r_obj [1,1] Order of the objective function (i.e. 3 = min jerk)
%   - g [1,1] Gravity acceleration [m/s^2]
%   - n_wp [1,1] Number of waypoints
%   - opt [] optimizer object
%
%   The optimizer object is [vcp.opt] and it has:
%   Inputs:
%   - x_0 [9,1] initial state
%   - x_f [9,1] final state
%   - BLam [N+1,I] weight for objective function
%   - B_1 [N+1,N+2] B_1 matrix
%   - B_2 [N+1,N+3] B_2 matrix
%   - B_3 [N+1,N+4] B_3 matrix
%   - TzB [3,2] is [T_0*z_B_0, T_f*z_B_f] for eul init/final
%   - fvol [3,2] is [r_lb, r_ub] flight volume bounds in [m]
%   - d_wp [1,1] Waypoint distance in [m]
%   - P_wp [3,n_wp] Waypoints [p_1, ..., p_n_wp] in [m]
%   - Lam_wp [N+1,n_wp] Lambda for each wp [Lam_1, ..., Lam_n_wp]
%   - v_max [1,1] Maximum linear speed [m/s]
%   - cot_eps [1,1] cot(epsilon) for angular bounds
%   - T_b [1,2] is thrust bounds [T_lb, T_ub] in [m/s^2]
%   - omega_max [1,1] Maximum angular velocity in [rad/s]
%
%   Outputs:
%   - P [1,N+1] control points
%
%   Copyright (c) 2022, University of Wisconsin-Madison

%% Setup Problem
vcp.d = 5; % B-spline degree (4 is min)
vcp.N = 25; % Control Points
vcp.I = 25; % Integral approximation resolution
vcp.r_obj = 4; % Objective function derivative order
vcp.g = 9.81; % Gravity acceleration [m/s^2]
vcp.n_wp = 4; % Number of waypoints


% Parameters
x_0 = sdpvar(9,1); % Initial state
x_f = sdpvar(9,1); % Final state
BLam = sdpvar(vcp.N+1,vcp.I,'full'); % Objective function weight
B_1 = sdpvar(vcp.N+1,vcp.N+2,'full'); % B_1 matrix
B_2 = sdpvar(vcp.N+1,vcp.N+3,'full'); % B_2 matrix
B_3 = sdpvar(vcp.N+1,vcp.N+4,'full'); % B_3 matrix
TzB = sdpvar(3,2,'full'); % T_q*z_B_q for initial/final eul
fvol = sdpvar(3,2,'full'); % [r_lb, r_ub] flight volume box
d_wp = sdpvar(1,1); % Waypoint distance in [m]
P_wp = sdpvar(3,vcp.n_wp,'full'); % [p_1, ..., p_n_wp] in [m]
Lam_wp = sdpvar(vcp.N+1,vcp.n_wp,'full'); % [Lam_1, ..., Lam_n_wp]
v_max = sdpvar(1,1); % Maximum speed in [m/s]
cot_eps = sdpvar(1,1); % cot(epsilon) for angular bounds
T_b = sdpvar(1,2,'full'); % [T_lb, T_ub] in [m/s^2] thrust bounds
omega_max = sdpvar(1,1); % Maximum angular velocity in [rad/s]


% Variables
P = sdpvar(3,vcp.N+1); % Control Points
P_1 = sdpvar(3,vcp.N+2); % 1-st order VCPs
P_2 = sdpvar(3,vcp.N+3); % 2-nd order VCPs
P_3 = sdpvar(3,vcp.N+4); % 3-rd order VCPs

% Init problem
obj = 0;
cst = [];

%% Sparsity
% Sample data with right sparsity structure
tau = bspline_knots(1,vcp.N,vcp.d);
t = linspace(0,1,vcp.I);
Lam_obj_d = bspline_lamvec(t,vcp.r_obj,vcp.d,tau);
B_r_d = bspline_bmat(vcp.r_obj,vcp.d,tau);
B_1_d = bspline_bmat(1,vcp.d,tau);
B_2_d = bspline_bmat(2,vcp.d,tau);
B_3_d = bspline_bmat(3,vcp.d,tau);
BLam_d = B_r_d*Lam_obj_d;
t_wp = linspace(0,1,vcp.n_wp+2);
t_wp = t_wp(2:end-1);
Lam_wp_d = bspline_lamvec(t_wp,0,vcp.d,tau);

% Make parameters sparse
BLam = BLam.*(BLam_d ~= 0);
B_1 = B_1.*(B_1_d ~= 0);
B_2 = B_2.*(B_2_d ~= 0);
B_3 = B_3.*(B_3_d ~= 0);
Lam_wp = Lam_wp.*(Lam_wp_d ~= 0);

%% Objective
costvar_sig = sdpvar(3,vcp.I); % i.e. \dddot{\sigma}(t)
costvar_epi = sdpvar(1,vcp.I); % Epigraph variables
cst = [cst, costvar_sig == P*BLam];
cst = [cst, cone([(costvar_epi+1)/sqrt(2); ...
    [(costvar_epi-1)/sqrt(2);sqrt(2)*costvar_sig]])];
obj = obj + sum(costvar_epi);

%% Constraints
% VCPs
cst = [cst, P_1 == P*B_1];
cst = [cst, P_2 == P*B_2];
cst = [cst, P_3 == P*B_3];

% Initial/Final States
cst = [cst, P(:,1) == x_0(1:3)]; % sig(0) = r_0
cst = [cst, P(:,vcp.N+1) == x_f(1:3)]; % sig(t_f) = r_f
cst = [cst, P_1(:,2) == x_0(7:9)]; % \dot{sig}(0) = \dot{r}_0
cst = [cst, P_1(:,vcp.N+1) == x_f(7:9)]; % \dot{sig}(t_f) = \dot{r}_f
cst = [cst, P_2(:,3) == TzB(:,1) - vcp.g*[0;0;1]];  % \xi_0
cst = [cst, P_2(:,vcp.N+1) == TzB(:,2) - vcp.g*[0;0;1]];  % \xi_f

% Flight volume constraints
cst_fvol = [];
for j = 1:vcp.N
  cst_fvol = [cst_fvol, fvol(:,1) <= P(:,j+1) <= fvol(:,2)];
end

% Waypoint constraints
cst_wp = [];
for i = 1:vcp.n_wp
  cst_wp = [cst_wp, cone(P_wp(:,i)-P*Lam_wp(:,i), d_wp)];
end

% Linear velocity constraints
cst_vel = [];
for j = 1:vcp.N
  cst_vel = [cst_vel, cone(P_1(:,j+1),v_max)];
end

% Angular constraints
cst_ang = [];
A_eps = cot_eps*blkdiag(eye(2),0);
for j = 2:vcp.N
  cst_ang = [cst_ang, cone(A_eps*P_2(:,j+1),vcp.g + P_2(3,j+1))];
end

% Thrust constraints
cst_thrust = [];
for j = 2:vcp.N
  cst_thrust = [cst_thrust,cone(P_2(:,j+1)+vcp.g*[0;0;1],T_b(1,2))]; % T_ub
  cst_thrust = [cst_thrust, P_2(3,j+1) >= T_b(1,1)-vcp.g]; % T_lb
end

% Angular velocity constraints
cst_dang = [];
zeta = sdpvar(vcp.N-vcp.d+1,1,'full');
obj_dang = - sum(zeta);
cst_dang = [cst_dang, zeta >= 0];
for l = vcp.d:vcp.N
  for j = l - vcp.d + 2:l
    cst_dang = [cst_dang, P_2(3,j+1) >= zeta(l-vcp.d+1) - vcp.g];
  end
  for j = l - vcp.d + 3:l
    cst_dang = [cst_dang, cone(P_3(:,j+1),omega_max*zeta(l-vcp.d+1))];
  end
end


%% Options
ops = sdpsettings('verbose',0,'solver','mosek','debug',0);

%% Setup Optimizers
% Constraint and Objective combinations
cst = [cst, cst_fvol]; % fvol is always ON
if vcp.n_wp
  cst = [cst, cst_wp]; % Waypoint constraints are decided at compile-time
  par = {x_0, x_f, BLam, B_1, B_2, B_3, TzB, fvol, ...
          d_wp, P_wp, Lam_wp, v_max, cot_eps, T_b, omega_max};
else
  par = {x_0, x_f, BLam, B_1, B_2, B_3, TzB, fvol, ...
          d_wp, sdpvar(1,1), sdpvar(1,1), v_max, cot_eps, T_b, omega_max};
end
out = {P,zeta};


% We have now 16 combinations of: VATO
% V: cst_vel
% A: cst_ang
% T: cst_thrust
% O: cst_dang
vcp.opts = cell(16,1);
% 0000: 0 ----
vcp.opts{1} = optimizer(cst,obj,ops,par,out);
% 0001: 1 ---O
vcp.opts{2} = optimizer([cst, cst_dang], obj+obj_dang,ops,par,out);
% 0010: 2 --T-
vcp.opts{3} = optimizer([cst, cst_thrust], obj,ops,par,out);
% 0011: 3 --TO
vcp.opts{4} = optimizer([cst, cst_thrust, cst_dang], obj+obj_dang,ops,par,out);
% 0100: 4 -A--
vcp.opts{5} = optimizer([cst, cst_ang],obj,ops,par,out);
% 0101: 5 -A-O
vcp.opts{6} = optimizer([cst, cst_ang, cst_dang],obj+obj_dang,ops,par,out);
% 0110: 6 -AT-
vcp.opts{7} = optimizer([cst, cst_ang, cst_thrust],obj,ops,par,out);
% 0111: 7 -ATO
vcp.opts{8} = optimizer([cst, cst_ang, cst_thrust, cst_dang],obj+obj_dang,ops,par,out);
% 1000: 8 V---
vcp.opts{9} = optimizer([cst, cst_vel],obj,ops,par,out);
% 1001: 9 V--O
vcp.opts{10} = optimizer([cst, cst_vel, cst_dang],obj+obj_dang,ops,par,out);
% 1010: 10 V-T-
vcp.opts{11} = optimizer([cst, cst_vel, cst_thrust],obj,ops,par,out);
% 1011: 11 V-TO
vcp.opts{12} = optimizer([cst, cst_vel, cst_thrust, cst_dang], obj+obj_dang,ops,par,out);
% 1100: 12 VA--
vcp.opts{13} = optimizer([cst, cst_vel, cst_ang], obj,ops,par,out);
% 1101: 13 VA-T
vcp.opts{14} = optimizer([cst, cst_vel, cst_ang, cst_dang],obj+obj_dang,ops,par,out);
% 1110: 14 VAT-
vcp.opts{15} = optimizer([cst, cst_vel, cst_ang, cst_thrust],obj,ops,par,out);
% 1111: 15 VATO
vcp.opts{16} = optimizer([cst, cst_vel, cst_ang, cst_thrust, cst_dang],obj+obj_dang,ops,par,out);