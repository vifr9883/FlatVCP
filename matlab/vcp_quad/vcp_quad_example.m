clear; clc; close all

%% Load compiled Yalmip optimizer
load("vcp_quad_compiled.mat","socp")

%% Load sample data
data = vcp_quad_data();
data.t_f = 10;
data.P_wp = [0.7, -0.7,  0.7, -0.7;  % x
             0.7,  0.7, -0.7, -0.7;  % y
             0.4,  0.5,  0.4,  0.5]; % z
data.T_ub = 20;
data.epsilon = deg2rad(15);
data.v_max = 1.5;
data.omega_max = deg2rad(200);


%% Solve problem
[sol, err] = vcp_quad_solve(socp,data);
disp(strcat("Solvetime: ",num2str(sol.solvetime)))
assert(~err,"Infeasible")

%% Recover State-space
[x, u, t] = vcp_quad_inflate(sol, 0.01);

%% Plot
col = 'b';
vcp_quad_plot(1,"3D",data,col,x,u,t);
vcp_quad_plot(2,"pos",data,col,x,u,t);
vcp_quad_plot(3,"eul",data,col,x,u,t);
vcp_quad_plot(4,"vel",data,col,x,u,t);
vcp_quad_plot(5,"inp",data,col,x,u,t);
vcp_quad_plot(6,"speed",data,col,x,u,t);
