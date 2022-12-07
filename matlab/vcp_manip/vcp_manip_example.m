clear; clc; close all;

% Parameters
Ts = 0.1; % [sec]

% Get data
data = vcp_manip_data();
% Modify data
data.n = 4;
data.l = [0.5; 0.2; 0.35; 0.1]; % Link lengths
data.tf = 10; % Final time
% Extremes
% th_0 = deg2rad([25;-10;-5;30]); % Initial joing config
% th_f = deg2rad([10;100;0;45]); % Final joing config
th_0 = deg2rad([-50;45;-45;-25]); % Initial joing config
th_f = deg2rad([40;45;-40;90]); % Final joing config
data.x_0 = [vcp_manip_joint(th_0, data.n, data); th_0]; % Initial state
data.x_f = [vcp_manip_joint(th_f, data.n, data); th_f]; % Final state
% Constraints
data.th_lb = min(th_0, th_f); % theta lower-bound vector
data.th_ub = max(th_0, th_f); % theta upper-bound vector
data.u_lim = deg2rad(2*data.tf); % Actuation effor limit


% Solve FQP
sol = vcp_manip_solve(data);
assert(~sol.problem, strcat("Infeasible FQP: ",num2str(sol.problem)))

% Inflate
[x, u, t] = vcp_manip_inflate(data, sol, Ts);

% Plot
vcp_manip_plot(1,"end",data,'b',x,u,t)
vcp_manip_plot(2,"theta",data,'b',x,u,t)
vcp_manip_plot(3,"input",data,'b',x,u,t)