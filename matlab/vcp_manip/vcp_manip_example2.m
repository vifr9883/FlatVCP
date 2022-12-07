clear; clc; close all;

% Parameters
Ts = 0.1; % [sec]

% Get data
data = vcp_manip_data();
% Modify data
data.n = 8;
data.l = [1; 0.4; 0.3; 0.5; 0.6; 0.4; 0.25; 0.4]; % Link lengths
data.tf = 10; % Final time
% Extremes
% th_0 = deg2rad([-40; 20; -45; -25; -10; -20; 40; 50]); % Initial joing config
% th_f = deg2rad([40; 45; -40; 50; 20; -20; 10 ; -25]); % Final joing config
th_0 = deg2rad([0; 20; -45; 45; -45; -20; 40; 10]); % Initial joing config
th_f = deg2rad([20; 45; -40; -50; -70; -20; 10 ; -25]); % Final joing config
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