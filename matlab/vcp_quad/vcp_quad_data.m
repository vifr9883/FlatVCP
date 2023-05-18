function data = vcp_quad_data()
%VCP_QUAD_DATA  Generates a default structure with relevant data.
%
%   data = VCP_QUAD_DATA() returns the struct with fields:
%       - t_f [1,1] Trajectory duration [s]
%       - x_0 [9,1] Initial state [m, rad, m/s]
%       - x_f [9,1] Final state [m, rad, m/s]
%       - T_0 [1,1] Initial thrust [m/s^2]
%       - T_f [1,1] Final thrust [m/s^2]
%       - d_wp [1,1] Waypoint allowed distance [m]
%       - P_wp [3,n_wp] Waypoints [p_1, ..., p_n_wp] (n_wp must match socp)
%       - v_max [1,1] Maximum linear speed [m/s]   (Set 0 to DISABLE)
%       - epsilon [1,1] Roll and pitch bound [rad] (Set 0 to DISABLE)
%       - T_lb [1,1] Thrust lower bound in [m/s^2] (Set 0 to DISABLE)
%       - T_ub [1,1] Thrust upper bound in [m/s^2] (Set 0 to DISABLE)
%       - omega_max [1,1] Max roll, pitch  [rad/s] (Set 0 to DISABLE)
%
%   Copyright (c) 2022, University of Wisconsin-Madison

data.t_f = 10; % Trajectory duration
data.x_0 = zeros(9,1); % Initial state
data.x_f = zeros(9,1); % Final state
data.T_0 = 9.81; % Initial thrust
data.T_f = 9.81; % Final thrust
data.r_lb = [-1.5;-1.5;0]; % Position box lower bounds
data.r_ub = [1.5;1.5;1.5]; % Position box upper bounds
data.d_wp = 0.05; % Waypoint allowed distance
data.P_wp = [0.5, -0.5,  0.5, -0.5;  % x
             0.5,  0.5, -0.5, -0.5;  % y
             0.7,  0.7,  0.7,  0.7]; % z
data.v_max = 1; % Maximum linear speed
data.epsilon = deg2rad(30); % Angular bound
data.T_lb = 0; % Thrust lower bound
data.T_ub = 2*9.81; % Thrust upper bound
data.omega_max = deg2rad(200); % Maximum angular velocity