%% Parameters used by PFC multi-agent RL example

% Copyright 2020 The MathWorks, Inc.
 
%%
m = 1600;           % total vehicle mass (kg)
Iz = 2875;          % yaw moment of inertia (mNs^2)
lf = 1.4;           % longitudinal distance from center of gravity to front tires (m)
lr = 1.6;           % longitudinal distance from center of gravity to rear tires (m)
Cf = 19000;         % cornering stiffness of front tires (N/rad)
Cr = 33000;         % cornering stiffness of rear tires (N/rad)
tau = 0.5;          % longitudinal time constant

x0_lead = 50;       % initial position for lead car (m)
v0_lead = 24;       % initial velocity for lead car (m/s)
x0_ego = 10;        % initial position for ego car (m)
v0_ego = 18;        % initial velocity for ego car (m/s)

D_default = 10;     % default spacing between lead and ego cars (m)
t_gap = 1.4;        % time gap for distance maintaining (s)
v_set = 28;         % set velovity for ego car (m/s)

amin_ego = -3;      % minimum acceleration for ego car (m/s^2)
amax_ego = 2;       % maximum acceleration for ego car (m/s^2)
umin_ego = -0.5;    % minimum steering angle (rad)
umax_ego = 0.5;     % maximum steering angle (rad)

rho = 0.001;        % curvature of road (1/m)
e1_initial = 0.2;   % initial lateral deviation (m)
e2_initial = -0.1;  % intitial yaw angle error (rad)