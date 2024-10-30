function in = pfcResetFcn(in) 
% Reset function for PFC multi-agent RL example

% Copyright 2020 The MathWorks, Inc.
in = setVariable(in,'x0_lead',40+randi(60,1,1));    % random value for initial position of lead car
in = setVariable(in,'e1_initial', 0.5*(-1+2*rand)); % random value for lateral deviation
in = setVariable(in,'e2_initial', 0.1*(-1+2*rand)); % random value for relative yaw angle
end