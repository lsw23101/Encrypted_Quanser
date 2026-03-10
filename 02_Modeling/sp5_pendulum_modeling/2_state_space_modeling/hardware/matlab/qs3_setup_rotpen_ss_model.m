% Load QUBE-Servo 3 model parameters
qs3_rotpen_param;
% Load state-space model for pendulum in downward configuration
qs3_rotpen_ABCD_eqns_down;
% Create Matlab state-space object for analysis
sys = ss(A,B,C,D);
