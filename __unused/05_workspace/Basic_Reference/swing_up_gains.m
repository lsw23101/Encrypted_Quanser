%% Swing-Up Control
% Reference Energy (J)
Er = Mp*g*Lp;
% Maximum torque for 10 V
tau_max = kt*10/Rm;
% Maximum acceleration of pivot (m/s^2)
a_max = tau_max / (Mr*Lr);
%
