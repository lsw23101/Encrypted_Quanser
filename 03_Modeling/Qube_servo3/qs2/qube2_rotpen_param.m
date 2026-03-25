%% Motor
% Resistance
% Rm = 8.4; % qs2
Rm = 7.5;  % qs3
% Current-torque (N-m/A)
kt = 0.042;
% Back-emf constant (V-s/rad)
km = 0.042;
%
%% Rotary Arm
% Mass (kg)
Mr = 0.095;
% Total length (m)
Lr = 0.085;
% Moment of inertia about pivot (kg-m^2)
Jr = Mr*Lr^2/12; % qs2
% Jr = Mr*Lr^2/3; % qs3
% Equivalent Viscous Damping Coefficient (N-m-s/rad)
% Dr = 0; % qs2
Dr = 1e-3; % qs3
%
%% Pendulum Link
% Mass (kg)
Mp = 0.024;
% Total length (m)
Lp = 0.129;
% Moment of inertia about pivot (kg-m^2)
Jp = Mp*Lp^2/12; % qs2
% Jp = Mp*Lp^2/3; % qs3

% Equivalent Viscous Damping Coefficient (N-m-s/rad)
% Dp = 0; % qs2 
Dp = 5e-5; % qs3
% Gravity Constant
g = 9.81;
