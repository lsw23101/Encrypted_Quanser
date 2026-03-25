%% Aero 2 — 3rd Order SISO: Pitch-Only Model
%
% Ref: Guillem Pereda Pérez Master's Thesis, §7
%
% 1-DOF pitch control. Rotor speed (Omega) kept as explicit state.
%
% State:  x = [theta_p,  dtheta_p,  Omega]'
%   theta_p  : pitch angle (rad),  0 = horizontal, positive = nose up
%   dtheta_p : pitch angular rate (rad/s)
%   Omega    : average rotor speed (rad/s)  Omega = (Omega0 + Omega1) / 2
%
% Input:  u = V_m  (average motor voltage, V)   V_m = (V0 + V1) / 2
% Output: y = theta_p
%
% Equations of motion (linearised about theta_p = 0):
%   Jp * ddtheta_p = 2*Dt*kt_p*Omega - Mb*g*Dm*theta_p - Dp*dtheta_p
%   Jeq * dOmega   = (kt/Rm)*V_m - (km^2/Rm + kd)*Omega

%% ── Parameters ──────────────────────────────────────────────────────────
% Body
Dt   = 0.16743;         % Pivot → rotor centre (m)
Mb   = 1.07;            % Body mass (kg)
Dm   = 2.7e-3;          % COM below pivot (m)
Jp   = 0.0231;          % Pitch-axis inertia (kg-m^2)
Dp   = 0.008;           % Pitch damping (N-m-s/rad)
kt_p = 0.0006;          % Thrust coeff  F = kt_p * Omega  (N-s/rad)
g    = 9.81;            % (m/s^2)

% Motor / Rotor
Rm    = 8.4;
kt    = 0.042;
km    = 0.042;
kd    = 8e-5;
mh    = 3.0e-3;  rh = 4.5e-3;
Jr    = 4.0e-6;  Jprop = 4.0e-5;
Jh    = 0.5*mh*rh^2;
Jeq   = Jr + Jprop + Jh;   % Per-rotor equivalent inertia (kg-m^2)

%% ── State-Space ──────────────────────────────────────────────────────────
%
%  [dtheta_p  ] = [0,           1,               0             ][theta_p ]   [0            ]
%  [ddtheta_p ] = [-Mb*g*Dm/Jp, -Dp/Jp,          2*Dt*kt_p/Jp ][dtheta_p] + [0            ] u
%  [dOmega    ]   [0,           0,               -tau_r^-1    ][Omega   ]   [kt/(Rm*Jeq)  ]
%
%  where tau_r = Jeq / (km^2/Rm + kd)  is the rotor time constant

tau_r_inv = (km^2/Rm + kd) / Jeq;   % rotor pole magnitude

A = [ 0,             1,              0;
     -Mb*g*Dm/Jp,   -Dp/Jp,         2*Dt*kt_p/Jp;
      0,             0,             -tau_r_inv];

B = [0;
     0;
     kt/(Rm*Jeq)];

C = [1, 0, 0];   % output: pitch angle
D = 0;

sys3 = ss(A, B, C, D, ...
    'StateName',  {'theta_p','dtheta_p','Omega'}, ...
    'InputName',  {'V_avg'}, ...
    'OutputName', {'theta_p'});

%% ── Analysis ─────────────────────────────────────────────────────────────
fprintf('=== 3rd-order SISO pitch model ===\n')
disp('A ='); disp(A)
disp('B ='); disp(B)

p = pole(sys3);
fprintf('Poles:\n'); disp(p)
fprintf('Controllability rank: %d/3\n', rank(ctrb(A,B)))
fprintf('Observability  rank: %d/3\n', rank(obsv(A,C)))

%% ── LQR Full-State Feedback ──────────────────────────────────────────────
% Minimise J = integral( x'Qx + u'Ru ) dt
Q3 = diag([100, 1, 0.1]);   % penalise theta_p most
R3 = 1;

[K3, ~, ~] = lqr(A, B, Q3, R3);
fprintf('\nLQR gain K (1x3): [Kp_theta  Kp_dtheta  Kp_Omega]\n')
disp(K3)

% Closed-loop
Acl3 = A - B*K3;
pcl3 = eig(Acl3);
fprintf('Closed-loop poles:\n'); disp(pcl3)

%% ── Reference pre-filter ─────────────────────────────────────────────────
% DC gain of (C*(sI-Acl)^-1*B_r) should = 1  =>  Nr = 1/dcgain
% For step tracking of theta_p:
Nr3 = -1 / (C * (Acl3 \ B));
fprintf('Pre-filter gain Nr = %.4f\n', Nr3)


%% ── Discrete-Time LQR Full-State Feedback ────────────────────────────────
Ts = 0.02;          % 50 Hz sample rate

sys_c = ss(A, B, C, D)


ds_sys = c2d(sys_c, Ts, 'zoh');
[Ad, Bd, Cd, Dd] = ssdata(ds_sys);

% LQR weights  (penalise pitch & yaw angles most)
Q = diag([100, 0, 0]);
R = eye(1);

[K, ~, cl_poles] = dlqr(Ad, Bd, Q, R);
fprintf('\nLQR gain K (2x4):\n'); disp(K)
fprintf('Closed-loop poles (dlqr):\n'); disp(cl_poles)

%% ── Observer (Luenberger) ────────────────────────────────────────────────
obs_poles = [0.71, 0.72, 0.73];
L = place(Ad', Cd', obs_poles).';
fprintf('Observer gain L (4x2):\n'); disp(L)

%% ── Output-Feedback Controller Matrices (F, G, H) ───────────────────────
%  x_c(k+1) = F*x_c(k) + G*y(k)
%  u(k)     = H*x_c(k)        (note: u = -K*x_hat, H = -K)
F_ctrl = Ad - Bd*K - L*Cd;
G_ctrl = L;
H_ctrl = -K;

fprintf('\nController (F, G, H):\n')
disp('F ='); disp(F_ctrl)
disp('G ='); disp(G_ctrl)
disp('H ='); disp(H_ctrl)



