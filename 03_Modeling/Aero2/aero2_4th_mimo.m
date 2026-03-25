%% Aero 2 — 4th-Order MIMO: Full Nonlinear + Feedback Linearization
%
% Ref: Guillem Pereda Pérez Master's Thesis, §7 (Eq. 7.5 – 7.6)
%
% 2-DOF pitch/yaw helicopter.
%
% State:  x = [theta_p,  theta_y,  dtheta_p,  dtheta_y]'
%   theta_p  : pitch angle  (rad)   — 0 = horizontal, positive = nose up
%   theta_y  : yaw angle    (rad)
%   dtheta_p : pitch rate   (rad/s)
%   dtheta_y : yaw rate     (rad/s)
%
% Input:  u = [V0, V1]'   (motor voltages, V)
%   Motor 0: CCW (from above) → positive thrust, negative drag torque on body
%   Motor 1: CW  (from above) → positive thrust, positive drag torque on body
%
% Output: y = [theta_p, theta_y]'
%
% Quasi-static rotor assumption:  Omega_i = Kv * V_i
%   Kv = kt / (km^2 + kd*Rm)
%
% =========================================================================
%  NONLINEAR EQUATIONS OF MOTION  (Lagrangian, Eq. 7.5)
% =========================================================================
%
%   Jp * ddtheta_p = Dt*kt_p*(Omega0+Omega1)
%                   - Mb*g*Dm*sin(theta_p)
%                   - Dp*dtheta_p
%                   + Jy*sin(theta_p)*cos(theta_p)*dtheta_y^2
%
%   Jy * ddtheta_y = kd*(Omega1-Omega0)*cos(theta_p)
%                   - Dy*dtheta_y
%                   - 2*Jy*sin(theta_p)*cos(theta_p)*dtheta_p*dtheta_y
%
% =========================================================================
%  FEEDBACK LINEARIZATION  (Eq. 7.6)
% =========================================================================
%
%  Choose auxiliary inputs v_p, v_y (new "virtual" accelerations), then:
%
%   (Omega0+Omega1) = (1 / (Dt*kt_p)) * [Jp*v_p
%                     + Mb*g*Dm*sin(theta_p)
%                     + Dp*dtheta_p
%                     - Jy*sin(theta_p)*cos(theta_p)*dtheta_y^2]
%
%   (Omega1-Omega0) = (1 / (kd*cos(theta_p))) * [Jy*v_y
%                     + Dy*dtheta_y
%                     + 2*Jy*sin(theta_p)*cos(theta_p)*dtheta_p*dtheta_y]
%
%  After cancellation:
%   ddtheta_p = v_p    (decoupled double integrator)
%   ddtheta_y = v_y    (decoupled double integrator)
%
%  Motor voltages from sum/diff:
%   V0 = (Omega_sum - Omega_diff) / (2*Kv)
%   V1 = (Omega_sum + Omega_diff) / (2*Kv)
%
% =========================================================================

%% ── Parameters ──────────────────────────────────────────────────────────
% Body
Dt   = 0.16743;         % Pivot → rotor centre (m)
Mb   = 1.07;            % Body mass (kg)
Dm   = 2.7e-3;          % COM below pivot (m)
Jp   = 0.0231;          % Pitch-axis inertia (kg-m^2)
Jy   = 0.0238;          % Yaw-axis inertia   (kg-m^2)
Dp   = 0.008;           % Pitch damping (N-m-s/rad)
Dy   = 0;               % Yaw   damping (N-m-s/rad)  [assumed negligible]
kt_p = 0.0006;          % Thrust coeff  F = kt_p * Omega  (N-s/rad)
g    = 9.81;            % (m/s^2)

% Motor / Rotor  (quasi-static)
Rm   = 8.4;
kt   = 0.042;
km   = 0.042;
kd   = 8e-5;
mh   = 3.0e-3;  rh = 4.5e-3;
Jr   = 4.0e-6;  Jprop = 4.0e-5;
Jh   = 0.5*mh*rh^2;
Jeq  = Jr + Jprop + Jh;   % Per-rotor equivalent inertia (kg-m^2) [kept for reference]

% Quasi-static rotor gain  Omega_ss / V  (rad/s per V)
Kv = kt / (km^2 + kd*Rm);
fprintf('Kv = %.4f rad/s/V\n', Kv)

%% ── Linearised State-Space  (about theta_p=0, dtheta_p=0, dtheta_y=0) ──
%
%  sin(theta_p) ≈ theta_p,  cos(theta_p) ≈ 1,  quadratic coupling → 0
%
%  ddtheta_p = -Mb*g*Dm/Jp * theta_p  -  Dp/Jp * dtheta_p
%              + Dt*kt_p*Kv/Jp * (V0+V1)
%
%  ddtheta_y = -Dy/Jy * dtheta_y  +  kd*Kv/Jy * (V1-V0)
%
%  x = [theta_p, theta_y, dtheta_p, dtheta_y]'
%  u = [V0, V1]'

A = [0,           0,      1,          0;
     0,           0,      0,          1;
    -Mb*g*Dm/Jp,  0,     -Dp/Jp,      0;
     0,           0,      0,         -Dy/Jy];
%      theta_p   theta_y  dtheta_p  dtheta_y

%  Column 1 = V0 effect:  positive pitch (sum),  negative yaw (CCW drag)
%  Column 2 = V1 effect:  positive pitch (sum),  positive yaw (CW  drag)
B = [0,                  0;
     0,                  0;
     Dt*kt_p*Kv/Jp,      Dt*kt_p*Kv/Jp;
    -kd*Kv/Jy,           kd*Kv/Jy];
%    V0                   V1

C = [1, 0, 0, 0;
     0, 1, 0, 0];

D = zeros(2, 2);

sys_c = ss(A, B, C, D, ...
    'StateName',  {'theta_p','theta_y','dtheta_p','dtheta_y'}, ...
    'InputName',  {'V0','V1'}, ...
    'OutputName', {'theta_p','theta_y'});

%% ── Analysis ─────────────────────────────────────────────────────────────
fprintf('\n=== 4th-Order MIMO (quasi-static rotor, linearized) ===\n')
disp('A ='); disp(A)
disp('B ='); disp(B)

p = pole(sys_c);
fprintf('Open-loop poles:\n'); disp(p)

fprintf('Controllability rank: %d/4\n', rank(ctrb(A,B)))
fprintf('Observability  rank: %d/4\n', rank(obsv(A,C)))

%% ── Decoupled input interpretation ──────────────────────────────────────
%  u_pitch = V0+V1  →  pitch only (sum)
%  u_yaw   = V1-V0  →  yaw   only (diff)
%  [V0; V1] = 0.5*[1,-1; 1,1] * [u_pitch; u_yaw]
T_mix = 0.5 * [1, -1; 1, 1];
B_dec = B * T_mix;
fprintf('\nDecoupled B (columns = u_pitch, u_yaw):\n'); disp(B_dec)

%% ── Discrete-Time LQR Full-State Feedback ────────────────────────────────
Ts = 0.02;          % 50 Hz sample rate
ds_sys = c2d(sys_c, Ts, 'zoh');
[Ad, Bd, Cd, Dd] = ssdata(ds_sys);

% LQR weights  (penalise pitch & yaw angles most)
Q = diag([1000, 1000, 0, 0]);
R = eye(2);

[K, ~, cl_poles] = dlqr(Ad, Bd, Q, R);
fprintf('\nLQR gain K (2x4):\n'); disp(K)
fprintf('Closed-loop poles (dlqr):\n'); disp(cl_poles)

%% ── Observer (Luenberger) ────────────────────────────────────────────────
obs_poles = [0.71, 0.72, 0.73, 0.74];
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
