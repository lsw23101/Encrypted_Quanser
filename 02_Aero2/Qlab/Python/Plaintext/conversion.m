clear all;
clc;
close all;

%% 1. Parameters (From Table I of Fellag et al., 2024)
Jp = 0.0211;    % Pitch inertia (kg.m^2)
Jy = 0.0221;    % Yaw inertia (kg.m^2)
Dp = 0.0053;    % Pitch damping (N.m.s/rad)
Dy = 0.0062;    % Yaw damping (N.m.s/rad)
Mg = 0.0153;    % Gravity term (N.m)
Kpp =  0.0011;  Kyy =  0.0047;
Kpy =  0.0021;  Kyp = -0.0027;

%% 2. Continuous-Time Model & Discretization
A0 = [0,       0,  1,       0;
      0,       0,  0,       1;
     -Mg/Jp,   0, -Dp/Jp,   0;
      0,       0,  0,      -Dy/Jy];

B0 = [0,         0;
      0,         0;
      Kpp/Jp,    Kpy/Jp;
      Kyp/Jy,    Kyy/Jy];

C = [1, 0, 0, 0;
     0, 1, 0, 0];

Ts = 0.02;
sysD = c2d(ss(A0, B0, C, []), Ts);
A = sysD.A;
B = sysD.B;

[n, m] = size(B);
[l, ~] = size(C);

% (1) LQR Feedback Gain (K)
Q = diag([1500, 1500, 10, 10]); % Penalize angle errors heavily
R = eye(2) * 0.1;               % Minimal penalty on voltage
[K, ~, ~] = dlqr(A, B, Q, R);

% (2) Feedforward Precompensator (N_bar) for Tracking
% Ensures steady-state error is zero for a step reference.
% Formula: N_bar = [ C * (I - (A - B*K))^-1 * B ]^-1
N_bar = inv(C * inv(eye(4) - (A - B * K)) * B);

% (3) Luenberger Observer Gain (L)
obs_poles = [0.65, 0.66, 0.67, 0.68]; % Faster than system dynamics
L = place(A', C', obs_poles).';

%% 4. Observer-Based Controller Matrices (Tracking)
%
%   u(k)        = H  * xhat(k) + N_bar * r
%   xhat(k+1)   = F  * xhat(k) + G * y(k) + P * r
%
%   where P = B * N_bar  (feedforward correction in observer)

F = A - B*K - L*C;
G = L;
H = -K;

% Observer feedforward
P = B * N_bar;

fprintf('\n=== Standard Controller Matrices ===\n');
fprintf('F =\n'); disp(F);
fprintf('G =\n'); disp(G);
fprintf('H =\n'); disp(H);
fprintf('N_bar =\n'); disp(N_bar);
fprintf('P = B*N_bar =\n'); disp(P);

fprintf('\n=== Python (Standard) ===\n');
print_numpy('F', F, 4);
print_numpy('G', G, 4);
print_numpy('H', H, 4);
print_numpy('N_bar', N_bar, 4);
print_numpy('P', P, 4);

%% 5. Integer Matrix Conversion (CDSL approach)
%
% Add/subtract R*u to separate u from xhat dynamics:
%   xhat(k+1) = (F - R*H)*xhat + R*u + G*y + (P - R*N_bar)*r
%
% Transform z = T*xhat:
%   z(k+1) = F_*z + G_*y + R_*u + P_*r
%   u(k)   = H_*z + N_bar*r
%
%   F_ = T*(F-R*H)*T^{-1}   [integer matrix]
%   G_ = T*G
%   R_ = T*R
%   H_ = H*T^{-1}
%   P_ = T*(P - R*N_bar) = T*(B - R)*N_bar

% Find R s.t. (F - R*H) has integer eigenvalues
int_poles = [0, 1, 2, -1];
R_fb = place(F.', H.', int_poles).';

% Canonical transformation
sys_fgh = ss(F - R_fb*H, G, H, []);
[~, T] = canon(sys_fgh, 'companion');

F_ = T * (F - R_fb*H) / T;
G_ = T * G;
R_ = T * R_fb;
H_ = H / T;
P_ = T * (P - R_fb * N_bar);   % = T*(B - R_fb)*N_bar

fprintf('\n=== Transformed Matrices ===\n');
fprintf('F_ (should be integer) =\n'); disp(round(F_));
fprintf('R_ =\n'); disp(R_);
fprintf('G_ =\n'); disp(G_);
fprintf('H_ =\n'); disp(H_);
fprintf('P_ =\n'); disp(P_);
fprintf('Integer check — max rounding error: %.2e\n', max(max(abs(F_ - round(F_)))));

fprintf('\n=== Python (Transformed) ===\n');
print_numpy('F_', round(F_), 0);
print_numpy('G_', G_, 4);
print_numpy('R_', R_, 4);
print_numpy('H_', H_, 4);
print_numpy('P_', P_, 4);
print_numpy('N_bar', N_bar, 4);

%% 6. Simulation (Tracking: pitch=10deg, yaw=-10deg)
T_sim = 10;
N_sim = round(T_sim / Ts);
t     = (0:N_sim-1) * Ts;

ref  = [deg2rad(10); deg2rad(-10)];
VMAX = 15;

% --- Case 1: Standard (F, G, H, P, N_bar) ---
x_std    = zeros(n, N_sim);
xhat_std = zeros(n, N_sim);
u_std    = zeros(m, N_sim);

% --- Case 2: Transformed (F_, G_, R_, H_, P_, N_bar) ---
x_trans  = zeros(n, N_sim);
z_trans  = zeros(n, N_sim);   % z = T*xhat
u_trans  = zeros(m, N_sim);

for k = 1:N_sim-1

    % ── Standard ─────────────────────────────────────────────────────
    y_std        = C * x_std(:, k);
    u_std(:, k)  = H * xhat_std(:, k) + N_bar * ref;
    u_std(:, k)  = max(min(u_std(:, k), VMAX), -VMAX);

    x_std(:, k+1)    = A * x_std(:, k) + B * u_std(:, k);
    xhat_std(:, k+1) = F * xhat_std(:, k) + G * y_std + P * ref;

    % ── Transformed ──────────────────────────────────────────────────
    y_trans        = C * x_trans(:, k);
    u_trans(:, k)  = H_ * z_trans(:, k) + N_bar * ref;
    u_trans(:, k)  = max(min(u_trans(:, k), VMAX), -VMAX);

    x_trans(:, k+1)  = A * x_trans(:, k)  + B * u_trans(:, k);
    z_trans(:, k+1)  = F_ * z_trans(:, k) + G_ * y_trans ...
                       + R_ * u_trans(:, k) + P_ * ref;
end

% Last step control
u_std(:, end)   = H  * xhat_std(:, end) + N_bar * ref;
u_trans(:, end) = H_ * z_trans(:, end)  + N_bar * ref;

% Recover xhat from z for comparison
xhat_trans = T \ z_trans;

fprintf('\nControl input difference norm  : %.6f\n', norm(u_std - u_trans));
fprintf('Plant state difference norm    : %.6f\n', norm(x_std - x_trans));

%% 7. Plotting
ref_deg = rad2deg(ref);

figure('Color', 'w', 'Position', [50, 50, 1200, 750], ...
       'Name', 'Tracking: Standard vs Integer-Transformed Observer');

% ── Pitch angle ──────────────────────────────────────────────────────
subplot(3, 2, 1);
plot(t, rad2deg(x_std(1,:)),    'b-',  'LineWidth', 2,   'DisplayName', 'Pitch (Std)');   hold on;
plot(t, rad2deg(x_trans(1,:)),  'r--', 'LineWidth', 1.5, 'DisplayName', 'Pitch (Trans)');
yline(ref_deg(1), 'k:', 'LineWidth', 1, 'DisplayName', 'Ref');
grid on; ylabel('Pitch (deg)'); title('Pitch Angle');
legend('Location', 'southeast'); ylim([-2, 14]);

% ── Yaw angle ────────────────────────────────────────────────────────
subplot(3, 2, 2);
plot(t, rad2deg(x_std(2,:)),    'b-',  'LineWidth', 2,   'DisplayName', 'Yaw (Std)');     hold on;
plot(t, rad2deg(x_trans(2,:)),  'r--', 'LineWidth', 1.5, 'DisplayName', 'Yaw (Trans)');
yline(ref_deg(2), 'k:', 'LineWidth', 1, 'DisplayName', 'Ref');
grid on; ylabel('Yaw (deg)'); title('Yaw Angle');
legend('Location', 'southeast'); ylim([-14, 2]);

% ── Main motor voltage ────────────────────────────────────────────────
subplot(3, 2, 3);
plot(t, u_std(1,:),   'b-',  'LineWidth', 2,   'DisplayName', 'V\_main (Std)');   hold on;
plot(t, u_trans(1,:), 'r--', 'LineWidth', 1.5, 'DisplayName', 'V\_main (Trans)');
grid on; ylabel('Voltage (V)'); title('Main Motor Voltage');
legend('Location', 'northeast');

% ── Tail motor voltage ────────────────────────────────────────────────
subplot(3, 2, 4);
plot(t, u_std(2,:),   'b-',  'LineWidth', 2,   'DisplayName', 'V\_tail (Std)');   hold on;
plot(t, u_trans(2,:), 'r--', 'LineWidth', 1.5, 'DisplayName', 'V\_tail (Trans)');
grid on; ylabel('Voltage (V)'); title('Tail Motor Voltage');
legend('Location', 'northeast');

% ── Observer state comparison (pitch) ────────────────────────────────
subplot(3, 2, 5);
plot(t, rad2deg(xhat_std(1,:)),   'b-',  'LineWidth', 2,   'DisplayName', 'xhat1 (Std)');   hold on;
plot(t, rad2deg(xhat_trans(1,:)), 'r--', 'LineWidth', 1.5, 'DisplayName', 'xhat1 (Trans)');
plot(t, rad2deg(x_std(1,:)),      'k:',  'LineWidth', 1,   'DisplayName', 'True x1');
grid on; ylabel('Pitch (deg)'); xlabel('Time (s)');
title('Observer State: Pitch'); legend('Location', 'southeast');

% ── Control input error ───────────────────────────────────────────────
subplot(3, 2, 6);
plot(t, u_std(1,:) - u_trans(1,:), 'b', 'LineWidth', 1.5, 'DisplayName', '\Deltau_1'); hold on;
plot(t, u_std(2,:) - u_trans(2,:), 'r', 'LineWidth', 1.5, 'DisplayName', '\Deltau_2');
grid on; ylabel('Error (V)'); xlabel('Time (s)');
title(sprintf('Control Difference (norm = %.2e)', norm(u_std - u_trans)));
legend('Location', 'best');

sgtitle('Tracking Control: Standard vs Integer-Transformed Observer', 'FontSize', 13);


%% Helper functions
function print_numpy(name, M, prec)
    if nargin < 3, prec = 4; end
    fmt = ['%0.' num2str(prec) 'f'];
    if isvector(M)
        fprintf('%s = np.array([ ', name);
        for k = 1:numel(M)
            fprintf(fmt, M(k));
            if k < numel(M), fprintf(', '); end
        end
        fprintf(' ], dtype=np.float64)\n\n', numel(M));
    else
        fprintf('%s = np.array([\n', name);
        for i = 1:size(M,1)
            fprintf('    [ ');
            for j = 1:size(M,2)
                fprintf(fmt, M(i,j));
                if j < size(M,2), fprintf(', '); end
            end
            if i < size(M,1), fprintf(' ],\n'); else, fprintf(' ]\n'); end
        end
        fprintf('], dtype=np.float64)\n\n');
    end
end

function print_go(name, M, prec)
    if nargin < 3, prec = 4; end
    fmt = ['%0.' num2str(prec) 'f'];
    fprintf('%s := [][]float64{\n', name);
    for i = 1:size(M,1)
        fprintf('\t{');
        for j = 1:size(M,2)
            fprintf(fmt, M(i,j));
            if j < size(M,2), fprintf(', '); end
        end
        fprintf('},\n');
    end
    fprintf('}\n\n');
end
