clear all;
clc;
close all;

%% 1. Numerical 4th-order system  (n=4, m=2, p=1)
%
%   Two lightly-damped oscillatory modes.
%   u1 primarily drives mode 1, u2 primarily drives mode 2 (cross-coupling present).
%   y  = weighted sum of both mode positions → single sensor output.

w1 = 1.5;  z1 = 0.08;   % mode 1: slow, lightly damped
w2 = 4.0;  z2 = 0.12;   % mode 2: fast, slightly more damped

A0 = [      0,          1,        0,           0;
      -w1^2,  -2*z1*w1,        0,           0;
           0,          0,        0,           1;
           0,          0,  -w2^2,   -2*z2*w2];

B0 = [0.0,  0.0;
      10.0, 4.0;
      0.0,  0.0;
      3.0, 10.0];

C = [1.0, 0.0, 0.5, 0.0];   % 1×4

Ts = 0.05;
sysD = c2d(ss(A0, B0, C, []), Ts)
A = sysD.A;
B = sysD.B;

[n, m] = size(B);   % n=4, m=2
[p, ~] = size(C);   % p=1

%% 2. LQR + Feedforward + Observer
Q     = diag([10,1,10,1]);
R_lqr = eye(m);
[K, ~, ~] = dlqr(A, B, Q, R_lqr);

A_cl  = A - B*K;
G_dc  = C * inv(eye(n) - A_cl) * B;   % 1×2
N_bar = pinv(G_dc);                    % 2×1

obs_poles = [0.70, 0.73, 0.76, 0.79];
L = place(A', C', obs_poles).';        % 4×1

fprintf('\n=== Numerical System (n=%d, m=%d, p=%d) ===\n', n, m, p);
fprintf('G_dc = [%.4f  %.4f]\n', G_dc(1), G_dc(2));

%% 3. Observer-Based Controller Matrices
%
%   u(k)      = H  * xhat(k) + N_bar * r
%   xhat(k+1) = F  * xhat(k) + G * y(k) + P * r

F = A - B*K - L*C;   % 4×4
G = L;               % 4×1
H = -K;              % 2×4
P = B * N_bar;       % 4×1

%% 4. Null-Space Transformation  (valid when n = 2m)
%
%   F_ is nilpotent (F_^2 = 0) by construction for any n=2m system.

NullH = null(H);        % 4×2
r_dim = n - m;          % = 2

Tinv = zeros(n, n);
for i = 1:r_dim
    Tinv(:, i)        = F * NullH(:, i);
    Tinv(:, r_dim+i)  = NullH(:, i);
end
T = inv(Tinv);

F_can = T * F * Tinv;
H_can = H * Tinv;

R_can = Tinv * F_can(:, 1:m) / H_can(:, 1:m);

F_  = T * (F - R_can * H) * Tinv;
G_  = T * G;
R_  = T * R_can;
H_  = H * Tinv;
P_  = T * (P - R_can * N_bar);

fprintf('\n=== Transformed Matrices ===\n');
fprintf('F_ nilpotent check  — max|F_^2|           = %.2e\n', max(max(abs(F_^2))));
fprintf('Integer check       — max|F_ - round(F_)| = %.2e\n', max(max(abs(F_ - round(F_)))));

fprintf('\n=== Go (Transformed) ===\n');
print_go('F_',    round(F_), 0);
print_go('G_',    G_,        4);
print_go('R_',    R_,        4);
print_go('H_',    H_,        4);
print_go('P_',    P_,        4);
print_go('N_bar', N_bar,     4);

%% 5. Export matrices for simulation.go
outDir = fullfile(fileparts(mfilename('fullpath')), 'controller_data');
if ~exist(outDir, 'dir'), mkdir(outDir); end

writematrix(A,     fullfile(outDir, 'Ad.csv'));
writematrix(B,     fullfile(outDir, 'Bd.csv'));
writematrix(C,     fullfile(outDir, 'C.csv'));
writematrix(F_,    fullfile(outDir, 'F_.csv'));
writematrix(G_,    fullfile(outDir, 'G_.csv'));
writematrix(R_,    fullfile(outDir, 'R_.csv'));
writematrix(H_,    fullfile(outDir, 'H_.csv'));
writematrix(P_,    fullfile(outDir, 'P_.csv'));
writematrix(N_bar, fullfile(outDir, 'Nbar.csv'));
writematrix(Ts,    fullfile(outDir, 'Ts.csv'));

fprintf('\n=== Saved to %s/ ===\n', outDir);
fprintf('Now run: go run simulation.go\n\n');

%% 6. Simulation  (ILC setting: model uncertainty + trial-invariant disturbance)
T_sim = 10;
N_sim = round(T_sim / Ts);   % 200 steps
t     = (0:N_sim-1) * Ts;

f0 = 1 / T_sim;   % 0.1 Hz fundamental

% Periodic reference — integer-multiple frequencies → r(0) = r(T) = 0
ref_traj = 0.10*sin(2*pi*1*f0*t) + 0.05*sin(2*pi*3*f0*t);   % 1×N_sim


% --- Case 1: Standard ---
x_std    = zeros(n, N_sim);
xhat_std = zeros(n, N_sim);
u_std    = zeros(m, N_sim);

% --- Case 2: Transformed ---
x_trans  = zeros(n, N_sim);
z_trans  = zeros(n, N_sim);
u_trans  = zeros(m, N_sim);

for k = 1:N_sim-1
    ref = ref_traj(k);   % scalar

    % ── Standard ─────────────────────────────────────────────────────
    y_std       = C * x_std(:, k);
    u_std(:, k) = H * xhat_std(:, k) + N_bar * ref;

    x_std(:, k+1)    = A * x_std(:, k) + B * u_std(:, k);
    xhat_std(:, k+1) = F * xhat_std(:, k) + G * y_std + P * ref;

    % ── Transformed ──────────────────────────────────────────────────
    y_trans       = C * x_trans(:, k);
    u_trans(:, k) = H_ * z_trans(:, k) + N_bar * ref;

    x_trans(:, k+1)  = A * x_trans(:, k) + B * u_trans(:, k);
    z_trans(:, k+1)  = F_ * z_trans(:, k) + G_ * y_trans ...
                       + R_ * u_trans(:, k) + P_ * ref;
end

% Last step control
u_std(:, end)   = H  * xhat_std(:, end) + N_bar * ref_traj(end);
u_trans(:, end) = H_ * z_trans(:, end)  + N_bar * ref_traj(end);

xhat_trans = T \ z_trans;

fprintf('Control difference norm : %.6f\n', norm(u_std - u_trans));
fprintf('State difference norm   : %.6f\n', norm(x_std - x_trans));

%% 7. Plotting
y_std_out   = C * x_std;    % 1×N_sim
y_trans_out = C * x_trans;

figure('Color', 'w', 'Position', [50, 50, 1200, 700], ...
       'Name', '4th-order Numerical System: Standard vs Transformed Observer');

% ── Output tracking ───────────────────────────────────────────────────
subplot(3, 2, 1);
plot(t, y_std_out,   'b-',  'LineWidth', 2,   'DisplayName', 'y (Std)');   hold on;
plot(t, y_trans_out, 'r--', 'LineWidth', 1.5, 'DisplayName', 'y (Trans)');
plot(t, ref_traj,    'k:',  'LineWidth', 1,   'DisplayName', 'Ref');
grid on; ylabel('y'); xlabel('Time (s)'); title('Output Tracking');
xlim([0, T_sim]); legend('Location', 'northeast');

% ── Tracking error ────────────────────────────────────────────────────
subplot(3, 2, 2);
plot(t, ref_traj - y_std_out,   'b-',  'LineWidth', 1.5, 'DisplayName', 'e (Std)');   hold on;
plot(t, ref_traj - y_trans_out, 'r--', 'LineWidth', 1.5, 'DisplayName', 'e (Trans)');
grid on; ylabel('Error'); xlabel('Time (s)'); title('Tracking Error (ILC residual)');
xlim([0, T_sim]); legend('Location', 'best');

% ── Control input u1 ─────────────────────────────────────────────────
subplot(3, 2, 3);
plot(t, u_std(1,:),   'b-',  'LineWidth', 2,   'DisplayName', 'u_1 (Std)');   hold on;
plot(t, u_trans(1,:), 'r--', 'LineWidth', 1.5, 'DisplayName', 'u_1 (Trans)');
grid on; ylabel('u_1'); xlabel('Time (s)'); title('Control Input u_1');
xlim([0, T_sim]); legend('Location', 'northeast');

% ── Control input u2 ─────────────────────────────────────────────────
subplot(3, 2, 4);
plot(t, u_std(2,:),   'b-',  'LineWidth', 2,   'DisplayName', 'u_2 (Std)');   hold on;
plot(t, u_trans(2,:), 'r--', 'LineWidth', 1.5, 'DisplayName', 'u_2 (Trans)');
grid on; ylabel('u_2'); xlabel('Time (s)'); title('Control Input u_2');
xlim([0, T_sim]); legend('Location', 'northeast');

% ── Observer estimate vs true state ──────────────────────────────────
subplot(3, 2, 5);
plot(t, xhat_std(1,:),   'b-',  'LineWidth', 2,   'DisplayName', '\hat{x}_1 (Std)');   hold on;
plot(t, xhat_trans(1,:), 'r--', 'LineWidth', 1.5, 'DisplayName', '\hat{x}_1 (Trans)');
plot(t, x_std(1,:),      'k:',  'LineWidth', 1,   'DisplayName', 'True x_1');
grid on; ylabel('x_1'); xlabel('Time (s)'); title('Observer: x_1');
xlim([0, T_sim]); legend('Location', 'southeast');

% ── Control difference ────────────────────────────────────────────────
subplot(3, 2, 6);
plot(t, u_std(1,:) - u_trans(1,:), 'b-', 'LineWidth', 1.5, 'DisplayName', '\Deltau_1'); hold on;
plot(t, u_std(2,:) - u_trans(2,:), 'r-', 'LineWidth', 1.5, 'DisplayName', '\Deltau_2');
grid on; ylabel('Difference'); xlabel('Time (s)');
title(sprintf('Control Difference (norm = %.2e)', norm(u_std - u_trans)));
xlim([0, T_sim]); legend('Location', 'best');

sgtitle('4th-order Numerical System: Standard vs Transformed Observer', 'FontSize', 13);


%% Helper
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
