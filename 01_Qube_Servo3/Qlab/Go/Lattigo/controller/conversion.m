clear all;
clc;
close all;
%% Observer-based controller design
% Example: four-tank system
% parameters

A0 = [ 0         0    1.0000         0;
      0         0         0    1.0000;
      0  149.2751   -0.0104         0;
      0  261.6091   -0.0103         0 ];

B0 = [ 0;
      0;
  -49.7275;
  -49.1493 ];


C = [1 0 0 0;
     0 1 0 0];

% sampling time
Ts = 0.02;

% discretize
sysC = ss(A0,B0,C,[]);
sysD = c2d(sysC, Ts);
A = sysD.A;
B = sysD.B;

% dimensions
[n,m] = size(B);
[l,~] = size(C);

% controller design
Q = [1000 0 0 0; 0 1000 0 0; 0 0 0 0; 0 0 0 0];
R1 = eye(m);
R2 = eye(l);
[~, K, ~] = idare(A,B,Q,R1,[],[]);
K = -K;


obs_poles = [0.71 0.72 0.73 0.74];            % 크기를 절반으로 줄인 극들
L = place(A', C', obs_poles).';      % 듀얼 시스템에 극배치 → 전치


% (F,G,H): resulting controller
F = A + B*K - L*C;
G = L;
H = K;



%% Converting the state matrix into integers
% One may freely change F, G, and H to different systems as they choose
% Finds R such that (F-RH) is an integer matrix through pole-placement


% Assign integer poles to (F-RH)
poles = [0,1,2,-1]; % Must consist of n-integers!
R = place(F.',H.',poles);
R = R.';




% Convert to modal canonical form
sys = ss(F-R*H, G, H, []);
[csys,T] = canon(sys, 'companion');
F_ = T*(F-R*H)/T
R_ = T*R
G_ = T*G
H_ = H/T


T_sim = 1;              % 시뮬레이션 시간 (초)
N_sim = round(T_sim/Ts); % 총 스텝 수
t = (0:N_sim-1) * Ts;    % 시간 축

x0 = [10; 10; 0; 0]; %

% Case 1: Original (F, G, H)
x_std = zeros(n, N_sim);
xhat_std = zeros(n, N_sim);
u_std = zeros(m, N_sim);
x_std(:, 1) = x0;
xhat_std(:, 1) = zeros(n, 1); 

% Case 2: Transformed (F_, G_, H_, R_)
x_trans = zeros(n, N_sim);
z_trans = zeros(n, N_sim); %  z = T*xhat
u_trans = zeros(m, N_sim);
x_trans(:, 1) = x0;
z_trans(:, 1) = zeros(n, 1);

for k = 1:N_sim-1

    y_k = C * x_std(:, k);
    
    u_std(:, k) = H * xhat_std(:, k);

    x_std(:, k+1) = A * x_std(:, k) + B * u_std(:, k);

    xhat_std(:, k+1) = F * xhat_std(:, k) + G * y_k;
    

    y_k_trans = C * x_trans(:, k);

    u_trans(:, k) = H_ * z_trans(:, k);
    

    x_trans(:, k+1) = A * x_trans(:, k) + B * u_trans(:, k);
    
    z_trans(:, k+1) = F_ * z_trans(:, k) + G_ * y_k_trans + R_ * u_trans(:, k);
end

u_std(:, end) = H * xhat_std(:, end);
u_trans(:, end) = H_ * z_trans(:, end);

% 오차 계산
ctrl_diff_norm = norm(u_std - u_trans);
state_diff_norm = norm(x_std - x_trans);

figure('Color', 'w', 'Position', [100, 100, 1000, 600]);

subplot(2,2,[1 3]);
plot(t, x_std(1,:), 'r-', 'LineWidth', 2, 'DisplayName', 'x1 (Std)'); hold on;
plot(t, x_std(2,:), 'b-', 'LineWidth', 2, 'DisplayName', 'x3 (Std)');
plot(t, x_trans(1,:), 'ko', 'MarkerSize', 4, 'DisplayName', 'x1 (Trans)');
plot(t, x_trans(2,:), 'gx', 'MarkerSize', 4, 'DisplayName', 'x3 (Trans)');
grid on; xlabel('Time (s)'); ylabel('State Value');
title('System State Response (Regulation)');
legend('Location', 'best');



subplot(2,2,2);
plot(t, u_std(1,:), 'r-', 'LineWidth', 2, 'DisplayName', 'u (Std)'); hold on;
plot(t, u_trans(1,:), 'k--', 'LineWidth', 2, 'DisplayName', 'u (Trans)');
grid on; xlabel('Time (s)'); ylabel('Voltage (V)');
title('Control Input Comparison');
legend('Location', 'best');

subplot(2,2,4);
est_error = x_std - xhat_std; 
plot(t, est_error(1,:), 'm', 'DisplayName', 'e1'); hold on;
plot(t, est_error(3,:), 'c', 'DisplayName', 'e3');
grid on; xlabel('Time (s)'); ylabel('Error');
title('Observer Estimation Error (Std Case)');
legend('Location', 'best');





%% 결과 확인
% 위에서 F, G, H를 구한 다음:


print_numpy('A', A, 4);
print_numpy('B', B, 4);
print_numpy('C', C, 4);






print_numpy('F', F, 4);
print_numpy('G', G, 4);
print_numpy('H', H, 4);





% (참고) 구현 시:
% xhat_{k+1} = F*xhat_k + G*y_k
% u_k        = H*xhat_k

function print_numpy(name, M, prec)
    if nargin < 3, prec = 4; end
    fmt = ['%0.' num2str(prec) 'f'];

    if isvector(M)
        fprintf('%s = np.array([ ', name);
        for k = 1:numel(M)
            fprintf(fmt, M(k));
            if k < numel(M), fprintf(', '); end
        end
        fprintf(' ], dtype=np.float64)  # shape (%d,)\n\n', numel(M));
    else
        fprintf('%s = np.array([\n', name);
        for i = 1:size(M,1)
            fprintf('    [ ');
            for j = 1:size(M,2)
                fprintf(fmt, M(i,j));
                if j < size(M,2), fprintf(', '); end
            end
            if i < size(M,1)
                fprintf(' ],\n');
            else
                fprintf(' ]\n');
            end
        end
        fprintf('], dtype=np.float64)\n\n');
    end
end








print_go('F', round(F_), 4);
print_go('G', G_, 4);
print_go('H', H_, 4);
print_go('R', R_, 4);

fprintf('// --------------------------------\n');

print_go('F', round(F_), 4);
print_go('GBar', round(G_*1000), 0);
print_go('HBar', round(H_*1000), 0);
print_go('RBar', round(R_*1000), 0);


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
