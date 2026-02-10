%% 0. 환경 초기화
clear; clc; close all;

%% 1. 데이터 로드 및 전처리
filename = 'qube_data_final.csv'; 
if ~exist(filename, 'file')
    error(['파일을 찾을 수 없습니다: ', filename]);
end
opts = detectImportOptions(filename);
opts.VariableNamingRule = 'preserve'; 
data = readtable(filename, opts);

% 데이터 커팅 및 변수 할당
offset = 100;
valid_range = (offset + 1) : (height(data) - offset);
time = data.Time(valid_range);
u_data = -data.u_noisy(valid_range); % 입력 부호 반전
theta_data = data.theta(valid_range);
alpha_data = data.alpha(valid_range);
y_data = [theta_data, alpha_data];
dt = mean(diff(time));
N = length(u_data);

%% 2. 원본 데이터 시각화 (Raw Data Plot)
figure('Name', 'Raw Data Visualization', 'Color', 'w', 'Position', [100 100 900 700]);
subplot(3,1,1);
plot(time, u_data, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1);
title('Control Input (Voltage)'); ylabel('u [V]'); grid on;

subplot(3,1,2);
plot(time, rad2deg(theta_data), 'b', 'LineWidth', 1);
title('Base Position (\theta)'); ylabel('Angle [deg]'); grid on;

subplot(3,1,3);
plot(time, rad2deg(alpha_data), 'r', 'LineWidth', 1);
title('Pendulum Angle (\alpha)'); ylabel('Angle [deg]'); xlabel('Time [s]'); grid on;
fprintf('데이터 시각화 완료.\n');

%% 2. 정답 모델(Reference) 정의 (시뮬레이션 검증용)
Ad_ref = [1.0000, 0.0301,  0.0200, 0.0002; 
          0,      1.0528, -0.0000, 0.0204;
          0,      3.0375,  0.9998, 0.0301; 
          0,      5.3236, -0.0002, 1.0528];
Bd_ref = [0.0100; 0.0099; 1.0043; 1.0001];
Cd_ref = [1, 0, 0, 0; 0, 1, 0, 0];

%% 3. 최소자승법(Matrix LS)을 이용한 시스템 식별
Y = y_data(3:N, :); 
Phi_mat = [y_data(2:N-1, :), y_data(1:N-2, :), u_data(2:N-1), u_data(1:N-2)];
W_mat = (Phi_mat' * Phi_mat) \ (Phi_mat' * Y);

A1_est = W_mat(1:2, :)'; 
A2_est = W_mat(3:4, :)'; 
B1_est = W_mat(5, :)';   
B2_est = W_mat(6, :)';   

Ad_ident = [A1_est, eye(2); A2_est, zeros(2)];
Bd_ident = [B1_est; B2_est];
Cd_ident = [eye(2), zeros(2)];

%% 4. 폴-제로 맵 비교
sys_ref = ss(Ad_ref, Bd_ref, Cd_ref, 0, dt);
sys_id  = ss(Ad_ident, Bd_ident, Cd_ident, 0, dt);

figure('Name', 'Channel-wise Pole-Zero Map', 'Color', 'w', 'Position', [100, 100, 1000, 450]);
subplot(1,2,1); hold on;
h1 = plot(real(pole(sys_ref)), imag(pole(sys_ref)), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
h2 = plot(real(pole(sys_id)), imag(pole(sys_id)), 'r+', 'MarkerSize', 12, 'LineWidth', 1.5);
title('u \rightarrow \theta Channel'); grid on; axis equal;
subplot(1,2,2); hold on;
plot(real(pole(sys_ref)), imag(pole(sys_ref)), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
plot(real(pole(sys_id)), imag(pole(sys_id)), 'r+', 'MarkerSize', 12, 'LineWidth', 1.5);
title('u \rightarrow \alpha Channel'); grid on; axis equal;

%% 4. Standard-CCF 구성 및 수동 계수 검증
[num_th, den] = ss2tf(Ad_ident, Bd_ident, Cd_ident(1,:), 0);
[num_al, ~]   = ss2tf(Ad_ident, Bd_ident, Cd_ident(2,:), 0);

% 수동 공식 적용
a1_11 = A1_est(1,1); a1_12 = A1_est(1,2); a1_21 = A1_est(2,1); a1_22 = A1_est(2,2);
a2_11 = A2_est(1,1); a2_12 = A2_est(1,2); a2_21 = A2_est(2,1); a2_22 = A2_est(2,2);

d1_m = -(a1_11 + a1_22);
d2_m = (a1_11*a1_22 - a1_12*a1_21) - (a2_11 + a2_22);
d3_m = (a1_11*a2_22 + a1_22*a2_11 - a1_12*a2_21 - a1_21*a2_12);
d4_m = (a2_11*a2_22 - a2_12*a2_21);

A_std_ccf = [0, 1, 0, 0; 0, 0, 1, 0; 0, 0, 0, 1; -d4_m, -d3_m, -d2_m, -d1_m];
B_std_ccf = [0; 0; 0; 1];
C_std_ccf = [num_th(5), num_th(4), num_th(3), num_th(2); num_al(5), num_al(4), num_al(3), num_al(2)];

%% 6. 제어기 설계 (Deadbeat Observer 적용)
des_poles_K = [0.61, 0.62, 0.63, 0.64]; 
K = place(Ad_ident, Bd_ident, des_poles_K);

% Deadbeat L 설계 (L1=A1, L2=A2)
L_sub = [A1_est; A2_est];

% OCF 기반 제어기 행렬
F_ocf = Ad_ident - Bd_ident*K - L_sub*Cd_ident;
G_ocf = L_sub;
H_ocf = K;

% CCF 기반 제어기 설계 (비교용)
K_ccf = place(A_std_ccf, B_std_ccf, des_poles_K);
% CCF용 L은 간단히 place 사용 (중복 회피를 위해 미세 조정)
L_ccf = place(A_std_ccf', C_std_ccf', [0.001, 0.0011, 0.0012, 0.0013]).';
F_ccf = A_std_ccf - B_std_ccf*K_ccf - L_ccf*C_std_ccf;
G_ccf = L_ccf;
H_ccf = K_ccf;

%% 7. 시뮬레이션 비교 수행 (3개 케이스 통일)
sim_time = 1; N_sim = round(sim_time / dt); t_sim = (0:N_sim-1)*dt;
x0_phys = [0; deg2rad(5); 0; 0]; 

% Case 1: True Plant + OCF Controller
x_true_ocf = zeros(4, N_sim); x_true_ocf(:, 1) = x0_phys;
xhat_ocf = zeros(4, N_sim); 

% Case 2: True Plant + CCF Controller
x_true_ccf = zeros(4, N_sim); x_true_ccf(:, 1) = x0_phys;
xhat_ccf = zeros(4, N_sim); 

% Case 3: Identified Model (OCF) + OCF Controller (Dynamic)
x_id_model = zeros(4, N_sim); x_id_model(:, 1) = x0_phys; % 물리 초기값과 동일하게 설정
xhat_id_model = zeros(4, N_sim); % 옵저버 상태 추가

for k = 1:N_sim-1
    % --- Case 1: True Plant + OCF Controller ---
    y_meas1 = Cd_ref * x_true_ocf(:, k);
    u_ocf = -H_ocf * xhat_ocf(:, k);
    x_true_ocf(:, k+1) = Ad_ref * x_true_ocf(:, k) + Bd_ref * u_ocf;
    xhat_ocf(:, k+1) = F_ocf * xhat_ocf(:, k) + G_ocf * y_meas1;
    
    % --- Case 2: True Plant + CCF Controller ---
    y_meas2 = Cd_ref * x_true_ccf(:, k);
    u_ccf = -H_ccf * xhat_ccf(:, k);
    x_true_ccf(:, k+1) = Ad_ref * x_true_ccf(:, k) + Bd_ref * u_ccf;
    xhat_ccf(:, k+1) = F_ccf * xhat_ccf(:, k) + G_ccf * y_meas2;
    
    % --- Case 3: Identified Model (OCF) + OCF Controller ---
    y_meas3 = Cd_ident * x_id_model(:, k);
    u_id = -H_ocf * xhat_id_model(:, k); 
    % 식별된 플랜트 모델 업데이트
    x_id_model(:, k+1) = Ad_ident * x_id_model(:, k) + Bd_ident * u_id;
    % 식별된 모델용 옵저버 업데이트
    xhat_id_model(:, k+1) = F_ocf * xhat_id_model(:, k) + G_ocf * y_meas3;
end

%% 8. 시나리오별 제어 성능 시각화
figure('Name', 'Control Performance: Scenario Comparison', 'Color', 'w', 'Position', [100 100 850 650]);
subplot(2,1,1);
plot(t_sim, rad2deg(x_true_ocf(2,:)), 'b', 'LineWidth', 1.5); hold on;
plot(t_sim, rad2deg(x_true_ccf(2,:)), 'g--', 'LineWidth', 1.5);
plot(t_sim, rad2deg(x_id_model(2,:)), 'r:', 'LineWidth', 2);
title('Pendulum Angle (\alpha) Stabilization'); ylabel('Angle (deg)'); grid on;
legend('True Plant (OCF Ctrl)', 'True Plant (CCF Ctrl)', 'Identified Model (Dynamic)');

subplot(2,1,2);
plot(t_sim, rad2deg(x_true_ocf(1,:)), 'b', 'LineWidth', 1.5); hold on;
plot(t_sim, rad2deg(x_true_ccf(1,:)), 'g--', 'LineWidth', 1.5);
plot(t_sim, rad2deg(x_id_model(1,:)), 'r:', 'LineWidth', 2);
title('Base Position (\theta) Stabilization'); ylabel('Angle (deg)'); xlabel('Time (s)'); grid on;
legend('True Plant (OCF Ctrl)', 'True Plant (CCF Ctrl)', 'Identified Model (Dynamic)');

%% 9. 제어기 행렬 출력
fprintf('\n# 1. [OCF Based Controller] - Recommended for sensor data\n');
print_numpy('F', F_ocf, 6);
print_numpy('G', G_ocf, 6);
print_numpy('H', H_ocf, 6);

%% 파이썬 출력용 서브 함수
function print_numpy(name, M, prec)
    if nargin < 3, prec = 6; end
    fmt = ['%0.' num2str(prec) 'f'];
    if isvector(M)
        fprintf('%s = np.array([', name);
        for k = 1:numel(M)
            fprintf(fmt, M(k));
            if k < numel(M), fprintf(', '); end
        end
        fprintf('], dtype=np.float64)\n\n');
    else
        fprintf('%s = np.array([\n', name);
        for i = 1:size(M,1)
            fprintf('    [');
            for j = 1:size(M,2)
                fprintf(fmt, M(i,j));
                if j < size(M,2), fprintf(', '); end
            end
            fprintf(']%s\n', char(repmat(',', i < size(M,1))));
        end
        fprintf('], dtype=np.float64)\n\n');
    end
end