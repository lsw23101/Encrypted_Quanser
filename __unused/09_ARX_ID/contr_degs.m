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

% 1) 제어 입력 (Control Input, u)
subplot(3,1,1);
plot(time, u_data, 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1);
title('Control Input (Voltage)');
ylabel('u [V]');
grid on;

% 2) 베이스 각도 (Base Angle, theta)
subplot(3,1,2);
plot(time, rad2deg(theta_data), 'b', 'LineWidth', 1);
title('Base Position (\theta)');
ylabel('Angle [deg]');
grid on;

% 3) 진자 각도 (Pendulum Angle, alpha)
subplot(3,1,3);
plot(time, rad2deg(alpha_data), 'r', 'LineWidth', 1);
title('Pendulum Angle (\alpha)');
ylabel('Angle [deg]');
xlabel('Time [s]');
grid on;

fprintf('데이터 시각화 완료. 그래프를 확인하여 신호의 이상 유무를 점검하십시오.\n');



%% 2. 정답 모델(Reference) 정의 (시뮬레이션 검증용)
Ad_ref = [1.0000, 0.0301,  0.0200, 0.0002; 
          0,      1.0528, -0.0000, 0.0204;
          0,      3.0375,  0.9998, 0.0301; 
          0,      5.3236, -0.0002, 1.0528];
Bd_ref = [0.0100; 0.0099; 1.0043; 1.0001];
Cd_ref = [1, 0, 0, 0; 0, 1, 0, 0];

%% 3. 최소자승법(Matrix LS)을 이용한 시스템 식별
% 모델: y[k] = A1*y[k-1] + A2*y[k-2] + B1*u[k-1] + B2*u[k-2]
Y = y_data(3:N, :); 
Phi_mat = [y_data(2:N-1, :), y_data(1:N-2, :), u_data(2:N-1), u_data(1:N-2)];
W_mat = (Phi_mat' * Phi_mat) \ (Phi_mat' * Y);

% 행렬 계수 분리
A1_est = W_mat(1:2, :)'; 
A2_est = W_mat(3:4, :)'; 
B1_est = W_mat(5, :)';   
B2_est = W_mat(6, :)';   

% [방식 B] Standard-OCF 구성 (식별의 본체)
Ad_ident = [A1_est, eye(2); A2_est, zeros(2)];
Bd_ident = [B1_est; B2_est];
Cd_ident = [eye(2), zeros(2)];


%% 4. 폴-제로 맵 비교 (각 채널별 영점 포함)
sys_ref = ss(Ad_ref, Bd_ref, Cd_ref, 0, dt);
sys_id  = ss(Ad_ident, Bd_ident, Cd_ident, 0, dt);

% 각 채널 분리 (1번 출력: theta, 2번 출력: alpha)
sys_ref_theta = sys_ref(1,:); sys_ref_alpha = sys_ref(2,:);
sys_id_theta  = sys_id(1,:);  sys_id_alpha  = sys_id(2,:);

% 영점 추출
z_ref_theta = zero(sys_ref_theta); z_ref_alpha = zero(sys_ref_alpha);
z_id_theta  = zero(sys_id_theta);  z_id_alpha  = zero(sys_id_alpha);
% 극점 추출 (극점은 시스템 고유 특성이라 채널과 상관없이 동일함)
p_ref = pole(sys_ref);
p_id  = pole(sys_id);

figure('Name', 'Channel-wise Pole-Zero Map', 'Color', 'w', 'Position', [100, 100, 1000, 450]);

% --- [왼쪽: theta 채널 비교] ---
subplot(1,2,1); hold on;
h1 = plot(real(p_ref), imag(p_ref), 'bx', 'MarkerSize', 10, 'LineWidth', 2); % Ref Poles
plot(real(z_ref_theta), imag(z_ref_theta), 'bo', 'MarkerSize', 10, 'LineWidth', 2); % Ref Zeros
h2 = plot(real(p_id), imag(p_id), 'r+', 'MarkerSize', 12, 'LineWidth', 1.5); % ID Poles
plot(real(z_id_theta), imag(z_id_theta), 'ro', 'MarkerSize', 8, 'LineWidth', 1.5); % ID Zeros
draw_unit_circle();
title('u \rightarrow \theta Channel'); grid on; axis equal;
legend([h1, h2], {'Reference', 'Identified'}, 'Location', 'northeast');

% --- [오른쪽: alpha 채널 비교] ---
subplot(1,2,2); hold on;
plot(real(p_ref), imag(p_ref), 'bx', 'MarkerSize', 10, 'LineWidth', 2);
plot(real(z_ref_alpha), imag(z_ref_alpha), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
plot(real(p_id), imag(p_id), 'r+', 'MarkerSize', 12, 'LineWidth', 1.5);
plot(real(z_id_alpha), imag(z_id_alpha), 'ro', 'MarkerSize', 8, 'LineWidth', 1.5);
draw_unit_circle();
title('u \rightarrow \alpha Channel'); grid on; axis equal;

function draw_unit_circle()
    th = 0:0.01:2*pi;
    plot(cos(th), sin(th), 'k:', 'LineWidth', 1);
    xlabel('Real'); ylabel('Imaginary');
end

%% 4. [방식 C] Standard-CCF (Companion Form) 구성
% B1, B2 정보를 모두 포함하기 위해 OCF 모델에서 전달함수를 추출합니다.
[num_th, den] = ss2tf(Ad_ident, Bd_ident, Cd_ident(1,:), 0);
[num_al, ~]   = ss2tf(Ad_ident, Bd_ident, Cd_ident(2,:), 0);

% A 행렬: 마지막 행에 특성 방정식 계수 배치 (Companion Matrix)
A_std_ccf = [0, 1, 0, 0;
             0, 0, 1, 0;
             0, 0, 0, 1;
             -den(5), -den(4), -den(3), -den(2)];


% ---------------------------------------------------------
% 2) 성분별 직접 전개 공식을 이용한 수동 계산 (Test)
% ---------------------------------------------------------
% A1_est와 A2_est의 성분 추출
a1_11 = A1_est(1,1); a1_12 = A1_est(1,2);
a1_21 = A1_est(2,1); a1_22 = A1_est(2,2);

a2_11 = A2_est(1,1); a2_12 = A2_est(1,2);
a2_21 = A2_est(2,1); a2_22 = A2_est(2,2);
% 공식 적용 (d1 ~ d4)
d1_manual = -(a1_11 + a1_22);
d2_manual = (a1_11*a1_22 - a1_12*a1_21) - (a2_11 + a2_22);
d3_manual = (a1_11*a2_22 + a1_22*a2_11 - a1_12*a2_21 - a1_21*a2_12);
d4_manual = (a2_11*a2_22 - a2_12*a2_21);

% 수동 den 벡터 구성 [1, d1, d2, d3, d4]
den_manual = [1, d1_manual, d2_manual, d3_manual, d4_manual];

% A 행렬 (테스트용) - 뺄셈 설계 시 사용하는 계수 배치
A_std_ccf_test = [0, 1, 0, 0;
                  0, 0, 1, 0;
                  0, 0, 0, 1;
                  -d4_manual, -d3_manual, -d2_manual, -d1_manual];


% ---------------------------------------------------------
% 3) 두 방식의 비교 출력
% ---------------------------------------------------------
fprintf('\n======================================================\n');
fprintf('      [4] 특성 방정식 계수 수동 공식 검증 비교\n');
fprintf('======================================================\n');
fprintf('항목         | ss2tf 방식    | 수동 공식 방식\n');
fprintf('------------------------------------------------------\n');
fprintf('d1 (z^3)    : %12.6f | %12.6f\n', den(2), d1_manual);
fprintf('d2 (z^2)    : %12.6f | %12.6f\n', den(3), d2_manual);
fprintf('d3 (z^1)    : %12.6f | %12.6f\n', den(4), d3_manual);
fprintf('d4 (const)  : %12.6f | %12.6f\n', den(5), d4_manual);

% 행렬 일치 여부 확인
A_err = max(max(abs(A_std_ccf - A_std_ccf_test)));
fprintf('\n>> 두 A 행렬의 최대 수치 오차: %.2e\n', A_err);

if A_err < 1e-10
    fprintf('>> 결과: ss2tf와 직접 전개 공식이 완벽하게 일치합니다.\n');
else
    fprintf('>> 경고: 수치적 차이가 발생했습니다. 공식을 다시 확인하세요.\n');
end


A_std_ccf = A_std_ccf_test

% B 행렬: 입력 에너지를 가속도 항(마지막 행)으로 집중
B_std_ccf = [0; 0; 0; 1];

% C 행렬: B1, B2의 영향력이 포함된 분자 계수를 배치
C_std_ccf = [num_th(5), num_th(4), num_th(3), num_th(2);
             num_al(5), num_al(4), num_al(3), num_al(2)];

%% 5. 모델 등가성 확인 (Verification: OCF vs Standard-CCF)
fprintf('\n======================================================\n');
fprintf('      시스템 모델 구조 비교 및 등가성 검증\n');
fprintf('======================================================\n');

% 행렬 값 출력
fprintf('\n[0-1] [방식 C] Standard-CCF 행렬 (Companion Form):\n');
disp(A_std_ccf); disp(B_std_ccf); disp(C_std_ccf);

fprintf('\n[0-2] [방식 B] Standard-OCF 행렬 (Identification Model):\n');
disp(Ad_ident); disp(Bd_ident); disp(Cd_ident);

% 고유값 비교
poles_ccf = sort(eig(A_std_ccf));
poles_ocf = sort(eig(Ad_ident));

fprintf('\n[1] 고유값(Poles) 비교 (CCF vs OCF):\n');
disp(table(poles_ccf, poles_ocf, 'VariableNames', {'Standard_CCF', 'Standard_OCF'}));

pole_err = max(abs(poles_ccf - poles_ocf));
if pole_err < 1e-10
    fprintf('>> 결과: 두 시스템은 수치적으로 동일합니다. (오차: %.2e)\n', pole_err);
else
    fprintf('>> 경고: 오차 발생. ss2tf 추출 과정을 확인하세요.\n');
end

%% 6. 제어기 및 옵저버 설계 (Based on OCF Model)
des_poles_K = [0.61, 0.62, 0.63, 0.64]; 
K = place(Ad_ident, Bd_ident, des_poles_K);

% des_poles_L = [0.41, 0.42, 0.43, 0.44];
des_poles_L = [0.0001, 0.00012, 0.00013, 0.00014];

L = place(Ad_ident', Cd_ident', des_poles_L).';

% 동적 제어기 행렬 구성 (u = -K * xhat)
F_ctrl = Ad_ident - Bd_ident*K - L*Cd_ident;
G_ctrl = L;
H_ctrl = K;

%% 6-2. 뺄셈 기반 K 게인 검산 (Standard-CCF 기반)
poly_des = poly(des_poles_K); % 목표 다항식
% K_ccf = [현재계수 - 목표계수]
K_ccf_manual = [den(5)-poly_des(5), den(4)-poly_des(4), den(3)-poly_des(3), den(2)-poly_des(2)];

%% 6-3. 뺄셈 기반 L 게인 원리 이해 (OCF의 쌍대성)
% % OCF에서 (A - LC)를 하면 L은 정확히 A1과 A2 블록의 '첫 번째 열'들에서 빠지게 됩니다.
% % y가 2개이므로 L = [L1; L2] (각 2x2) 형태입니다.
% 
% poly_des_L = poly(des_poles_L); % 목표 옵저버 다항식 [1, d1, d2, d3, d4]
% 
% % [참고] OCF에서 L의 역할:
% % A - LC = [ A1 - L1 ,  I ]
% %          [ A2 - L2 ,  0 ]
% % 이 행렬의 특성 방정식 계수들(Characteristic coefficients)은 
% % 원래 A1, A2의 계수에서 L의 성분들을 뺀 결과와 일치해야 합니다.
% 
% fprintf('\n======================================================\n');
% fprintf('      [6-3] 옵저버 게인 L의 뺄셈 원리 (OCF 기준)\n');
% fprintf('======================================================\n');
% fprintf('식별된 모델의 특성 계수(den): [%s]\n', num2str(den(2:5), '%.4f '));
% fprintf('목표 옵저버 특성 계수(des): [%s]\n', num2str(poly_des_L(2:5), '%.4f '));
% 
% % K와 마찬가지로, 이론적인 '계수 차이'는 다음과 같습니다.
% L_coeffs_diff = den(2:5) - poly_des_L(2:5);
% fprintf('필요한 계수 보정량 (den - des): [%s]\n', num2str(L_coeffs_diff, '%.4f '));
% 
% fprintf('\n설계된 L 행렬 (4x2):\n');
% disp(L);
% 
% fprintf('설명: L은 4x2 행렬이므로 8개의 숫자를 가집니다.\n');
% fprintf('이 8개의 숫자가 A1-L1, A2-L2에 적절히 분배되어,\n');
% fprintf('시스템 전체의 특성 계수를 정확히 [%.2f, %.2f, %.2f, %.2f]만큼 보정합니다.\n', L_coeffs_diff);
% 
% 
% fprintf('\n[K 게인 설계 검산 - Standard-CCF 뺄셈 방식]\n');
% fprintf('계산된 K_ccf (Manual Subtraction): [%s]\n', num2str(K_ccf_manual, '%.4f '));


%% 6-3. 뺄셈 기반 Deadbeat Observer 설계 (Poles = [0 0 0 0])
% Standard-OCF 구조에서는 L1=A1, L2=A2로 설정하면 
% 모든 상태 오차가 최소 스텝(2스텝) 내에 0으로 수렴하는 Deadbeat Observer가 됩니다.

% 1) 뺄셈 방식으로 L 게인 직접 할당 (L = [A1; A2])
L1_sub = A1_est; 
L2_sub = A2_est;
L_sub = [L1_sub; L2_sub]; % 4x2 행렬 생성

% 2) 목표 다항식 정의 (Poles at 0 -> z^4 = 0)
poly_des_L = [1, 0, 0, 0, 0]; % z^4 + 0z^3 + 0z^2 + 0z + 0

fprintf('\n======================================================\n');
fprintf('      [6-3] Deadbeat 옵저버 게인 L_sub 설계 (Poles=0)\n');
fprintf('======================================================\n');
fprintf('식별된 A1_est:\n'); disp(A1_est);
fprintf('식별된 A2_est:\n'); disp(A2_est);
fprintf('설계된 L_sub (4x2, A1과 A2를 그대로 할당):\n'); disp(L_sub);

% 3) 검증: A - LC의 고유값 확인
A_obs_closed = Ad_ident - L_sub * Cd_ident;
actual_L_poles = abs(eig(A_obs_closed));
fprintf('설계된 옵저버의 실제 극점 크기 (Abs): [%s]\n', num2str(actual_L_poles', '%.4e '));

%% 6-4. L_sub을 이용한 동적 제어기 행렬 최종 구성
% u[k] = -H*xhat[k], xhat[k+1] = F*xhat[k] + G*y[k]
F_ctrl = Ad_ident - Bd_ident*K - L_sub*Cd_ident;
G_ctrl = L_sub;
H_ctrl = K;

fprintf('\n동적 제어기 행렬(F, G, H)이 Deadbeat L_sub 기반으로 업데이트되었습니다.\n');



%% 6-4. 좌표 변환(Similarity Transform) 및 제어기 등가성 확인
fprintf('\n======================================================\n');
fprintf('      [6-4] 좌표 변환 기반 제어기 구성 및 등가성 확인\n');
fprintf('======================================================\n');

% 1. 좌표 변환 행렬 T 구하기 (x_ocf = T * x_ccf)
% 가제어성 행렬(Controllability Matrix)을 이용한 변환
Co_ocf = ctrb(Ad_ident, Bd_ident);
Co_ccf = ctrb(A_std_ccf, B_std_ccf);
T = Co_ocf / Co_ccf;  % T = Co_ocf * inv(Co_ccf)

% 2. 게인 변환 확인
% K_ocf = K_ccf * inv(T)
% L_ocf = T * L_ccf  => L_ccf = inv(T) * L_ocf
K_ccf = place(A_std_ccf, B_std_ccf, des_poles_K);
L_ccf = place(A_std_ccf', C_std_ccf', des_poles_L).';

K_ocf_transformed = K_ccf / T;
L_ocf_transformed = T * L_ccf;

% --- [Case 1] OCF 기준 제어기 (현재 사용자님 방식) ---
F_ocf = Ad_ident - Bd_ident*K - L*Cd_ident;
G_ocf = L;
H_ocf = K;

% --- [Case 2] CCF 기준 제어기 ---
F_ccf = A_std_ccf - B_std_ccf*K_ccf - L_ccf*C_std_ccf;
G_ccf = L_ccf;
H_ccf = K_ccf;

% 3. 제어기 등가성 검증 (DNA 비교)
poles_F_ocf = sort(eig(F_ocf));
poles_F_ccf = sort(eig(F_ccf));

fprintf('[1] 제어기(F)의 고유값 비교:\n');
disp(table(poles_F_ocf, poles_F_ccf, 'VariableNames', {'OCF_Based', 'CCF_Based'}));

% 4. 제어기 변환 관계 확인 (F_ocf = T * F_ccf * inv(T))
F_check = T * F_ccf / T;
err_F = max(max(abs(F_ocf - F_check)));

fprintf('[2] 제어기 구조적 일치 여부:\n');
if err_F < 1e-10
    fprintf('>> 결과: 두 제어기는 좌표 변환 관계(T)를 통해 완벽히 일치합니다.\n');
    fprintf('>> 오차: %.2e\n', err_F);
else
    fprintf('>> 경고: 오차 발생. 변환 행렬 T의 수치 안정성을 확인하세요.\n');
end



%% 7. 시뮬레이션 비교 수행 (3개 케이스)
sim_time = 1; N_sim = round(sim_time / dt); t_sim = (0:N_sim-1)*dt;

% 초기 조건 설정
x0_phys = [0; deg2rad(5); 0; 0]; % 실제 물리 상태 (theta, alpha, dot_theta, dot_alpha)

% --- 데이터 저장 공간 초기화 ---
% Case 1: True Plant + OCF Controller
x_true_ocf = zeros(4, N_sim); x_true_ocf(:, 1) = x0_phys;
xhat_ocf = zeros(4, N_sim); 

% Case 2: True Plant + CCF Controller
x_true_ccf = zeros(4, N_sim); x_true_ccf(:, 1) = x0_phys;
xhat_ccf = zeros(4, N_sim); 

% Case 3: Identified Model (OCF) + Ideal Control
x_id_model = zeros(4, N_sim); x_id_model(:, 1) = [Cd_ref*x0_phys; 0; 0]; % OCF 초기화

for k = 1:N_sim-1
    % --- Case 1: True Plant + OCF Controller ---
    y_meas1 = Cd_ref * x_true_ocf(:, k);
    u_ocf = -H_ocf * xhat_ocf(:, k);
    % u_ocf = max(min(u_ocf, 10), -10);
    x_true_ocf(:, k+1) = Ad_ref * x_true_ocf(:, k) + Bd_ref * u_ocf;
    xhat_ocf(:, k+1) = F_ocf * xhat_ocf(:, k) + G_ocf * y_meas1;
    
    % --- Case 2: True Plant + CCF Controller ---
    y_meas2 = Cd_ref * x_true_ccf(:, k);
    u_ccf = -H_ccf * xhat_ccf(:, k);
    % u_ccf = max(min(u_ccf, 10), -10);
    x_true_ccf(:, k+1) = Ad_ref * x_true_ccf(:, k) + Bd_ref * u_ccf;
    xhat_ccf(:, k+1) = F_ccf * xhat_ccf(:, k) + G_ccf * y_meas2;
    
% --- Case 3: Identified Model (OCF) + OCF Controller ---
    % 가상 모델에서도 실제와 똑같이 옵저버를 통해 상태를 추정하고 제어함
    y_meas3 = Cd_ident * x_id_model(:, k); % 식별된 모델의 출력 행렬 사용
    u_id = -H_ocf * xhat_id_model(:, k);    % 설계된 제어 게인 사용
    % u_id = max(min(u_id, 10), -10);
end

%% 8. 시나리오별 제어 성능 시각화

figure('Name', 'Control Performance: Scenario Comparison', 'Color', 'w', 'Position', [100 100 850 650]);

% 1) 진자 각도(Alpha) 비교
subplot(2,1,1);
plot(t_sim, rad2deg(x_true_ocf(2,:)), 'b', 'LineWidth', 1.5); hold on;
plot(t_sim, rad2deg(x_true_ccf(2,:)), 'g--', 'LineWidth', 1.5);
plot(t_sim, rad2deg(x_id_model(2,:)), 'r:', 'LineWidth', 2);
title('Pendulum Angle (\alpha) Stabilization');
ylabel('Angle (deg)'); grid on;
legend('True Plant (OCF Ctrl)', 'True Plant (CCF Ctrl)', 'Identified Model');

% 2) 베이스 각도(Theta) 비교
subplot(2,1,2);
plot(t_sim, rad2deg(x_true_ocf(1,:)), 'b', 'LineWidth', 1.5); hold on;
plot(t_sim, rad2deg(x_true_ccf(1,:)), 'g--', 'LineWidth', 1.5);
plot(t_sim, rad2deg(x_id_model(1,:)), 'r:', 'LineWidth', 2);
title('Base Position (\theta) Stabilization');
ylabel('Angle (deg)'); xlabel('Time (s)'); grid on;
legend('True Plant (OCF Ctrl)', 'True Plant (CCF Ctrl)', 'Identified Model');

fprintf('\n시뮬레이션 완료. 세 가지 케이스의 거동이 일치하는지 확인하십시오.\n');

%% 9. 제어기 행렬 출력 및 안정성 확인
fprintf('\n======================================================\n');
fprintf('           제어기 행렬 출력 및 안정성 검사\n');
fprintf('======================================================\n');
fprintf('[1] 제어기 행렬 F_ctrl의 고유값 크기:\n'); disp(abs(eig(F_ctrl))');
fprintf('\n[2] 파이썬(NumPy) 복사용 제어기 행렬:\n');
print_numpy('F', F_ctrl, 6);
print_numpy('G', G_ctrl, 6);
print_numpy('H', H_ctrl, 6);



%% 10. OCF 및 CCF 기반 제어기 파이썬(NumPy) 출력
fprintf('\n======================================================\n');
fprintf('      [10] OCF 및 CCF 기반 제어기 행렬 출력\n');
fprintf('======================================================\n');

% --- [Case 1] OCF 기반 제어기 출력 ---
% (참고: F_ocf는 F_ctrl과 수치적으로 거의 동일합니다)
fprintf('\n# 1. [OCF Based Controller] - Recommended for sensor data\n');
print_numpy('F', F_ocf, 6);
print_numpy('G', G_ocf, 6);
print_numpy('H', H_ocf, 6);

% --- [Case 2] CCF 기반 제어기 출력 ---
% (주의: 이 제어기를 파이썬에서 쓸 경우 상태변수 x_hat이 CCF 정의를 따라야 함)
fprintf('\n# 2. [CCF Based Controller] - Ideal for manual gain verification\n');
print_numpy('F', F_ccf, 6);
print_numpy('G', G_ccf, 6);
print_numpy('H', H_ccf, 6);



%% 파이썬 출력용 서브 함수 (벡터 감지 기능 포함)
function print_numpy(name, M, prec)
    if nargin < 3, prec = 6; end
    fmt = ['%0.' num2str(prec) 'f'];
    
    % M이 벡터(1행 N열 또는 N행 1열)인 경우 1차원 np.array로 출력
    if isvector(M)
        fprintf('%s = np.array([', name);
        for k = 1:numel(M)
            fprintf(fmt, M(k));
            if k < numel(M), fprintf(', '); end
        end
        fprintf('], dtype=np.float64)\n\n');
        
    else
        % M이 일반 행렬인 경우 2차원 구조로 출력
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