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

% 초반/후반 안정화를 위해 데이터 커팅 (User 요청 반영)
offset = 100;
total_samples = height(data);
valid_range = (offset + 1) : (total_samples - offset);

time = data.Time(valid_range);
u_data = data.u_noisy(valid_range);
u_data = -u_data;
theta_data = data.theta(valid_range);
alpha_data = data.alpha(valid_range);

y_data = [theta_data, alpha_data]; % 출력 벡터 [N x 2]
dt = mean(diff(time));
N = length(u_data);

%% 2. 정답 모델(Reference)에서 차분방정식 계수 추출
% 시스템 행렬 정의
Ad_ref = [1.0000, 0.0301,  0.0200, 0.0002; 
          0,      1.0528, -0.0000, 0.0204;
          0,      3.0375,  0.9998, 0.0301; 
          0,      5.3236, -0.0002, 1.0528];
Bd_ref = [0.0100; 0.0099; 1.0043; 1.0001];
Cd_ref = [1, 0, 0, 0; 0, 1, 0, 0];

% 관측 행렬을 이용한 ARX 계수 유도 (추정치와 비교용)
% y[k] = C*x[k]
% y[k+1] = C*A*x[k] + C*B*u[k]
% [y[k]; y[k+1]] = [C; C*A]*x[k] + [0; C*B]*u[k]
Ob = [Cd_ref; Cd_ref*Ad_ref]; 
H = [zeros(2,1); Cd_ref*Bd_ref];

% 이 관계를 통해 유도된 이론적 ARX 계수 (A1, A2, B1, B2)
% 실제로는 데이터 식별값과 비교하기 위해 아래 LS 방식을 정답 모델 시뮬레이션에 먼저 적용해봅니다.

%% 3. 최소자승법(Least Squares)을 이용한 식별
% 모델: y[k] = A1*y[k-1] + A2*y[k-2] + B1*u[k-1] + B2*u[k-2]
% 타겟 행렬 Y (k=3부터 N까지)
Y = y_data(3:N, :); 

% 리그레서 행렬 Phi 구성
% Phi = [y[k-1], y[k-2], u[k-1], u[k-2]]
Phi = [y_data(2:N-1, :), y_data(1:N-2, :), u_data(2:N-1), u_data(1:N-2)];

% 회귀 분석 (W = Phi \ Y)
% W는 [6 x 2] 행렬이 되며, 각 열은 theta와 alpha에 대한 계수들임
W = (Phi' * Phi) \ (Phi' * Y);

% 식별된 계수 분리
A1_est = W(1:2, :)';
A2_est = W(3:4, :)';
B1_est = W(5, :)';
B2_est = W(6, :)';
%% 4. 정답 모델(Reference)로부터 이론적 ARX 계수 산출
% 가관측성 행렬 기반 유도 (2-step Observability)
O2 = [Cd_ref; Cd_ref*Ad_ref]; 

% 계수 계산 (A2, A1 순서로 계산됨)
A_coeffs_ref = (Cd_ref * Ad_ref^2) / O2;
A1_ref = A_coeffs_ref(:, 3:4);
A2_ref = A_coeffs_ref(:, 1:2);

% 입력 계수 계산
B1_ref = Cd_ref * Bd_ref;
B2_ref = Cd_ref * Ad_ref * Bd_ref - A1_ref * (Cd_ref * Bd_ref);

%% 5. 수치적 비교 출력
fprintf('\n======================================================\n');
fprintf('           차분방정식 계수 비교 분석 (Reference vs Ident)\n');
fprintf('======================================================\n');

% A1 Matrix 비교
fprintf('\n[A1 Matrix (y[k-1] Coeffs)]\n');
fprintf('Reference (정답):\n'); disp(A1_ref);
fprintf('Identified (식별):\n'); disp(A1_est);
fprintf('Error (%%): \n'); disp(abs(A1_ref - A1_est)./abs(A1_ref)*100);

% A2 Matrix 비교
fprintf('\n[A2 Matrix (y[k-2] Coeffs)]\n');
fprintf('Reference (정답):\n'); disp(A2_ref);
fprintf('Identified (식별):\n'); disp(A2_est);
fprintf('Error (%%): \n'); disp(abs(A2_ref - A2_est)./abs(A2_ref)*100);

% B1, B2 비교
fprintf('\n[B1, B2 Vectors (Input Coeffs)]\n');
B_table = table((1:2)', B1_ref, B1_est, B2_ref, B2_est, ...
    'VariableNames', {'Output', 'B1_Ref', 'B1_Ident', 'B2_Ref', 'B2_Ident'});
disp(B_table);

fprintf('======================================================\n');

%% 6. 식별된 계수를 상태공간 모델(SS)로 변환
nx = 4; nu = 1; ny = 2;
% 가관측 정형 (Observable Canonical Form) 구성
Ad_ident = [A1_est, eye(2); 
            A2_est, zeros(2)];
Bd_ident = [B1_est; 
            B2_est];
Cd_ident = [eye(2), zeros(2)];

%% 7. 제어기 및 옵저버 설계 (Based on Identified Model)
% 제어기 극점 배치 (K)
des_poles_K = [0.61, 0.62, 0.63, 0.64]; 
K = place(Ad_ident, Bd_ident, des_poles_K);

% 옵저버 극점 배치 (L)
des_poles_L = [0.41, 0.42, 0.43, 0.44];
L = place(Ad_ident', Cd_ident', des_poles_L).';

% 동적 제어기 행렬 구성 (u = -K * xhat)
F_ctrl = Ad_ident - Bd_ident*K - L*Cd_ident;
G_ctrl = L;
H_ctrl = K;

%% 8. 시뮬레이션 비교 수행
sim_time = 5; % 5초 시뮬레이션
N_sim = round(sim_time / dt);
t_sim = (0:N_sim-1)*dt;

% 초기 조건 설정 (진자가 5도 기울어진 상태 가정)
x0_ref = [0; deg2rad(5); 0; 0]; 
x_ref = zeros(4, N_sim); x_ref(:, 1) = x0_ref;
xhat_ref = zeros(4, N_sim); % 실제 플랜트용 옵저버 상태

x_ident = zeros(4, N_sim); x_ident(:, 1) = x0_ref; % 가상 플랜트 상태

for k = 1:N_sim-1
    % --- Case 1: 실제 물리 플랜트(Reference) 제어 ---
    y_ref_meas = Cd_ref * x_ref(:, k);
    u_ref = -H_ctrl * xhat_ref(:, k); % 식별 모델 기반 제어기 적용
    u_ref = max(min(u_ref, 10), -10); % 전압 제한
    
    % 플랜트 및 옵저버 업데이트
    x_ref(:, k+1) = Ad_ref * x_ref(:, k) + Bd_ref * u_ref;
    xhat_ref(:, k+1) = F_ctrl * xhat_ref(:, k) + G_ctrl * y_ref_meas;
    
    % --- Case 2: 식별된 가상 플랜트(Identified) 제어 ---
    y_id_meas = Cd_ident * x_ident(:, k);
    u_id = -H_ctrl * x_ident(:, k); % 상태 피드백 (가상 환경)
    u_id = max(min(u_id, 10), -10);
    
    x_ident(:, k+1) = Ad_ident * x_ident(:, k) + Bd_ident * u_id;
end

%% 9. 시뮬레이션 결과 시각화
figure('Name', 'Control Performance: True vs Identified', 'Color', 'w', 'Position', [100 100 800 600]);

% 진자 각도(Alpha) 비교
subplot(2,1,1);
plot(t_sim, rad2deg(x_ref(2,:)), 'b', 'LineWidth', 1.5); hold on;
plot(t_sim, rad2deg(x_ident(2,:)), 'r--', 'LineWidth', 1.5);
title('Pendulum Angle (\alpha) Stabilization');
ylabel('Angle (deg)'); grid on;
legend('On True Plant', 'On Identified Plant');

% 베이스 각도(Theta) 비교
subplot(2,1,2);
plot(t_sim, rad2deg(x_ref(1,:)), 'b', 'LineWidth', 1.5); hold on;
plot(t_sim, rad2deg(x_ident(1,:)), 'r--', 'LineWidth', 1.5);
title('Base Position (\theta) Stabilization');
ylabel('Angle (deg)'); xlabel('Time (s)'); grid on;

fprintf('\n시뮬레이션 완료. 그래프를 통해 제어 성능을 확인하십시오.\n');


%% [기존 코드 유지] 0 ~ 9 섹션 (시스템 식별 및 시뮬레이션)
% ... (사용자께서 제공하신 1번부터 9번까지의 코드가 여기에 위치합니다) ...

%% 10. 제어기 행렬 출력 및 안정성 확인
fprintf('\n======================================================\n');
fprintf('           제어기 행렬 출력 및 안정성 검사\n');
fprintf('======================================================\n');

% 1) 제어기(F_ctrl)의 안정성 확인 (Discrete-time Stability)
% 모든 고유값의 크기가 1보다 작아야 수치적으로 안정합니다.
ctrl_poles = eig(F_ctrl);
fprintf('[1] 제어기 행렬 F_ctrl의 고유값 크기 (Mag < 1 필수):\n');
disp(abs(ctrl_poles)');

if all(abs(ctrl_poles) < 1)
    fprintf('결과: 제어기가 안정합니다. (Stable)\n');
else
    fprintf('경고: 제어기가 불안정합니다. (Unstable) 극점 배치를 조정하세요.\n');
end

% 2) 파이썬(NumPy) 포맷 출력
fprintf('\n[2] 파이썬(NumPy) 복사용 제어기 행렬:\n');
fprintf('import numpy as np\n\n');

print_numpy('F', F_ctrl, 6);
print_numpy('G', G_ctrl, 6);
print_numpy('H', H_ctrl, 6);

%% 파이썬 출력용 서브 함수 (코드 하단에 배치)
function print_numpy(name, M, prec)
    if nargin < 3, prec = 4; end
    fmt = ['%0.' num2str(prec) 'f'];
    if isvector(M) && size(M,1) == 1
        % 행 벡터 처리
        fprintf('%s = np.array([ ', name);
        for k = 1:numel(M)
            fprintf(fmt, M(k));
            if k < numel(M), fprintf(', '); end
        end
        fprintf(' ], dtype=np.float64)\n\n');
    else
        % 행렬 처리
        fprintf('%s = np.array([\n', name);
        for i = 1:size(M,1)
            fprintf('    [ ');
            for j = 1:size(M,2)
                fprintf(fmt, M(i,j));
                if j < size(M,2), fprintf(', '); end
            end
            fprintf(' ]%s\n', char(repmat(',', i < size(M,1))));
        end
        fprintf('], dtype=np.float64)\n\n');
    end
end