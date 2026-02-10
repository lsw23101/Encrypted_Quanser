%% 0. 환경 초기화
clear; clc; close all;

%% 1. 데이터 로드 및 전처리
data = readtable('qube_data.csv');
start_idx = 101; 
t_raw = data.Time(start_idx:end);
u = -1* data.u_noisy(start_idx:end); 
y1 = data.theta(start_idx:end);
y2 = data.alpha(start_idx:end);
Y = [y1, y2]; 
Ts = mean(diff(t_raw)); 

%% 2. [물리 모델 분석] 정답 모델의 이산화 및 계수 추출
% 연속 시간 물리 모델 (상태변수: [theta; alpha; theta_dot; alpha_dot])
A_true = [ 0, 0, 1, 0; 
           0, 0, 0, 1; 
           0, 149.2751, -0.0104, 0; 
           0, 261.6091, -0.0103, 0 ];
B_true = [ 0; 0; 49.7275; 49.1493 ];
C_true = [ 1, 0, 0, 0; 0, 1, 0, 0 ];
D_true = [ 0; 0 ];

sys_true_c = ss(A_true, B_true, C_true, D_true);
sys_true_d = c2d(sys_true_c, Ts, 'zoh');
[At, Bt, Ct, Dt] = ssdata(sys_true_d);

% 물리 모델의 전달함수 변환을 통한 차분 방정식 계수 추출 (ARX 형태 유도)
tf_true = tf(sys_true_d);
[num1, den1] = tfdata(tf_true(1), 'v'); % theta 식
[num2, den2] = tfdata(tf_true(2), 'v'); % alpha 식

%% 3. [데이터 식별] Least Squares (LS) 계산
% 모델 구조: y(k) = a1*y1(k-1) + a2*y1(k-2) + a3*y2(k-1) + a4*y2(k-2) + b1*u(k-1) + b2*u(k-2)
Phi = [y1(2:end-1), y1(1:end-2), y2(2:end-1), y2(1:end-2), u(2:end-1), u(1:end-2)];
Y_target = Y(3:end, :); 
Theta = Phi \ Y_target; 

%% 4. 결과 출력 및 비교
fprintf('\n======================================================\n');
fprintf('1. 물리 모델 20ms 이산 시간 상태공간 행렬 (x = [th; al; th_d; al_d])\n');
fprintf('======================================================\n');
disp('Ad (Physical):'); disp(At);
disp('Bd (Physical):'); disp(Bt);

fprintf('\n======================================================\n');
fprintf('2. 차분 방정식 계수 비교 (y(k) = a1*y1(k-1) + a2*y1(k-2) ...)\n');
fprintf('======================================================\n');
fprintf('%-15s | %-20s | %-20s\n', 'Coefficient', 'Physical Model', 'Data Identified (LS)');
fprintf('------------------------------------------------------\n');
% 물리 모델 전달함수 분모/분자에서 계수 매칭 (부호 주의: den은 1, -a1, -a2...)
fprintf('theta: a1      | %-20.6f | %-20.6f\n', -den1(2), Theta(1,1));
fprintf('theta: a2      | %-20.6f | %-20.6f\n', -den1(3), Theta(2,1));
fprintf('theta: b1      | %-20.6f | %-20.6f\n', num1(2),  Theta(5,1));
fprintf('------------------------------------------------------\n');
fprintf('alpha: a3      | %-20.6f | %-20.6f\n', -den2(2), Theta(3,2));
fprintf('alpha: a4      | %-20.6f | %-20.6f\n', -den2(3), Theta(4,2));
fprintf('alpha: b1      | %-20.6f | %-20.6f\n', num2(2),  Theta(5,2));

%% 5. 식별 모델을 상태공간 행렬(Ad, Bd)로 구성
Ad = [Theta(1,1), Theta(2,1), Theta(3,1), Theta(4,1);
      1,          0,          0,          0;
      Theta(1,2), Theta(2,2), Theta(3,2), Theta(4,2);
      0,          0,          1,          0];

Bd = [Theta(5,1); 0; Theta(5,2); 0];
Cd = [1 0 0 0; 0 0 1 0];

%% 6. 제어기 설계 (Pole Placement)
% 제어기 극점
des_poles_K = [0.755, 0.754, 0.962, 0.961]; 
K = place(Ad, Bd, des_poles_K);

% Q = [500 0 0 0; 0 100 0 0; 0 0 0 0; 0 0 0 0];
% R = 1;
% % 
% % %% 상태피드백 이득 K (discrete LQR)
% [K, S, cl_poles] = dlqr(Ad, Bd, Q, R);

% 옵저버 극점
des_poles_L = [0.60, 0.61, 0.62, 0.63];
L = place(Ad', Cd', des_poles_L).';


%% 6. 제어기 설계 (DLQR 기반)
% 
% --- [Controller Gain K 설계] ---
% Qy: 실제 출력 [theta; alpha]에 대한 가중치
% theta보다 alpha(진자 각도)의 오차에 더 민감하게 반응하도록 설정 권장
% Qy = diag([5000, 100]); 
% 
% 상태 가중치 행렬 Q_lqr 생성 (Q = C' * Qy * C)
% 상태가 섞여 있어도 Cd를 통해 출력 성분에만 정확히 페널티를 부여합니다.
% Q_lqr = Cd' * Qy * Cd;
% 수치적 안정성을 위해 대각 성분에 아주 작은 값 추가
% Q_lqr = Q_lqr + eye(size(Ad)) * 1e-4;
% 
% R_lqr: 입력 전압(Voltage) 사용에 대한 페널티
% 이 값이 커질수록 제어기가 부드러워지고, 작을수록 강력(Aggressive)해집니다.
% R_lqr = 1;
% 
% [K, ~, ~] = dlqr(Ad, Bd, Q_lqr, R_lqr);

% % --- [Observer Gain L 설계] ---
% % 옵저버는 LQR의 Dual 문제로 설계 (Kalman Filter 구조)
% % Qe: 시스템 모델의 불확실성 (클수록 센서 측정값을 더 믿음)
% Qe = eye(size(Ad)) * 100; 
% 
% % Re: 센서 측정 노이즈에 대한 가중치 (클수록 모델의 예측값을 더 믿음)
% % 20ms 샘플링 환경에서는 센서 노이즈가 존재하므로 적절히 조절합니다.
% Re = diag([1, 1]); 
% 
% % Dual 시스템에 대해 dlqr 적용하여 L 도출
% Lt = dlqr(Ad', Cd', Qe, Re);
% L = Lt';



%% 7. 동적 컨트롤러 실현 및 출력
F = Ad - Bd*K - L*Cd;
G = L;
H = K; % u = -Kx 구현을 위해 -K 사용

fprintf('\n======================================================\n');
fprintf('3. 최종 제어기 안정성 및 파이썬용 행렬 출력\n');
fprintf('======================================================\n');
max_eig_F = max(abs(eig(F)));
fprintf('Controller Max Eigenvalue: %.4f\n\n', max_eig_F);

print_numpy('F', F, 6);
print_numpy('G', G, 6);
print_numpy('H', H, 6);

%% --- Helper Function ---
function print_numpy(name, M, prec)
    if nargin < 3, prec = 4; end
    fmt = ['%0.' num2str(prec) 'f'];
    if isvector(M) && size(M,1) == 1
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
            if i < size(M,1)
                fprintf(' ],\n');
            else
                fprintf(' ]\n');
            end
        end
        fprintf('], dtype=np.float64)\n\n');
    end
end