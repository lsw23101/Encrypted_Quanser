% countinuous time system
clc;
clear;


omega_0=3
Ac = [0 1; -omega_0^2 0];
Bc =[0; 1];
Cc=[1 0; 0 1]; % full state 
Dc=0;

sysc = ss(Ac,Bc,Cc,Dc);


%이산화

Ts=0.1;
sysd = c2d(sysc,Ts);

[Ad, Bd, Cd, Dd] = ssdata(sysd);

Tfinal = 5; % final time

Ud_rand = rand(1, Tfinal/Ts+1)*5 -2.5;


Tspan=0:Ts:Tfinal;

Ud=Ud_rand;
x0=[1;0];
[YY, TT, XX] =lsim(sysd, Ud, Tspan, x0);
X0=XX(1:(size(XX,1)-1),:)';
X1=XX(2:size(XX,1),:)';
U0=Ud(:, 1:size(X0,2));

% 상태 및 입력 stem 플롯
figure;

% --- 상태 x1, x2 ---
subplot(2,1,1);
stem(Tspan, XX(:,1), 'r', 'filled'); hold on;
stem(Tspan, XX(:,2), 'b', 'filled');
xlabel('Time [s]');
ylabel('States');
title('States x_1 (red) and x_2 (blue)');
legend('x_1','x_2');
grid on;

% --- 입력 u ---
subplot(2,1,2);
stem(Tspan, Ud, 'k', 'filled');
xlabel('Time [s]');
ylabel('Input u');
title('Input u (stem plot)');
grid on;



% [B A] = X1 * [U0; X0]^†

% X0, X1, U0가 이미 위에서 계산되어 있다고 가정
Z = [U0; X0];          % [U0; X0] 블록 행렬 구성
BA = X1 * pinv(Z);     % 식 적용

% B와 A 분리
nx = size(X0,1);       % 상태 개수
nu = size(U0,1);       % 입력 개수

B_est = BA(:, 1:nu);
A_est = BA(:, nu+1:end);

% 결과 출력
disp('A_est =');
disp(A_est);

disp('B_est =');
disp(B_est);


%% ========= (이전 셋업: 네 코드 바로 아래에 붙여 넣기) =========
% 여기까지로 Ad,Bd,Cd,Dd, X0,X1,U0, Z=[U0;X0] 가 준비되어 있음

n = size(X0,1);   % 상태 차원
m = size(U0,1);   % 입력 차원
T = size(X0,2);   % 샘플 수 (열 개수)

% --- 랭크 조건 점검: rank([U0;X0]) = n+m 이어야 식(10) 해가 존재 (정리1의 가정 (8))
rank_Z = rank([U0; X0]);
fprintf('rank([U0;X0]) = %d (필요: %d)\n', rank_Z, n+m);

%% ========= A) CVX 사용: 정리 2 (식(12) → (13))로 K, L_K, A_cl 구하기 =========
% (가능하면 이 경로를 권장: 식별 없이 데이터로 바로 안정화 K 계산)
use_cvx = exist('cvx_begin','file')==2;

if use_cvx
    fprintf('CVX를 사용해 데이터 주도 LMI (식(12))를 풉니다...\n');

    % 변수 Q \in R^{T x n}  (논문 표기와 동일: Q의 크기는 (열샘플 수) x (상태차원))
    cvx_begin sdp
        variable Q(T, n)
        % 식(12)에 해당: [X0*Q, X1*Q; (X1*Q)', X0*Q] \succ 0
        % 수치적 여유를 위해 작은 eps를 더해 양정상 보장
        eps_pd = 1e-3;
        M = [X0*Q, X1*Q; (X1*Q)', X0*Q];
        M >= eps_pd*eye(2*n);
        % (선택) X0*Q 대칭조건을 추가하는 버전(논문 부록 코드 참고)
        % X0*Q == (X0*Q)';
    cvx_end

    if ~strcmp(cvx_status,'Solved') && ~strcmp(cvx_status,'Inaccurate/Solved')
        print('CVX가 해를 찾지 못했습니다: status = %s', cvx_status);
    end

    % 식(13): K = U0*Q * (X0*Q)^{-1}
    P  = X0*Q;               % P ≻ 0
    K  = (U0*Q) / P;         % == U0*Q*(X0*Q)^{-1}

    % 식(10)과 연결해 L_K 도출: L_K = Q * (X0*Q)^{-1}  (L_K = Q * P^{-1})
    L_K = Q / P;

    % 식(9): 폐루프 시스템 행렬 A_cl = X1 * L_K
    A_cl = X1 * L_K;

    % 식(11): 제어입력 u(k) = U0 * L_K * x(k)  (= K*x(k)와 동일, K = U0*L_K)
    % 확인: U0*L_K == K
    fprintf('‖U0*L_K - K‖_F = %.3e\n', norm(U0*L_K - K, 'fro'));

else
    %% ========= B) CVX가 없을 때: 원하는 K를 정하고 식(10)으로 L_K 구하기 =========
    print('고정 K')
    % 아래 gain은 A+BK 가 stable
    K = [-1.5, -1.2];      % 크기: 1 x n

    Z  = [U0; X0];         % 크기: (m+n) x T
    LI = [K; eye(n)];      % 크기: (m+n) x n
    L_K = pinv(Z) * LI;    % 최소제곱 해 (정리1 조건(8)이면 해 존재)

    % 식(9): 폐루프 시스템 행렬
    A_cl = X1 * L_K;
end

A_bar = X1*pinv(X0)
B_bar = X1*(eye(T) - pinv(X0)*X0)

desired_poles = [0.5, 0.4]

w = place(A_bar, B_bar, desired_poles);


%% ========= 폐루프 시뮬 (데이터 기반 모형) =========
% 동역학: x_{k+1} = A_cl * x_k
% 입력:   u_k     = U0*L_K * x_k   (또는 u_k = K*x_k)
K_from_LK = U0 * L_K;   % 검산용 (CVX 경로면 K와 동일해야 함)

Nsim = length(Tspan);
x_sim = zeros(n, Nsim);
u_sim = zeros(m, Nsim);
x_sim(:,1) = x0;

for k = 1:Nsim-1
    u_sim(:,k) = K_from_LK * x_sim(:,k);    % = U0*L_K*x
    x_sim(:,k+1) = A_cl * x_sim(:,k);
end
u_sim(:,Nsim) = K_from_LK * x_sim(:,Nsim);

% 고유치(극) 확인 (단위원 내부면 안정)
eigAcl = eig(A_cl);
disp('eig(A_cl) =');
disp(eigAcl.');

%% ========= 결과 플롯 =========
figure;
subplot(3,1,1);
plot(Tspan, x_sim(1,:), 'LineWidth', 1.5); grid on;
xlabel('Time [s]'); ylabel('x_1');
title('데이터 주도 폐루프: 상태 x_1');

subplot(3,1,2);
plot(Tspan, x_sim(2,:), 'LineWidth', 1.5); grid on;
xlabel('Time [s]'); ylabel('x_2');
title('데이터 주도 폐루프: 상태 x_2');

subplot(3,1,3);
stairs(Tspan, u_sim, 'LineWidth', 1.5); grid on;
xlabel('Time [s]'); ylabel('u');
title('데이터 주도 폐루프: 입력 u = U_0 L_K x ( = Kx )');

%% ========= (선택) 실제 식별모델(Ad,Bd)과 비교 시뮬 =========
% 동일 K를 써서 "전통적" 폐루프도 확인해 볼 수 있음: x_{k+1} = (Ad+Bd*K)*x_k
Acl_id = Ad + Bd*K_from_LK;   % 주의: 부호 약속 (본 논문은 u=Kx)
eigAcl_id = eig(Acl_id);
fprintf('eig(Ad+Bd*K) = ['); fprintf(' %.3f%+.3fi', [real(eigAcl_id), imag(eigAcl_id)].'); fprintf(' ]\n');

x_id = zeros(n, Nsim);
x_id(:,1) = x0;
for k = 1:Nsim-1
    x_id(:,k+1) = Acl_id * x_id(:,k);
end

figure;
plot(Tspan, x_sim(1,:), '-',  'LineWidth', 1.5); hold on;
plot(Tspan, x_id(1,:),  '--', 'LineWidth', 1.2);
grid on; xlabel('Time [s]'); ylabel('x_1');
legend('데이터주도 폐루프 (X_1 L_K)', '식별모델 폐루프 (Ad+Bd K)');
title('x_1 응답 비교');
