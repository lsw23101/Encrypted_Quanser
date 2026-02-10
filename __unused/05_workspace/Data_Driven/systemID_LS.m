%% Data-Driven Output Feedback Control Design (De Persis & Tesi, 2020)
% 논문의 식 (79)-(81) 기반 설계
clear; clc;

% cd 'C:\Users\publi\Documents\cvx'  % 폴더 이름이 cvx가 맞는지 확인하세요
% cvx_setup

%% Data-Driven Output Feedback Control Design (De Persis & Tesi, 2020)
clear; clc;

%% 1. 데이터 로드
filename = 'qube_data.csv'; 
data = readtable(filename);
valid_data = data(501:end, :); 

u_data = valid_data.u_noisy'; % (m x N)
y_data = [valid_data.theta, valid_data.alpha]'; % (p x N)

N_total = length(u_data);
m = 1; % 입력 개수
p = 2; % 출력 개수
n = 4; % 시스템 차수 (가정)

% 논문의 상태 정의: x_hat(k) = [u(k-1); ...; u(k-n); y(k-1); ...; y(k-n)]
nh = (m+p)*n; % 확장된 상태 차원 (여기서는 3*4 = 12)

%% 2. 데이터 행렬 (X0_hat, X1_hat, U0_hat) 구성
% 논문의 Lemma 1 및 Theorem 8 구조에 맞춤
T = 1000; % 데이터 열 수 (PE 조건 만족을 위해 충분히 크게)
if T > N_total - n - 1, T = N_total - n - 1; end

X0_hat = zeros(nh, T);
X1_hat = zeros(nh, T);
U0_hat = zeros(m, T);

for k = 1:T
    % x_hat(k) 구성
    idx0 = k : k+n-1;
    idx1 = k+1 : k+n;
    
    % 과거 입력/출력 데이터를 거꾸로 또는 순서대로 쌓음 (논문 식 66-67 참고)
    % 여기서는 [u(k-1)...u(k-n); y(k-1)...y(k-n)] 구조
    X0_hat(:, k) = [reshape(u_data(:, idx0), [], 1); reshape(y_data(:, idx0), [], 1)];
    X1_hat(:, k) = [reshape(u_data(:, idx1), [], 1); reshape(y_data(:, idx1), [], 1)];
    
    % 현재 입력 u(k)
    U0_hat(:, k) = u_data(:, k+n);
end

%% 3. CVX를 이용한 LMI 최적화 (논문 식 74)
cvx_begin sdp
    variable Q(T, nh) % Q는 T x nh 크기여야 함
    
    % Lyapunov 행렬 Z = X0_hat * Q (nh x nh)
    Z = X0_hat * Q;
    
    % 목적 함수: 제어 이득의 크기 최소화 (정규화)
    minimize(norm(Q, 'fro'))
    
    subject to
        % 식 (74): Lyapunov 안정성 조건 (Schur Complement 형태)
        Z == Z'; % 대칭 조건 (P^-1은 대칭이어야 함)
        [Z, X1_hat * Q; (X1_hat * Q)', Z] >= 1e-6 * eye(2*nh);
        
        % Z는 양의 정부호 행렬이어야 함 (안정성)
        Z >= 1e-6 * eye(nh);
cvx_end

%% 4. 제어기 추출 및 상태공간 모델 생성 (수정본)
if strcmp(cvx_status, 'Solved') || strcmp(cvx_status, 'Inaccurate/Solved')
    % 샘플링 타임 정의 (에러 해결을 위해 명시)
    Ts = 0.02; 
    
    % 제어 이득 K 계산 (u = K * x_hat)
    K = U0_hat * Q / Z; 
    
    fprintf('제어기 설계 성공! K 행렬 크기: %d x %d\n', size(K,1), size(K,2));
    
    % --- 제어기 상태공간 모델 구성 (Ac, Bc, Cc, Dc) ---
    Ku = K(:, 1:m*n);      % 과거 입력 게인 (1 x 4)
    Ky = K(:, m*n+1:end);  % 과거 출력 게인 (1 x 8)
    
    % 1. Ac 구성 (상태 업데이트 행렬)
    % u_hist (n개)와 y_hist (n개)의 관계를 정의
    % Ac_u: u_hist의 shift 및 새로운 u(k) 계산
    Au_top = Ku; % 새로운 u(k)
    Au_bot = [eye(m*(n-1)), zeros(m*(n-1), m)]; 
    Au = [Au_top; Au_bot];
    
    % Ac_y: y_hist의 shift (새로운 y(k)는 입력 Bc를 통해 들어옴)
    Ay = [zeros(p, p*n); 
          eye(p*(n-1)), zeros(p*(n-1), p)];
    
    % 전체 Ac (nh x nh)
    % nh = 12 (m*n + p*n = 4 + 8)
    Ac = [Au, [Ky; zeros(m*(n-1), p*n)];
          zeros(p*n, m*n), Ay];
    
    % 2. Bc 구성 (측정값 y(k)가 제어기 내부 상태로 들어가는 경로)
    % x_hat(k+1)에서 y(k)는 y_hist의 첫 번째 칸에 배치됨
    Bc = [zeros(m*n, p); 
          eye(p); 
          zeros(p*(n-1), p)];
    
    % 3. Cc, Dc 구성 (u(k) 출력)
    Cc = K;
    Dc = zeros(m, p);
    
    % 최종 상태공간 제어기 생성
    sys_ctrl = ss(Ac, Bc, Cc, Dc, Ts);
    
    % 결과 확인
    disp('상태공간 제어기(sys_ctrl)가 성공적으로 생성되었습니다.');
    disp('제어기 극점(Poles):');
    disp(eig(Ac)); 
    
else
    error('LMI 해를 찾지 못했습니다.');
end