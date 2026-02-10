%% Minimal ID from qube_log_data.csv: build U0, X0, X1 and estimate A,B
clear; clc;





% ===== 사용자 설정 =====
fname  = 'qube_log_data.csv';  % 같은 폴더
T      = 10000;                % 사용할 길이 (모자라면 자동 축소)
% =======================

% 1) CSV 로드
tbl = readtable(fname);
need_cols = {'timeStamp_s','u_noisy_V','theta_rad','alpha_rad','theta_dot_rad_s','alpha_dot_rad_s'};
if ~all(ismember(need_cols, tbl.Properties.VariableNames))
    error('CSV 컬럼명이 예상과 다릅니다. 필요한 컬럼: %s', strjoin(need_cols, ', '));
end

t    = tbl.timeStamp_s;
u    = tbl.u_noisy_V;
Xall = [tbl.theta_rad, tbl.alpha_rad, tbl.theta_dot_rad_s, tbl.alpha_dot_rad_s];  % 4 states

N = height(tbl);
if N < 10, error('데이터가 너무 적습니다. N=%d', N); end

% 2) 앞/뒤 1/4 제거 후 가운데 구간 선택
q = floor(N/4);
idx_mid = (q+1):(N-q);

t_mid = t(idx_mid);
u_mid = u(idx_mid);
X_mid = Xall(idx_mid, :);          % (Nmid x 4)

% 3) 가운데에서 T+1 길이 확보 (부족하면 자동 축소)
avail = length(t_mid);
if avail < 2, error('가운데 구간이 너무 짧습니다. avail=%d', avail); end
T_effective = min(T, avail - 1);

if T_effective < T
    warning('요청한 T=%d → 사용가능 T=%d 로 자동 조정합니다. (가운데 길이=%d)', ...
        T, T_effective, avail);
end

idx_seg = 1:(T_effective+1);
u_seg   = u_mid(idx_seg);          % (T_effective+1) x 1
X_seg   = X_mid(idx_seg, :);       % (T_effective+1) x 4

% 4) X0, X1, U0 구성
X0 = X_seg(1:end-1, :)';           % 4 x T_effective
X1 = X_seg(2:end,   :)';           % 4 x T_effective
U0 = u_seg(1:end-1, :)';           % 1 x T_effective

% 5) 최소제곱으로 A,B 추정
%    X1 = [B A] * [U0; X0]  →  [B A] = X1 * pinv([U0; X0])
Z  = [U0; X0];                     % (1+4) x T_effective
BA = X1 * pinv(Z);                 % 4 x 5

B_est = BA(:, 1);                  % 4 x 1
A_est = BA(:, 2:end);              % 4 x 4


%% ===== 추가: sys_orig vs sys_est 전달함수 비교 (Ts = 0.005 고정) =====

% 상태출력(C=I4, D=0)로 비교
C_tf = eye(4);
D_tf = zeros(4,1);



% 식별 이산 시스템 (A_est, B_est, Ts=0.005 사용)
sys_est  = ss(A_est, B_est, C_tf, D_tf, 0.005);


sys_cont_est = d2c(sys_est);

sys_dis_est = c2d(sys_cont_est, 0.025);

A = sys_dis_est.A;
B = sys_dis_est.B;

C = [1 0 0 0; 0 1 0 0]
% 모델 식별한걸로 해보기



% dimensions
[nx,nu] = size(B);
[ny,~] = size(C);

% controller design
%% LQR 가중치 

Q = [1000 0 0 0; 0 100 0 0; 0 0 1 0; 0 0 0 1];
R = 1;

%% 상태피드백 이득 K (discrete LQR)
[K, S, cl_poles] = dlqr(A, B, Q, R);
K=-K;

obs_poles = [0.31 0.32 0.33 0.34];            % 크기를 절반으로 줄인 극들
L = place(A', C', obs_poles).';      % 듀얼 시스템에 극배치 → 전치

% (F,G,H): resulting controller
F = A + B*K - L*C;
G = L;
H = K;

% plant initial state
xp0 = [0.1; 0.1; 0.1; 0.1];
% controller initial state
xc0 = [0; 0; 0; 0];


%% Simulation
iter = 100;

% variables for simulation with original controller
xp = xp0;
xc = xc0;
u = [];
y = [];



% 안정도 
abs(eig(A+B*K))
abs(eig(F))

% Acl = [A B*K; L*C F];


% abs(eig(Acl))

% 위에서 F, G, H를 구한 다음:
print_numpy('F', F, 4);
print_numpy('G', G, 4);
print_numpy('H', H, 4);







%% ============ Go [][]float64 프린터 (행렬) ============
function print_go_slice2(name, M, prec)
    if nargin < 3, prec = 4; end
    fmt = ['%0.' num2str(prec) 'f'];

    fprintf('%s := [][]float64{\n', name);
    for i = 1:size(M,1)
        fprintf('    { ');
        for j = 1:size(M,2)
            fprintf(fmt, M(i,j));
            if j < size(M,2), fprintf(', '); end
        end
        if i < size(M,1)
            fprintf(' },\n');
        else
            fprintf(' }\n');
        end
    end
    fprintf('}\n\n');
end

%% ============ Go []float64 프린터 (벡터) ============
function print_go_slice1(name, v, prec)
    if nargin < 3, prec = 4; end
    fmt = ['%0.' num2str(prec) 'f'];

    v = v(:).'; % row vector
    fprintf('%s := []float64{ ', name);
    for k = 1:numel(v)
        fprintf(fmt, v(k));
        if k < numel(v), fprintf(', '); end
    end
    fprintf(' }\n\n');
end

function print_numpy(name, M, prec)
    if nargin < 3, prec = 4; end
    fmt = ['%0.' num2str(prec) 'f'];

    if isvector(M)
        % 1D 벡터는 한 줄로 출력 (예: H)
        fprintf('%s = np.array([ ', name);
        for k = 1:numel(M)
            fprintf(fmt, M(k));
            if k < numel(M), fprintf(', '); end
        end
        fprintf(' ], dtype=np.float64)  # shape (%d,)\n\n', numel(M));
    else
        % 2D 행렬은 행 단위로 출력 (예: F, G)
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
                fprintf(' ],\n');
            end
        end
        fprintf('], dtype=np.float64)\n\n');
    end
end