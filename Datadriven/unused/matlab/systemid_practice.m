%% Minimal ID from qube_log_data.csv: build U0, X0, X1 and estimate A,B
clear; clc;



%% 연속시간 시스템 모델
A0 = [ 0         0    1.0000         0;
      0         0         0    1.0000;
      0  149.2751   -0.0104         0;
      0  261.6091   -0.0103         0 ];

B0 = [ 0;
      0;
  49.7275;
  49.1493 ];

C = [1 0 0 0;
     0 1 0 0];

% sampling time
Ts = 0.005; % 5ms

% discretize
sysC = ss(A0,B0,C,[]);
sysD = c2d(sysC, Ts);
A = sysD.A;
B = sysD.B;



%% 




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

% 6) 결과 출력
disp('A_est ='); disp(A_est);
disp('A ='); disp(A);

disp('B_est ='); disp(B_est);
disp('B ='); disp(B);


%% ===== 추가: sys_orig vs sys_est 전달함수 비교 (Ts = 0.005 고정) =====

% 상태출력(C=I4, D=0)로 비교
C_tf = eye(4);
D_tf = zeros(4,1);

% 원래(참) 이산 시스템 (위에서 c2d로 얻은 A, B, Ts 사용)
sys_orig = ss(A, B, C_tf, D_tf, Ts);

% 식별 이산 시스템 (A_est, B_est, Ts=0.005 사용)
sys_est  = ss(A_est, B_est, C_tf, D_tf, Ts);



%% 제어기 설계

Q = [100 0 0 0; 0 10 0 0; 0 0 0 0; 0 0 0 0];
R = 1;

%% 상태피드백 이득 K (discrete LQR)
[K, S, cl_poles] = dlqr(A_est, B_est, Q, R);
K=-K


print_numpy('k', K, 4);

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
