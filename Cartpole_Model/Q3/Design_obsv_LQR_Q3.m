clear;
clc;

%% 연속시간 시스템 모델
A = [ 0         0    1.0000         0;
      0         0         0    1.0000;
      0  28.4817   -2.1911         -0.0938;
      0  89.5956  -1.8780         -0.2950 ];

B = [ 0;
      0;
  12.3119;
  10.5530 ];

C = [1 0 0 0;
     0 1 0 0];
%% (수정) 출력이 2개이므로 D 크기 수정
D = zeros(2,1);

%% 연속 → 이산 (ZOH, Ts = 0.02 s)
Ts = 0.02;
ct_sys = ss(A,B,C,D);
ds_sys = c2d(ct_sys, Ts, 'zoh');
[Ad, Bd, Cd, Dd] = ssdata(ds_sys);

%% LQR 가중치 

Q = [50 0 0 0; 0 10 0 0; 0 0 0 0; 0 0 0 0];
R = 1;

%% 상태피드백 이득 K (discrete LQR)
[K, S, cl_poles] = dlqr(Ad, Bd, Q, R);
K=-K;

%% 옵저버 이득 L 설계 (극배치 예시)
% “더 빠른” 옵저버를 위해 폐루프 극의 크기를 절반으로 축소해서 사용
% (필요에 따라 직접 원하는 극들을 지정하셔도 됩니다.)
obs_poles = [0.31 0.35 0.33 0.34];            % 크기를 절반으로 줄인 극들
L = place(Ad', Cd', obs_poles).';      % 듀얼 시스템에 극배치 → 전치


% % Qe, Re는 조정 파라미터입니다. (예시는 꽤 빠른 옵저버)
% Qe = diag([100, 100, 0, 0]);   % 상태(추정오차) 가중 (크면 보정이 공격적/빠름)
% Re = diag([1, 1]);           % 측정 보정 가중 (크면 보정을 덜 함 = 더 느림/부드러움)
% 
% Lt = dlqr(Ad', Cd', Qe, Re); % Lt: 2x4
% L  = Lt';                    % L: 4x2
% 



%% 동적 컨트롤러 실현 (Observer-based)
F = Ad + Bd*K - L*Cd;
G = L;
H = K;

%% 결과 확인
disp('F ='); disp(F);
disp('G ='); disp(G);
disp('H ='); disp(H);

% (참고) 구현 시:
% xhat_{k+1} = F*xhat_k + G*y_k
% u_k        = H*xhat_k

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
                fprintf(' ]\n');
            end
        end
        fprintf('], dtype=np.float64)\n\n');
    end
end

% 안정도 
abs(eig(Ad+Bd*K))
abs(eig(F))

Acl = [Ad Bd*K; L*Cd F];


abs(eig(Acl))

% 위에서 F, G, H를 구한 다음:
print_numpy('F', F, 4);
print_numpy('G', G, 4);
print_numpy('H', H, 4);



