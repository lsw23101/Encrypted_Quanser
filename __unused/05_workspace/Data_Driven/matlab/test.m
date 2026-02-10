clear all;
clc;


% %% 연속시간 시스템 모델
% A0 = [ 0         0    1.0000         0;
%       0         0         0    1.0000;
%       0  149.2751   -0.0104         0;
%       0  261.6091   -0.0103         0 ];
% 
% B0 = [ 0;
%       0;
%   49.7275;
%   49.1493 ];
% 
% C = [1 0 0 0;
%      0 1 0 0];
% 
% % 테스트 모델 깃헙 서보3
% A0 = [ 0         0    1.0000         0;
%       0         0         0    1.0000;
%       0  55.1525   -4.5471         -0.1816;
%       0  168.5810   -4.4942        -0.5551 ];
% 
% B0 = [ 0;
%       0;
%   20.6755;
%   20.4351 ];
% 
% C = [1 0 0 0;
%      0 1 0 0];



% 테스트 모델
A0 = [ -0.4302    0.7045   -0.3051     1.619;
      0.6343   -0.7904  -0.09874     1.194;
      -22.2     36.14    -65.62     81.33;
      63.14    -78.67    -9.967     19.47];

B0 = [ 0;
      0;
  20.6755;
  20.4351 ];

C = [1 0 0 0;
     0 1 0 0];




% sampling time
Ts = 0.025; % 5ms

% discretize
sysC = ss(A0,B0,C,[]);
sysD = c2d(sysC, Ts);
A = sysD.A;
B = sysD.B;


% 모델 식별한걸로 해보기



% dimensions
[nx,nu] = size(B);
[ny,~] = size(C);

% controller design
%% LQR 가중치 

Q = [100 0 0 0; 0 50 0 0; 0 0 1 0; 0 0 0 1];
R = 1;

%% 상태피드백 이득 K (discrete LQR)
[K, S, cl_poles] = dlqr(A, B, Q, R);
K=-K;

obs_poles = [0.21 0.22 0.23 0.24];            % 크기를 절반으로 줄인 극들
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



for i = 1:iter
    % plant + original controller
    y = [y, C*xp(:,i)];
    u = [u, H*xc(:,i)];
    xp = [xp, A*xp(:,i) + B*u(:,i)];
    xc = [xc, F*xc(:,i) + G*y(:,i)];

end

figure(1)
plot(Ts*(0:iter-1), u)
hold on
title('Control input u')


figure(2)
plot(Ts*(0:iter-1), y)
hold on
title('Plant output y')



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