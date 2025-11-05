clear all;
clc;


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
Ts = 0.02; % 20

% discretize
sysC = ss(A0,B0,C,[]);
sysD = c2d(sysC, Ts);
A = sysD.A;
B = sysD.B;


% controller design
%% LQR 가중치 

Q = [50 0 0 0; 0 10 0 0; 0 0 0 0; 0 0 0 0];
R = 1;

% 상태피드백 이득 K (discrete LQR)
[K, S, cl_poles] = dlqr(A, B, Q, R);

K= -K;

obs_poles = [0.31 0.32 0.33 0.34];            % 크기를 절반으로 줄인 극들
L = place(A', C', obs_poles).';      % 듀얼 시스템에 극배치 → 전치


% controller design
% Q = eye(n);
% R1 = eye(m);
% R2 = eye(l);

% [~, K, ~] = idare(A,B,Q,R,[],[]);
% K = -K;
% [~, L, ~] = idare(A.', C.', Q, R, [], []);
% L = L.';





% (F,G,H): resulting controller
F = A + B*K - L*C;
G = L;
H = K;

% plant initial state
xp0 = [0.1; 0.1; 0.1; 0.1];
% controller initial state
xc0 = [0; 0; 0; 0];




%% Converting the state matrix into integers
% One may freely change F, G, and H to different systems as they choose
% Finds R such that (F-RH) is an integer matrix through pole-placement


% Assign integer poles to (F-RH)
poles = [0 1 2 -1]; % Must consist of n-integers!
R = place(F.',H.',poles).';


% Convert to modal canonical form
sys = ss(F-R*H, G, H, []);
[csys,T] = canon(sys, 'companion');
F_ = T*(F-R*H)/T
R_ = T*R
G_ = T*G
H_ = H/T






% Acl = [A B*K; L*C F];


% abs(eig(Acl))

% 위에서 F, G, H를 구한 다음:

% 위에서 F, G, H를 구한 다음:
print_numpy('F', F, 4);
print_numpy('G', G, 4);
print_numpy('H', H, 4);


print_go_slice2('F', round(F_), 0);   % ← '%.0f' 포맷으로 출력
print_go_slice2('G', G_, 4);
print_go_slice2('H', H_, 4);
print_go_slice2('R', R_, 4);


% 안정도 
abs(eig(A+B*K))
abs(eig(F))




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
                fprintf(' ]\n');
            end
        end
        fprintf('], dtype=np.float64)\n\n');
    end
end