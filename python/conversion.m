clear all;
clc;
close all;
%% Observer-based controller design
% Example: four-tank system
% parameters
A1 = 28; A2 = 32; A3 = 28; A4 = 32;
a1 = 0.071; a2 = 0.057; a3 = 0.071; a4 = 0.057;
kc = 0.5;
g = 981;
h10 = 12.4; h20 = 12.7; h30 = 1.8; h40 = 1.4;
v10 = 3; v20 = 3;
k1 = 3.33; k2 = 3.35;
g1 = 0.7; g2 = 0.6;

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
Ts = 0.02;

% discretize
sysC = ss(A0,B0,C,[]);
sysD = c2d(sysC, Ts);
A = sysD.A;
B = sysD.B;

% dimensions
[n,m] = size(B);
[l,~] = size(C);

% controller design
Q = [5000 0 0 0; 0 100 0 0; 0 0 0 0; 0 0 0 0];
R1 = eye(m);
R2 = eye(l);
[~, K, ~] = idare(A,B,Q,R1,[],[]);
K = -K;
[~, L, ~] = idare(A.', C.', Q, R2, [], []);
L = L.';

% (F,G,H): resulting controller
F = A + B*K - L*C;
G = L;
H = K;



%% Converting the state matrix into integers
% One may freely change F, G, and H to different systems as they choose
% Finds R such that (F-RH) is an integer matrix through pole-placement


% Assign integer poles to (F-RH)
poles = [0,1,2,-1]; % Must consist of n-integers!
R = place(F.',H.',poles);
R = R.';

% Convert to modal canonical form
sys = ss(F-R*H, G, H, []);
[csys,T] = canon(sys, 'modal');
F_ = T*(F-R*H)/T
R_ = T*R
G_ = T*G
H_ = H/T



%% 결과 확인
% 위에서 F, G, H를 구한 다음:


print_numpy('A', A, 4);
print_numpy('B', B, 4);
print_numpy('C', C, 4);






print_numpy('F', F_, 4);
print_numpy('G', G_, 4);
print_numpy('H', H_, 4);
print_numpy('R', R_, 4);




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






