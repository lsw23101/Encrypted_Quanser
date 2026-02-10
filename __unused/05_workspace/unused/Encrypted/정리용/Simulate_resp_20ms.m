%% ------------------------------------------------------------
%  Discrete-time plant (Ad,Bd,Cd,Dd) and controller (F,G,H) given
%  Impulse response of FULL closed-loop (plant + observer + state feedback)
%  Input: exogenous w at plant input (additive at actuator)
%  Output: y = [x; phi]
%  Ts must match your design sampling time
%% ------------------------------------------------------------

clear; clc; close all;

%% (1) 플랜트(이산) 정의: 네가 사용한 값으로 채워 넣으세요
Ts = 0.02;  % sampling time [s]

% 예시 (너의 설계에서 나온 ds_sys 사용)
Ad = [ 1.0000    0.1970    0.04999   0.003213;
       0         1.3450   -0.00001359 0.05563;
       0         8.3020    0.9994     0.1970;
       0        14.5500   -0.0005729  1.3450 ];

Bd = [ 0.0641;
       0.06485;
       2.6440;
       2.7340 ];

Cd = [1 0 0 0;
      0 1 0 0];

Dd = zeros(2,1);   % 반드시 [2x1]

% ---- (주의) 위의 Ad,Bd,Cd,Dd는 너의 실제 ds_sys 결과로 교체해도 됨 ----

%% (2) 옵저버 기반 동적 제어기 (너가 이미 구한 행렬 사용)
% xhat_{k+1} = F*xhat_k + G*y_k
% u_ctrl     = H*xhat_k
%
% (예시) 너의 코드에서 print_numpy로 출력했던 F,G,H를 그대로 붙여 넣어라.
% 여기엔 placeholder를 넣어둘게. 아래를 네 값으로 바꿔 넣으세요.
F = [ 0.0408  -0.4549   0.0352  -0.0241;
     -0.0901  -0.5925   0.0150  -0.0038;
     -7.3139  -33.9020  2.5236  -2.4129;
      1.9006  -50.3063  1.5173  -1.3801 ];

G = [ 1.0157   0.1971;
      0.1462   1.3597;
     12.9934   8.0088;
      3.7554  26.8191 ];

H = [ 5.6553  -28.8075  1.5173  -2.4326 ];   % row vector (1x4)

% 차원 체크
nx = size(Ad,1);          % plant states
ny = size(Cd,1);          % outputs (x, phi)
nu = size(Bd,2);          % inputs (1)
nxh = size(F,1);          % observer states (should be 4 here)
assert(nu==1 && nx==nxh && ny==2, '차원 확인 필요');

%% (3) 폐루프 증분입력 w에 대한 결합 행렬 구성
% Plant:  x_{k+1}   = Ad*x_k + Bd*(u_ctrl + w_k)
% y_k    = Cd*x_k            (Dd=0)
% Ctrl:   xhat_{k+1} = F*xhat_k + G*y_k = F*xhat + G*Cd*x
% u_ctrl = H*xhat
%
% Augmented state z = [x; xhat]
% z_{k+1} = [ Ad,   Bd*H ; G*Cd,  F ] * z  + [ Bd ; 0 ] * w
% y       = [ Cd,   0 ] * z
A_aug = [ Ad,        Bd*H;
          G*Cd,      F    ];

B_aug = [ Bd;
          zeros(nxh,1) ];

C_aug = [ Cd, zeros(ny, nxh) ];
D_aug = zeros(ny,1);

sys_aug = ss(A_aug, B_aug, C_aug, D_aug, Ts, ...
    'statename', [compose("x_%d",1:nx), compose("xhat_%d",1:nxh)], ...
    'inputname', "w", ...
    'outputname', ["x (m)"; "phi (rad)"]);

%% (4) 임펄스 응답
% 방법 A: impulse (디스크리트 시스템에서 N 스텝 지정 가능)
N = 300; % 300 샘플 = 300*Ts 초
%% (임펄스 응답 계산)
N = 300;                          % 샘플 개수
[y_imp, t_imp] = impulse(sys_aug, N);   % t_imp와 y_imp는 서로 길이가 맞음

%% (MIMO 안전 추출: 단일 입력 가정 → 첫 번째 입력 채널 사용)
if ndims(y_imp) == 3
    y1 = squeeze(y_imp(:,1,1));   % 출력 1: x
    y2 = squeeze(y_imp(:,2,1));   % 출력 2: phi
else
    y1 = y_imp(:,1);
    y2 = y_imp(:,2);
end

%% (플롯: t_imp와 동일 길이의 y 사용)
figure('Color','w');
yyaxis left
plot(t_imp, y1, 'LineWidth', 1.5); grid on;
ylabel('cart position x (m)');

yyaxis right
plot(t_imp, y2, '--', 'LineWidth', 1.5); grid on;
ylabel('pendulum angle \phi (rad)');

xlabel('Time (s)');
title('Impulse Response of Full Closed-Loop (input: actuator disturbance w)');
legend({'x','\phi'}, 'Location','best');
%% (6) 안정성/극 확인 (선택)
ev_aug = eig(A_aug);
fprintf('max |eig(A_aug)| = %g\n', max(abs(ev_aug)));

% (원하면) 제어 입력 u_ctrl(t)도 확인
% u = H * xhat = [0 I]*z * H' 이므로 상태경로로 얻거나,
% state trajectory를 함께 받고 u를 재구성해서 플롯할 수 있음.
% 아래는 impulse(sys)에서 상태가 필요할 때 lsim으로 대체하는 예시:

% t = tsec;
% w = zeros(size(t)); w(1)=1;
% [y_l, t_l, z_l] = lsim(sys_aug, w, t);
% xhat_seq = z_l(:, nx+1:end);
% u_seq = (H * xhat_seq.').';    % (N x 1)
% figure('Color','w');
% plot(t, u_seq, 'LineWidth',1.5); grid on;
% xlabel('Time (s)'); ylabel('u_{ctrl} (V)'); title('Control input u_{ctrl} to impulse on w');

