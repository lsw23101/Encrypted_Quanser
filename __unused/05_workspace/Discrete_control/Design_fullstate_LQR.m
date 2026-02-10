
A = [ 0         0    1.0000         0;
      0         0         0    1.0000;
      0  149.2751   -0.0104         0;
      0  261.6091   -0.0103         0 ];

B = [ 0;
      0;
  49.7275;
  49.1493 ];

C = eye(4,4);

D = zeros(4,1);

% 시스템 객체로 정의하고 싶으면
ct_sys = ss(A,B,C,D);

%% 1) Discretize (ZOH, Ts = 0.05 s)
Ts = 0.02;
ds_sys = c2d(ct_sys, Ts, 'zoh');
[Ad, Bd, Cd, Dd] = ssdata(ds_sys);

%% 2) LQR Weights 

% (원하시면 직접 가중치 고정)
Q = diag([5000, 100, 0, 0]);
R = 1;

%% 3) Discrete LQR
[K, S, cl_poles] = dlqr(Ad, Bd, Q, R);

%% 4) 결과 확인
disp('Ad ='); disp(Ad);
disp('Bd ='); disp(Bd);
disp('LQR Gain K ='); disp(K);
disp('Closed-loop poles (discrete) ='); disp(cl_poles);