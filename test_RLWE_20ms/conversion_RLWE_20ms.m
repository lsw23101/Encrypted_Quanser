
%% 연속시간 시스템 모델
A = [ 0         0    1.0000         0;
      0         0         0    1.0000;
      0  149.2751   -0.0104         0;
      0  261.6091   -0.0103         0 ];

B = [ 0;
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
[nx,nu] = size(B);
[ny,~] = size(C);

% controller design
%% LQR 가중치 

Q = [50 0 0 0; 0 10 0 0; 0 0 0 0; 0 0 0 0];
R = 1;

%% 상태피드백 이득 K (discrete LQR)
[K, S, cl_poles] = dlqr(Ad, Bd, Q, R);
K=-K;

obs_poles = [0.31 0.35 0.33 0.34];            % 크기를 절반으로 줄인 극들
L = place(Ad', Cd', obs_poles).';      % 듀얼 시스템에 극배치 → 전치

% (F,G,H): resulting controller
F = A + B*K - L*C;
G = L;
H = K;

% plant initial state
xp0 = [0.1; 0.1; 0.1; 0.1];
% controller initial state
xc0 = [0; 0; 0; 0];

%%%% controller conversion %%%%
% observability matrix
On = obsv(F,H);

% Toeplitz matrix
Tn = zeros(nu, nx*ny);
for i = 1:nx-1
    tmp = [H*F^(i-1)*G, Tn(nu*(i-1)+1:end,1:ny*(nx-1))];
    Tn = [Tn; tmp];
end

% (flipped) controllability matrix [F^(n-1)G, ..., FG, G]
Cn = F^(nx-1)*G;
for i = 2:nx
    Cn = [Cn, F^(nx-i)*G];
end

% converted form: u(k)=Hu*[u(k-n);...;u(k-1)]+Hy*[y(k-n);...;y(k-1)]
Hu = H*F^nx*pinv(On);
Hy = H*(Cn - F^nx*pinv(On)*Tn);

% vectorization for proposed controller
% (with padded zeros when nu!=ny)
h = max(nu,ny);
HHu = zeros(nu,nu,nx);
HHy = zeros(nu,ny,nx);
for i = 1:nx
    HHu(:,:,i) = Hu(:,nu*(i-1)+1:nu*i);
    HHy(:,:,i) = Hy(:,ny*(i-1)+1:ny*i);
end
vecHu = zeros(h*nu,nx);
vecHy = zeros(h*nu,nx);
for i = 1:nx
    for j = 1:nu
        vecHu(h*(j-1)+1:h*j,i) = [HHu(j,:,i).'; zeros(h-nu,1)];
        vecHy(h*(j-1)+1:h*j,i) = [HHy(j,:,i).'; zeros(h-ny,1)];
    end
end

%%%% find initial input-output trajectory %%%%
% yini = [y(-n);...;y(-1)], uini = [u(-n);...;u(-1)]
yini = Cn\xc0;
uini = Tn*yini;
Yini = reshape(yini,[],nx);
Uini = reshape(uini,[],nx);

vecHy_T = vecHy';
vecHu_T = vecHu';
Yini_T = Yini';
Uini_T = Uini';

%% Simulation
iter = 100;

% variables for simulation with original controller
xp = xp0;
xc = xc0;
u = [];
y = [];

% variables for simulation with converted controller
Xp = xp0;
U = Uini;
Y = Yini;

% quantization parameters
L = 0.0001;
s = 0.0001;


% quantization of control parameters
qHu = round(Hu/s);
qHy = round(Hy/s);

% variables for simulation with converted & quantized controller
qXp = xp0;
qU = round(Uini/L);
qY = round(Yini/L);
rY = [];
rU = [];

for i = 1:iter
    % plant + original controller
    y = [y, C*xp(:,i)];
    u = [u, H*xc(:,i)];
    xp = [xp, A*xp(:,i) + B*u(:,i)];
    xc = [xc, F*xc(:,i) + G*y(:,i)];

    % plant + converted controller
    U = [U,Hu*reshape(U(:,end-nx+1:end),[],1)+Hy*reshape(Y(:,end-nx+1:end),[],1)];
    Y = [Y,C*Xp(:,i)];
    Xp = [Xp,A*Xp(:,i)+B*U(:,end)];

    % plant + quantized controller
    rU = [rU,L*s*(qHu*reshape(qU(:,end-nx+1:end),[],1)+qHy*reshape(qY(:,end-nx+1:end),[],1))];
    rY = [rY,C*qXp(:,i)];
    qY = [qY,round(rY(:,end)/L)];
    qU = [qU,round(rU(:,end)/L)];
    qXp = [qXp,A*qXp(:,i)+B*rU(:,end)];
end

figure(1)
plot(Ts*(0:iter-1), u)
hold on
%plot(Ts*(0:iter-1), U(:,nx+1:end))
hold on
%plot(Ts*(0:iter-1), rU)
hold on
title('Control input u')
legend('original 1', 'original 2', 'converted 1', 'converted 2', 'quantized 1', 'quantized 2')

figure(2)
plot(Ts*(0:iter-1), y)
hold on
%plot(Ts*(0:iter-1), Y(:,nx+1:end))
hold on
%plot(Ts*(0:iter-1), rY)
hold on
title('Plant output y')
legend('original 1', 'original 2', 'converted 1', 'converted 2', 'quantized 1', 'quantized 2')

rUmax = max(rU) * (1/(s*L));