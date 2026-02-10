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

% (A0,B0,C): continuous-time system
A0 = [-a1/A1 * 2*g / (2*sqrt(2*g*h10)), 0 , a3/A1 * 2*g / (2*sqrt(2*g*h30)), 0;...
      0, -a2/A2 * 2*g / (2*sqrt(2*g*h20)), 0 , -a4/A2 * 2*g / (2*sqrt(2*g*h40));...
      0, 0, -a3/A3 * 2*g / (2*sqrt(2*g*h30)), 0 ;...
      0, 0, 0, -a4/A4 * 2*g / (2*sqrt(2*g*h40))];
B0 = [g1*k1/A1, 0;...
      0, g2*k2/A2;...
      0, (1-g2)*k2/A3;...
      (1-g1)*k1/A4,0];
C = [kc, 0, 0, 0;...
     0, kc, 0, 0];

% sampling time
Ts = 0.1;

% discretize
sysC = ss(A0,B0,C,[]);
sysD = c2d(sysC, Ts);
A = sysD.A;
B = sysD.B;

% dimensions
[n,m] = size(B);
[l,~] = size(C);

% controller design
Q = eye(n);
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

% plant initial state
xp0 = ones(n,1);
% controller initial state
xc0 = [0.5; 0.02; -1; 0.9];

%%%% controller conversion %%%%
% observability matrix
On = obsv(F,H);

% Toeplitz matrix
Tn = zeros(m, n*l);
for i = 1:n-1
    tmp = [H*F^(i-1)*G, Tn(m*(i-1)+1:end,1:l*(n-1))];
    Tn = [Tn; tmp];
end

% (flipped) controllability matrix [F^(n-1)G, ..., FG, G]
Cn = F^(n-1)*G;
for i = 2:n
    Cn = [Cn, F^(n-i)*G];
end

% converted form: u(k)=Hu*[u(k-n);...;u(k-1)]+Hy*[y(k-n);...;y(k-1)]
Hu = H*F^n*pinv(On);
Hy = H*(Cn - F^n*pinv(On)*Tn);

% vectorization for proposed controller
% (with padded zeros when m!=l)
h = max(m,l);
HHu = zeros(m,m,n);
HHy = zeros(m,l,n);
for i = 1:n
    HHu(:,:,i) = Hu(:,m*(i-1)+1:m*i);
    HHy(:,:,i) = Hy(:,l*(i-1)+1:l*i);
end
vecHu = zeros(h*m,n);
vecHy = zeros(h*m,n);
for i = 1:n
    for j = 1:m
        vecHu(h*(j-1)+1:h*j,i) = [HHu(j,:,i).'; zeros(h-m,1)];
        vecHy(h*(j-1)+1:h*j,i) = [HHy(j,:,i).'; zeros(h-l,1)];
    end
end

%%%% find initial input-output trajectory %%%%
% yini = [y(-n);...;y(-1)], uini = [u(-n);...;u(-1)]
yini = Cn\xc0;
uini = Tn*yini;
Yini = reshape(yini,[],n);
Uini = reshape(uini,[],n);

%% Simulation
iter = 500;

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
r = 0.0002;
s = 0.0001;

% quantization of control parameters
qHu = round(Hu/s);
qHy = round(Hy/s);

% variables for simulation with converted & quantized controller
qXp = xp0;
qU = round(Uini/r);
qY = round(Yini/r);
rY = [];
rU = [];

for i = 1:iter
    % plant + original controller
    y = [y, C*xp(:,i)];
    u = [u, H*xc(:,i)];
    xp = [xp, A*xp(:,i) + B*u(:,i)];
    xc = [xc, F*xc(:,i) + G*y(:,i)];

    % plant + converted controller
    U = [U,Hu*reshape(U(:,end-n+1:end),[],1)+Hy*reshape(Y(:,end-n+1:end),[],1)];
    Y = [Y,C*Xp(:,i)];
    Xp = [Xp,A*Xp(:,i)+B*U(:,end)];

    % plant + quantized controller
    rU = [rU,r*s*(qHu*reshape(qU(:,end-n+1:end),[],1)+qHy*reshape(qY(:,end-n+1:end),[],1))];
    rY = [rY,C*qXp(:,i)];
    qY = [qY,round(rY(:,end)/r)];
    qU = [qU,round(rU(:,end)/r)];
    qXp = [qXp,A*qXp(:,i)+B*rU(:,end)];
end

figure(1)
plot(Ts*(0:iter-1), u)
hold on
plot(Ts*(0:iter-1), U(:,n+1:end))
hold on
plot(Ts*(0:iter-1), rU)
title('Control input u')
legend('original 1', 'original 2', 'converted 1', 'converted 2', 'quantized 1', 'quantized 2')

figure(2)
plot(Ts*(0:iter-1), y)
hold on
plot(Ts*(0:iter-1), Y(:,n+1:end))
hold on
plot(Ts*(0:iter-1), rY)
title('Plant output y')
legend('original 1', 'original 2', 'converted 1', 'converted 2', 'quantized 1', 'quantized 2')
