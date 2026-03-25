%% Aero 2 — 4th-Order MIMO: Feedback Linearization + LQR Simulation
clear; clc; close all;

%% 1. 시스템 파라미터 정의 (논문 Table 3.1 & 3.2 기반)
% 기본 물리량
M1 = 0.146; M2 = 0.2; M3 = 0.02; L1 = 0.168; 
R = 0.077; h = 0.03; Jp = 0.0232; Jy = 0.0238; Jm = 4.0e-6;
Mb = 1.07; Dm = 2.42e-3; Ms = 0.526; Ls = 0.305; g = 9.81;

% 마찰 계수
Bqv = 0.017; Bqh = 0.0227; 

% 비선형 항 상쇄를 위한 alpha 파라미터 계산
alpha1 = 2*(M1+M2)*L1^2 + 2*M3*L1^2 + M1*(R^2/4 + h^2/12) + M1*R^2/2 + Jp + 2*Jm;
alpha2 = 2*(M1+M2)*L1^2 + 2*M3*L1^2 + M1*R^2/2 + Jy - M1*(R^2/4 + h^2/12);
alpha3 = 2*(M1+M2)*L1^2 + 2*M3*L1^2 + M1*R^2/2 + Jy;
alpha4 = M1*(R^2/4 + h^2/12);
alpha5 = M1*(R^2/4 + h^2/12) + Ms/12*Ls^2 + 2*Jm;
alpha6 = M1*(R^2/2 + h^2/6) - 4*(M1+M2)*L1^2 - 4*M3*L1^2 - M1*R^2 - 2*Jy;

%% 2. 제어기 설계 (LQR for Double Integrator)
% 피드백 선형화 후 시스템은 ddot(q) = v 형태가 됨
Ad_fl = [0, 0, 1, 0; 0, 0, 0, 1; 0, 0, 0, 0; 0, 0, 0, 0];
Bd_fl = [0, 0; 0, 0; 1, 0; 0, 1];

% 논문에서 계산된 LQR Gain 및 Tracking Gain
K = [22.3607, 0, 6.6874, 0; 0, 22.3607, 0, 6.6874];
kstar = [22.3607, 0; 0, 22.3607];

%% 3. 시뮬레이션 설정
Ts = 0.02; T_final = 10;
t = 0:Ts:T_final;
N_sim = length(t);

x = zeros(4, N_sim);          % 상태: [pitch, yaw, pitch_rate, yaw_rate]
u_volts = zeros(2, N_sim);    % 출력 전압: [V_main, V_tail]

% 목표 각도 설정: 10도, 10도
ref = deg2rad([10; -10]);

%% 4. 시뮬레이션 루프
for k = 1:N_sim-1
    % 현재 상태 추출
    q_v = x(1, k); q_h = x(2, k);
    dq_v = x(3, k); dq_h = x(4, k);
    
    % (1) 가상 입력 v 계산 (LQR + Tracking)
    % v = -K*x + kstar*ref
    v = -K * x(:, k) + kstar * ref;
    v1 = v(1); v2 = v(2);
    
    % (2) 피드백 선형화: 비선형 항 상쇄 및 목표 토크(Gamma) 계산
    Gamma_qv = alpha2*sin(q_v)*cos(q_v)*dq_h^2 + Mb*g*Dm*sin(q_v) + Bqv*dq_v + alpha1*v1;
    Gamma_qh = alpha6*sin(q_v)*cos(q_v)*dq_v*dq_h + Bqh*dq_h + (alpha3*cos(q_v)^2 + alpha4*sin(q_v)^2 + alpha5)*v2;
    
    % (3) 토크(Gamma) -> 추력(T) 변환
    % D(q) matrix: [Lm, -k; -k*cos(qv), Lt*cos(qh)]
    k_torque = 0.02; % Reactive torque constant
    D_q = [L1, -k_torque; -k_torque*cos(q_v), L1*cos(q_h)];
    T_d = D_q \ [Gamma_qv; Gamma_qh]; % T_d = [T_main; T_tail]
    
    % (4) 추력(T) -> 전압(V) 변환 (Steady-state approximation)
    % T = Cm * w^3 -> w = (T/Cm)^(1/3)
    % V = 0.058*w + 0.0789
    Cm = 2.047e-7;
    w_m = sign(T_d(1)) * (abs(T_d(1))/Cm)^(1/3);
    w_t = sign(T_d(2)) * (abs(T_d(2))/Cm)^(1/3);
    
    V_m = 0.058 * w_m + 0.0789;
    V_t = 0.058 * w_t + 0.0789;
    
    % 전압 제한 (Safety)
    u_volts(:, k) = max(min([V_m; V_t], 18), -18);
    
    % (5) 실제 비선형 시스템 업데이트 (Plant Dynamics)
    % 시뮬레이션 성능 확인을 위해 가속도 산출
    ddq_v = (Gamma_qv - alpha2*sin(q_v)*cos(q_v)*dq_h^2 - Mb*g*Dm*sin(q_v) - Bqv*dq_v) / alpha1;
    ddq_h = (Gamma_qh - alpha6*sin(q_v)*cos(q_v)*dq_v*dq_h - Bqh*dq_h) / (alpha3*cos(q_v)^2 + alpha4*sin(q_v)^2 + alpha5);
    
    % Euler integration
    x(3, k+1) = x(3, k) + ddq_v * Ts;
    x(4, k+1) = x(4, k) + ddq_h * Ts;
    x(1, k+1) = x(1, k) + x(3, k+1) * Ts;
    x(2, k+1) = x(2, k) + x(4, k+1) * Ts;
end

%% 5. 결과 시각화
figure('Color', 'w', 'Position', [100, 100, 1000, 500]);
subplot(1,2,1);
plot(t, rad2deg(x(1,:)), 'b', t, rad2deg(x(2,:)), 'g', 'LineWidth', 1.5);
yline(10, 'r--', 'Pitch Ref (10 deg)');
yline(10, 'm--', 'Yaw Ref (10 deg)');
title('Position Response with Feedback Linearization');
xlabel('Time (s)'); ylabel('Angle (deg)'); legend('Pitch', 'Yaw'); grid on;

subplot(1,2,2);
plot(t, u_volts(1,:), 'r', t, u_volts(2,:), 'k');
title('Control Action (Motor Voltages)');
xlabel('Time (s)'); ylabel('Voltage (V)'); legend('V\_main', 'V\_tail'); grid on;