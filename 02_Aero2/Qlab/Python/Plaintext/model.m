%% Aero 2 — LQR Tracking Control (Based on R. Fellag et al., 2024)
clear; clc; close all;

%% 1. Parameters (From Table I of the paper)
% System constants
Jp = 0.0211;    % Pitch inertia (kg.m^2)
Jy = 0.0221;    % Yaw inertia (kg.m^2)
Dp = 0.0053;    % Pitch damping (N.m.s/rad)
Dy = 0.0062;    % Yaw damping (N.m.s/rad)
Mg = 0.0153;    % Gravity term (N.m)

% Thrust and Torque coefficients
Kpp =  0.0011;  % Pitch torque per Main Voltage (N.m/V)
Kyy =  0.0047;  % Yaw torque per Tail Voltage (N.m/V)
Kpy =  0.0021;  % Cross-coupling: Pitch torque per Tail Voltage
Kyp = -0.0027;  % Cross-coupling: Yaw torque per Main Voltage

%% 2. Continuous-Time State-Space Model (Linearized about 0 deg)
% State: x = [theta (pitch), psi (yaw), d_theta, d_psi]'
% Input: u = [Vm (Main Motor), Vt (Tail Motor)]'

A = [0,         0,  1,        0;
     0,         0,  0,        1;
    -Mg/Jp,     0, -Dp/Jp,    0;
     0,         0,  0,       -Dy/Jy];

B = [0,          0;
     0,          0;
     Kpp/Jp,     Kpy/Jp;
     Kyp/Jy,     Kyy/Jy];

C = [1, 0, 0, 0;
     0, 1, 0, 0];  % Output: [Pitch, Yaw]
D = zeros(2, 2);

sys_c = ss(A, B, C, D);

%% 3. Discrete-Time Model & Controller Design
Ts = 0.02; % 50 Hz sample time
sys_d = c2d(sys_c, Ts, 'zoh');
[Ad, Bd, Cd, Dd] = ssdata(sys_d);

% (1) LQR Feedback Gain (K)
Q = diag([1500, 1500, 10, 10]); % Penalize angle errors heavily
R = eye(2) * 0.1;               % Minimal penalty on voltage
[K, ~, ~] = dlqr(Ad, Bd, Q, R);

% (2) Feedforward Precompensator (N_bar) for Tracking
% Ensures steady-state error is zero for a step reference.
% Formula: N_bar = [ C * (I - (A - B*K))^-1 * B ]^-1
N_bar = inv(Cd * inv(eye(4) - (Ad - Bd * K)) * Bd);

% (3) Luenberger Observer Gain (L)
obs_poles = [0.65, 0.66, 0.67, 0.68]; % Faster than system dynamics
L = place(Ad', Cd', obs_poles).';

fprintf('\n=== Controller Matrices ===\n');
disp('LQR Gain (K) = '); disp(K);
disp('Feedforward Gain (N_bar) = '); disp(N_bar);

%% 4. Simulation Setup
T_final = 10;
t = 0:Ts:T_final;
N_sim = length(t);

x = zeros(4, N_sim);          % True state
xhat = zeros(4, N_sim);       % Estimated state
u = zeros(2, N_sim);          % Control inputs
y = zeros(2, N_sim);          % Measured outputs

% Reference: Pitch = 10 deg, Yaw = -10 deg
ref = deg2rad([10; -10]);     

%% 5. Simulation Loop
for k = 1:N_sim-1
    % 1. Measure output
    y(:, k) = Cd * x(:, k);
    
    % 2. Calculate Control Input: u = -K*xhat + N_bar*ref
    % (Using estimated states and the feedforward gain for tracking)
    u_k = -K * xhat(:, k) + N_bar * ref;
    u(:, k) = max(min(u_k, 15), -15); % Hardware voltage clamp at +/- 15V
    
    % 3. Plant Update (True dynamics)
    x(:, k+1) = Ad * x(:, k) + Bd * u(:, k);
    
    % 4. Observer Update
    % Predict next state based on current input and output error
    xhat(:, k+1) = Ad * xhat(:, k) + Bd * u(:, k) + L * (y(:, k) - Cd * xhat(:, k));
end

%% 6. Plotting Results
figure('Color', 'w', 'Position', [100, 100, 1000, 500]);

% Pitch & Yaw Response
subplot(1, 2, 1);
plot(t, rad2deg(x(1, :)), 'b', 'LineWidth', 1.5); hold on;
plot(t, rad2deg(x(2, :)), 'g', 'LineWidth', 1.5);
yline(10, 'b--', 'Pitch Ref');
yline(-10, 'g--', 'Yaw Ref');
title('Helicopter Angles (Tracking)');
xlabel('Time (s)'); ylabel('Angle (deg)');
legend('Pitch', 'Yaw', 'Location', 'Best'); grid on;

% Motor Voltages
subplot(1, 2, 2);
plot(t, u(1, :), 'r', 'LineWidth', 1.2); hold on;
plot(t, u(2, :), 'k', 'LineWidth', 1.2);
title('Motor Voltages (u)');
xlabel('Time (s)'); ylabel('Voltage (V)');
legend('V_m (Main)', 'V_t (Tail)', 'Location', 'Best'); grid on;