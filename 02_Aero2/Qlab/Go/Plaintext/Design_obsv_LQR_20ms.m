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

Ts = 0.02;
ct_sys = ss(A,B,C,D);
ds_sys = c2d(ct_sys, Ts, 'zoh');
[Ad, Bd, Cd, Dd] = ssdata(ds_sys);

%% LQR 가중치 

Q = [5000 0 0 0; 0 100 0 0; 0 0 0 0; 0 0 0 0];
R = 1;

[K, S, cl_poles] = dlqr(Ad, Bd, Q, R);

obs_poles = [0.71 0.72 0.73 0.74];         
L = place(Ad', Cd', obs_poles).';      


F = Ad - Bd*K - L*Cd;
G = L;
H = -K;

disp('F ='); disp(F);
disp('G ='); disp(G);
disp('H ='); disp(H);


function print_numpy(name, M, prec)
    if nargin < 3, prec = 4; end
    fmt = ['%0.' num2str(prec) 'f'];

    if isvector(M)
        fprintf('%s = np.array([ ', name);
        for k = 1:numel(M)
            fprintf(fmt, M(k));
            if k < numel(M), fprintf(', '); end
        end
        fprintf(' ], dtype=np.float64)  # shape (%d,)\n\n', numel(M));
    else
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

print_numpy('F', F, 4);
print_numpy('G', G, 4);
print_numpy('H', H, 4);
abs(eig(F))