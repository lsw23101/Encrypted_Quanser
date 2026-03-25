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


D = zeros(2,1);

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