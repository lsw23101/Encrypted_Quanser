% =========================================================================
% 파일명: RIP_Controller_Observer.m
% 설명: 플랫폼에서 Full State를 받지만, 내부적으로 Output(y)만 사용하여 제어
% =========================================================================

function [out1, out2] = RIP_Controller_Observer(mode, in1, in2)
    
    switch mode
        case 'init'
            %% [초기화 모드]
            model = in1;
            
            % 1. 실제 물리적으로 측정 가능한 C 행렬 정의
            % (플랫폼이 주는 model.C는 eye(4)일 수 있으므로, 여기서 실제 센서 모델을 재정의)
            C_sensor = [1 0 0 0;  % Arm 각도 측정 가능
                        0 1 0 0]; % Pendulum 각도 측정 가능
            
            % 2. LQR 게인 (K) 설계
            Q = diag([1, 100, 0.1, 0.1]); 
            R = 1;
            [K, ~, ~] = dlqr(model.A, model.B, Q, R);
            
            % 3. Observer 게인 (L) 설계
            % Process/Measurement Noise 공분산 행렬 (튜닝 파라미터)
            Q_obs = diag([0.1, 0.1, 10, 10]); % 상태 잡음 (속도 쪽에 불확실성 둠)
            R_obs = diag([0.01, 0.01]);       % 센서 잡음 (엔코더는 꽤 정확함)
            
            % Kalman Filter 방식 게인 계산 (dlqe)
            [L, ~, ~] = dlqe(model.A, eye(4), C_sensor, Q_obs, R_obs);
            
            % 4. 데이터 저장
            ctrl_data.K = K;
            ctrl_data.L = L;
            ctrl_data.A = model.A;
            ctrl_data.B = model.B;
            ctrl_data.C_sensor = C_sensor; % 센서 모델 저장
            
            % 초기 추정값 (모르는 상태에서 시작)
            ctrl_data.x_est = [0; 0; 0; 0]; 
            
            out1 = ctrl_data;
            out2 = [];

        case 'run'
            %% [실행 모드]
            % in1: Full State (x) - 플랫폼이 보내준 정답지
            % in2: ctrl_data
            
            x_full_truth = in1; % 플랫폼이 준 실제 상태 (정답)
            D = in2; 
            
            % --- [핵심] 센서 측정 시뮬레이션 ---
            % 정답(Full State)을 다 쓰지 않고, 측정 가능한 y만 뽑아냄
            % y = C_sensor * x
            y_meas = D.C_sensor * x_full_truth; 
            
            % (원한다면 여기에 센서 노이즈를 추가해서 더 리얼하게 만들 수도 있음)
            % y_meas = y_meas + 0.001 * randn(2,1); 
            
            
            % --- [옵저버 알고리즘 시작] ---
            
            % 1. Prediction (Time Update): x[k|k-1]
            if ~isfield(D, 'u_prev'), D.u_prev = 0; end
            x_pred = D.A * D.x_est + D.B * D.u_prev;
            
            % 2. Correction (Measurement Update): x[k|k]
            % 예측된 y와 실제(가정된) 센서값 y_meas의 차이로 보정
            y_pred = D.C_sensor * x_pred;
            x_est_new = x_pred + D.L * (y_meas - y_pred);
            
            % 3. 제어 법칙 (LQR)
            % *중요*: 진짜 상태(x_full_truth)가 아니라, 추정된 상태(x_est_new)를 믿고 제어함
            u = -D.K * x_est_new;
            
            % 4. 데이터 업데이트
            D.x_est = x_est_new;
            D.u_prev = u;
            
            out1 = u;
            out2 = D;
    end
end