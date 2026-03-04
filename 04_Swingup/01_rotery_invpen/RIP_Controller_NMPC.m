function [out1, out2] = RIP_Controller_NMPC(mode, in1, in2)
    
    switch mode
        case 'init'
            %% [초기화 모드] NMPC 객체 생성
            model = in1;
            Ts = model.Ts; % 0.02s
            
            % 1. NMPC 객체 생성 (State:4개, Output:4개(Full state), Input:1개)
            nx = 4; ny = 4; nu = 1;
            nlobj = nlmpc(nx, ny, nu);
            
            % 2. 샘플링 타임 및 예측 구간 설정
            nlobj.Ts = Ts;
            nlobj.PredictionHorizon = 25; % 미래 25스텝(0.5초)을 내다봄
            nlobj.ControlHorizon = 4;     % 앞쪽 4스텝만 최적화 (계산량 절약)
            
            % 3. 물리 모델 연결
            % 위에서 만든 함수 파일 이름('rip_nmpc_model')을 지정
            nlobj.Model.StateFcn = 'rip_nmpc_model';
            
            % Output이 State 그 자체임을 명시 (Full State Feedback)
            nlobj.Model.OutputFcn = @(x,u) x; 
            
            % 4. 제약 조건 (Constraints)
            nlobj.MV.Min = -10; % 전압 최소 -10V
            nlobj.MV.Max = 10;  % 전압 최대 10V
            
            % 5. 가중치 설정 (Weights) - 튜닝 포인트!
            % OutputVariables: [theta, alpha, theta_dot, alpha_dot]
            % 진자 각도(2번)를 0으로 만드는 게 최우선 목표
            
            % [Arm각도, Pen각도, Arm속도, Pen속도]
            nlobj.Weights.OutputVariables = [0.1, 50, 0.1, 0.1]; 
            
            % 입력(전압) 변화율에 대한 페널티 (너무 급격하게 바꾸지 마라)
            nlobj.Weights.ManipulatedVariablesRate = 0.1; 
            
            % 6. Solver 옵션 (속도 향상용)
            nlobj.Optimization.SolverOptions.MaxIterations = 10; % 반복 제한
            
            % 7. 초기화 및 유효성 검사
            x0 = [0; pi; 0; 0];
            u0 = 0;
            validateFcns(nlobj, x0, u0); % 설정이 올바른지 체크
            
            % 데이터 저장
            ctrl_data.nlobj = nlobj;
            ctrl_data.u_last = 0; % 이전 입력값 저장용
            
            out1 = ctrl_data;
            out2 = [];

        case 'run'
            %% [실행 모드] nlmpcmove로 최적 입력 계산
            x_meas = in1; 
            D = in2;
            
            % NMPC Solver 호출
            % x_meas: 현재 상태
            % D.u_last: 직전 입력 (입력 변화율 계산용)
            
            % 타겟(Reference) 설정: 모든 상태가 0이 되길 원함 (Upright)
            % [theta=0, alpha=0, theta_dot=0, alpha_dot=0]
            ref = [0, 0, 0, 0]; 
            
            % *중요*: 각도 wrapping 문제 해결
            % 진자가 한 바퀴 돌아서 2pi가 되면, 0으로 가려하지 말고 2pi에 머물러야 함.
            % (NMPC는 pi와 -pi를 다르게 인식함)
            % 하지만 스윙업은 pi -> 0 으로 가야 하므로, 
            % 여기서는 x_meas 그대로 넣고 Reference를 0으로 두는 게 맞음.
            % (단, alpha가 2pi를 넘어가면 ref도 2pi로 바꿔주는 로직이 필요할 수 있으나
            %  일단 기본적으로 0을 목표로 둠)
            
            [u_opt, ~, ~] = nlmpcmove(D.nlobj, x_meas, D.u_last, ref);
            
            % 다음 스텝을 위해 저장
            D.u_last = u_opt;
            
            out1 = u_opt;
            out2 = D;
    end
end