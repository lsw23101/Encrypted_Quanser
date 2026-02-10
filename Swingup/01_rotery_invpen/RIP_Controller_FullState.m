% =========================================================================
% 파일명: RIP_Controller_FullState.m
% 설명: Full State Feedback Controller (u = -Kx)
% =========================================================================

function [out1, out2] = RIP_Controller_FullState(mode, in1, in2)
    
    switch mode
        case 'init'
            %% [초기화 모드]
            % 입력: in1 = model 구조체
            % 출력: out1 = ctrl_data (K 게인 저장)
            
            model = in1;
            
            % LQR 게인 설계
            Q = diag([1, 100, 0.1, 0.1]); 
            R = 1;
            
            % Full State용 K 계산
            [K, ~, ~] = dlqr(model.A, model.B, Q, R);
            
            ctrl_data.K = K;
            
            out1 = ctrl_data;
            out2 = [];

        case 'run'
            %% [실행 모드]
            % 입력: in1 = y_meas (여기서는 Full State x와 동일)
            %       in2 = ctrl_data
            % 출력: out1 = u
            %       out2 = ctrl_data
            
            x_meas = in1; % 플랜트가 x를 통째로 줌
            K = in2.K;
            
            % 단순 상태 피드백
            u = -K * x_meas;
            
            out1 = u;
            out2 = in2; % 업데이트할 상태 없음
    end
end