% =========================================================================
% 파일명: RIP_Controller_SwingUp.m
% 설명: [사용자 아이디어] Directional Pumping (Selective Input)
% 원리: Arm이 치우친 방향으로 더 가속하려는 제어 입력은 차단(0)하고,
%       중앙으로 돌아오려는 입력만 허용하여 Drift를 방지함.
% =========================================================================
function [out1, out2] = RIP_Controller_SwingUp(mode, in1, in2)
    switch mode
       case 'init'
            %% [1. 초기화 모드]
            model = in1;
            
            p.mp = 0.024; p.g = 9.81;
            p.Lp = 0.129; p.l = p.Lp / 2;
            p.Jp = p.mp * p.Lp^2 / 3;
            ctrl_data.p = p;
            
            % LQR (Balancing)
            Q = diag([10, 100, 0, 0]); 
            R = 1; 
            [K, ~, ~] = dlqr(model.A, model.B, Q, R);
            ctrl_data.K = K;
            
            % Swing-up 파라미터
            ctrl_data.E_ref = 0;   
            
            % [중요] 한쪽 방향으로만 밀어주므로, 에너지가 덜 들어갈 수 있습니다.
            % 따라서 게인(mu)을 기존 60에서 90~100 정도로 좀 더 키우는 것을 추천합니다.
            ctrl_data.mu = 90;     
            ctrl_data.direction = -1; 
            
            ctrl_data.switch_deg = 20; 
            ctrl_data.sat_limit = 5;   
            
            out1 = ctrl_data;
            out2 = [];
            
        case 'run'
            %% [2. 실행 모드]
            x_meas = in1;
            ctrl_data = in2;
            p = ctrl_data.p;
            
            theta     = x_meas(1); % Arm
            alpha     = x_meas(2); % Pendulum
            theta_dot = x_meas(3); 
            alpha_dot = x_meas(4); 
            
            alpha_norm = mod(alpha + pi, 2*pi) - pi;
            threshold_rad = ctrl_data.switch_deg * (pi/180);
            
            if abs(alpha_norm) < threshold_rad
                %% >> [Mode A] LQR Balancing (20도 이내)
                u = -ctrl_data.K * [theta; alpha_norm; theta_dot; alpha_dot];
                
            else
                %% >> [Mode B] Selective Energy Swing-Up (사용자 아이디어 적용)
                
                % 1. 기본 Åström 에너지 제어 입력 계산
                E_pot = p.mp * p.g * p.l * (cos(alpha_norm) - 1);
                E_kin = 0.5 * p.Jp * alpha_dot^2;
                E_err = ctrl_data.E_ref - (E_pot + E_kin);
                
                term = sign(alpha_dot * cos(alpha_norm));
                if abs(alpha_dot) < 0.05 && abs(cos(alpha_norm)) < 0.1, term = 0; end
                
                % u_raw: 필터링 전의 원본 제어 입력
                u_raw = ctrl_data.direction * ctrl_data.mu * E_err * term;
                
                % ---------------------------------------------------------
                % [핵심 로직] Arm 위치에 따른 입력 선별 (Selective Pumping)
                % ---------------------------------------------------------
                
                if theta > 0 
                    % [상황 1] Arm이 오른쪽에 가 있음
                    if u_raw > 0
                        % 제어기가 오른쪽(+)으로 더 밀려고 함 -> "안 돼!" (차단)
                        u = 0;
                    else
                        % 제어기가 왼쪽(-)으로 당기려고 함 -> "그래!" (허용)
                        u = u_raw;
                    end
                    
                elseif theta < 0
                    % [상황 2] Arm이 왼쪽에 가 있음
                    if u_raw < 0
                        % 제어기가 왼쪽(-)으로 더 밀려고 함 -> "안 돼!" (차단)
                        u = 0;
                    else
                        % 제어기가 오른쪽(+)으로 당기려고 함 -> "그래!" (허용)
                        u = u_raw;
                    end
                    
                else
                    % [상황 3] 정확히 0도 (거의 없지만) -> 그냥 허용
                    u = u_raw;
                end
                
                % *참고: 이 방식은 반대쪽 힘을 아예 안 쓰므로 에너지가 천천히 찹니다.
                % 그래서 위쪽 init에서 mu 값을 좀 키워두었습니다.
                % ---------------------------------------------------------
            end
            
            % 전압 제한
            limit = ctrl_data.sat_limit; 
            u = max(min(u, limit), -limit);
            
            out1 = u;
            out2 = ctrl_data; 
    end
end