% =========================================================================
% 파일명: RIP_Platform.m
% 설명: Rotary Inverted Pendulum 통합 시뮬레이션 플랫폼
% 수정: 시각화 시 보간(Interpolation)을 제거하고 모든 프레임 표시
% =========================================================================
function RIP_Platform
    clear; clc; close all;
    %% 0. [핵심] 제어기 모드 선택
    % 1 = Full State Feedback 
    % 2 = Observer Feedback
    % 3 = Swing-Up & Balance
    CTRL_MODE = 3; 
    
    %% 1. 파라미터 및 플랜트 설정
    p.Rm = 8.4; p.kt = 0.042; p.km = 0.042;
    p.mr = 0.095; p.r = 0.085; p.Jr = p.mr * p.r^2 / 3;
    p.mp = 0.024; p.Lp = 0.129; p.l = p.Lp / 2;
    p.Jp = p.mp * p.Lp^2 / 3;
    p.br = 0; p.bp = 0; p.g = 9.81;
    
    %% 2. 모델 선형화 및 이산화
    x0 = [0; 0; 0; 0]; u0 = 0;
    [A_cont, B_cont] = get_linear_model(@(x,u) rip_dynamics_plant(0, x, u, p), x0, u0);
    
    Ts = 0.02; % 20ms 고정  변경 x
    sys_c = ss(A_cont, B_cont, eye(4), 0);
    sys_d = c2d(sys_c, Ts, 'zoh');
    
    model.A = sys_d.A;
    model.B = sys_d.B;
    model.C = eye(4); 
    model.Ts = Ts;
    
    %% 3. 제어기 함수 매핑 및 초기화
    switch CTRL_MODE
        case 1, ctrl_func = @RIP_Controller_FullState;
        case 2, ctrl_func = @RIP_Controller_Observer;
        case 3, ctrl_func = @RIP_Controller_SwingUp;
        otherwise, error('Mode Error');
    end
    [ctrl_data] = ctrl_func('init', model, []); 
    
    %% 4. 시뮬레이션 루프
    T_final = 10; 
    steps = floor(T_final / Ts);
    
    if CTRL_MODE == 3
        x_curr = [0; pi*99/100; 0; 0]; 
    else
        x_curr = [0; 10*pi/180; 0; 0]; 
    end
    
    time_log = []; state_log = []; input_log = []; est_log = [];
    
    fprintf('Simulating... ');
    for k = 1:steps
        y_meas = x_curr; 
        [u, ctrl_data] = ctrl_func('run', y_meas, ctrl_data);
        u_apply = max(min(u, 10), -10);
        
        if CTRL_MODE ~= 3
            if abs(x_curr(2)) > 60*pi/180, u_apply = 0; end 
        end
        
        t_start = (k-1)*Ts; t_end = k*Ts;
        [t_seg, x_seg] = ode45(@(t,x) rip_dynamics_plant(t, x, u_apply, p), [t_start, t_end], x_curr);
        
        time_log = [time_log; t_seg];
        state_log = [state_log; x_seg];
        input_log = [input_log; repmat(u_apply, length(t_seg), 1)];
        
        if isfield(ctrl_data, 'x_est')
            est_log = [est_log; repmat(ctrl_data.x_est', length(t_seg), 1)];
        end
        x_curr = x_seg(end, :)';
    end
    fprintf('Done.\n');
    
    %% 5. 시각화
    visualize_results(time_log, state_log, input_log, est_log, p, Ts);
end

% --- 물리 엔진 ---
function dxdt = rip_dynamics_plant(~, x, u, p)
    theta = x(1); alpha = x(2); theta_dot = x(3); alpha_dot = x(4);
    Vm = u; 
    tau = (p.kt * (Vm - p.km * theta_dot)) / p.Rm;
    M11 = p.Jr + p.mp * p.r^2 + p.mp * p.l^2 * sin(alpha)^2;
    M12 = p.mp * p.r * p.l * cos(alpha); M21 = M12; M22 = p.Jp;
    det_M = M11*M22 - M12*M21;
    C1 = p.mp * p.l^2 * alpha_dot * theta_dot * sin(2*alpha) + p.mp * p.r * p.l * alpha_dot^2 * sin(alpha);
    C2 = -0.5 * p.mp * p.l^2 * theta_dot^2 * sin(2*alpha) - p.mp * p.g * p.l * sin(alpha);
    D1 = -p.br * theta_dot; D2 = -p.bp * alpha_dot;
    RHS1 = tau + D1 - C1; RHS2 = 0 + D2 - C2;
    q_ddot1 = (M22 * RHS1 - M12 * RHS2) / det_M; q_ddot2 = (-M21 * RHS1 + M11 * RHS2) / det_M;
    dxdt = [theta_dot; alpha_dot; q_ddot1; q_ddot2];
end

function [A, B] = get_linear_model(dyn_func, x0, u0)
    n=length(x0); m=length(u0); eps=1e-5; f0=dyn_func(x0,u0);
    A=zeros(n); B=zeros(n,m);
    for i=1:n, xp=x0; xp(i)=xp(i)+eps; A(:,i)=(dyn_func(xp,u0)-f0)/eps; end
    for i=1:m, up=u0+eps; B(:,i)=(dyn_func(x0,up)-f0)/eps; end
end

% --- [수정됨] 시각화 함수 ---
function visualize_results(t, x, u, x_est, p, Ts)
    % 중복 시간 제거 및 정렬
    [t, idx] = unique(t); 
    x = x(idx, :); 
    u_plot_t = (0:length(u)-1)*(t(end)/length(u));
    
    has_est = ~isempty(x_est);
    if has_est, x_est = x_est(idx, :); end
    
    figure('Color','w','Position',[100,100,800,700]);
    
    % 1. Pendulum Angle
    subplot(3,1,1); hold on;
    plot(t, x(:,2)*180/pi, 'r', 'LineWidth', 2); 
    if has_est, plot(t, x_est(:,2)*180/pi, 'g--', 'LineWidth',1.5); end
    ylabel('Pen (deg)'); grid on; title('Pendulum Angle');
    if has_est, legend('Real','Estimated'); end
    yline(0, 'k--');
    
    % 2. Arm Angle (Limit 표시)
    subplot(3,1,2); hold on;
    plot(t, x(:,1)*180/pi, 'b', 'LineWidth', 1.5);
    if has_est, plot(t, x_est(:,1)*180/pi, 'c--', 'LineWidth',1.5); end
    yline(100, 'r--', 'Limit (+100)');
    yline(-100, 'r--', 'Limit (-100)');
    ylabel('Arm (deg)'); grid on; title('Arm Angle');
    
    % 3. Control Input
    subplot(3,1,3); stairs(u_plot_t, u, 'k'); ylabel('V'); title('Control Input'); grid on;
    
    % --- Animation 설정 ---
    f_anim=figure('Name','Replay (Step by Ts)','Color','w'); axis equal; grid on; view(45,30);
    xlim([-0.25 0.25]); ylim([-0.25 0.25]); zlim([-0.25 0.25]); hold on;
    
    % 가동 범위(Virtual Wall) 시각화
    lim_angle = 100 * pi/180; r_vis = p.r * 1.3;
    plot3([0, r_vis*cos(lim_angle)], [0, r_vis*sin(lim_angle)], [0, 0], 'r--', 'LineWidth', 1.5);
    plot3([0, r_vis*cos(-lim_angle)], [0, r_vis*sin(-lim_angle)], [0, 0], 'r--', 'LineWidth', 1.5);
    text(r_vis*cos(lim_angle), r_vis*sin(lim_angle), 0, ' Limit (+100)', 'Color', 'r', 'FontSize', 8);
    
    h_arm=plot3([0 0],[0 0],[0 0],'b-','LineWidth',4);
    h_pen=plot3([0 0],[0 0],[0 0],'r-','LineWidth',3);
    h_bob=plot3(0,0,0,'mo','MarkerSize',6,'MarkerFaceColor','m');
    
    % [핵심 수정] SliderStep 계산
    % 전체 시간 범위
    total_time = t(end);
    
    % 슬라이더 한 칸(화살표 키)을 Ts(0.005s) 비율로 설정
    minor_step = Ts / total_time; 
    
    % 슬라이더 몸통 클릭(PageUp/Down)은 10배(0.05s) 정도로 설정
    major_step = minor_step * 10; 
    
    % SliderStep 속성 추가
    uicontrol('Parent',f_anim,'Style','slider','Position',[80 20 400 20],...
        'Min',0,'Max',total_time,...
        'SliderStep', [minor_step, major_step], ... 
        'Callback',@(s,~) update(s.Value, x, t, p, h_arm, h_pen, h_bob));
        
    % 초기 업데이트
    update(0, x, t, p, h_arm, h_pen, h_bob);
end

% --- [수정됨] 업데이트 콜백 ---
function update(val, x_d, t_d, p, ha, hp, hb)
    % [수정] 현재 슬라이더 시간(val)과 가장 가까운 인덱스를 찾음
    [~, id] = min(abs(t_d - val));
    
    th=x_d(id,1); al=x_d(id,2);
    xa=p.r*cos(th); ya=p.r*sin(th); 
    xp=xa+p.Lp*sin(al)*sin(th); yp=ya-p.Lp*sin(al)*cos(th); zp=p.Lp*cos(al);
    
    set(ha,'XData',[0 xa],'YData',[0 ya]); 
    set(hp,'XData',[xa xp],'YData',[ya yp],'ZData',[0 zp]); 
    set(hb,'XData',xp,'YData',yp,'ZData',zp);
    title(sprintf('Time: %.3f s (Frame: %d)', t_d(id), id));
end