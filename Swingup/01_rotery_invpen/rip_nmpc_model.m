function dxdt = rip_nmpc_model(x, u)
    % NMPC Solver가 내부적으로 사용하는 Continuous Dynamics
    % x = [theta; alpha; theta_dot; alpha_dot]
    % u = Voltage
    
    % --- 물리 파라미터 (Platform과 동일해야 함) ---
    p.Rm = 8.4; p.kt = 0.042; p.km = 0.042;
    p.mr = 0.095; p.r = 0.085; p.Jr = p.mr * p.r^2 / 3;
    p.mp = 0.024; p.Lp = 0.129; p.l = p.Lp / 2;
    p.Jp = p.mp * p.Lp^2 / 3;
    p.br = 4.7078e-05; p.bp = 4e-4; p.g = 9.81;

    theta = x(1);
    alpha = x(2);
    theta_dot = x(3);
    alpha_dot = x(4);
    
    Vm = u; 
    
    % 운동 방정식 (기존 rip_dynamics_plant와 동일 로직)
    tau = (p.kt * (Vm - p.km * theta_dot)) / p.Rm;

    M11 = p.Jr + p.mp * p.r^2 + p.mp * p.l^2 * sin(alpha)^2;
    M12 = p.mp * p.r * p.l * cos(alpha);
    M21 = M12;
    M22 = p.Jp;
    
    det_M = M11*M22 - M12*M21;
    
    C1 = p.mp * p.l^2 * alpha_dot * theta_dot * sin(2*alpha) + ...
         p.mp * p.r * p.l * alpha_dot^2 * sin(alpha);
    C2 = -0.5 * p.mp * p.l^2 * theta_dot^2 * sin(2*alpha) ...
         - p.mp * p.g * p.l * sin(alpha);
     
    D1 = -p.br * theta_dot;
    D2 = -p.bp * alpha_dot;
    
    RHS1 = tau + D1 - C1;
    RHS2 = 0   + D2 - C2;
    
    q_ddot1 = (M22 * RHS1 - M12 * RHS2) / det_M;
    q_ddot2 = (-M21 * RHS1 + M11 * RHS2) / det_M;
    
    dxdt = [theta_dot; alpha_dot; q_ddot1; q_ddot2];
end