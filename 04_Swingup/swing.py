## example swingup_balance_qube.py
# Strategy: Selective Energy Swing-Up + LQR Balancing
# Feature: Easy Sign Flipping (Calibration Knobs) at the top
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

import sys
sys.path.append(r"C:\Quanser\0_libraries\python")

from threading import Thread
import signal
import time
import math
import numpy as np
from pal.products.qube import QubeServo2, QubeServo3
from pal.utilities.math import SignalGenerator, ddt_filter
from pal.utilities.scope import Scope

# =============================================================================
# [★중요] 부호 보정 스위치 (이 값들을 하나씩 바꿔보며 맞추세요)
# =============================================================================

# 1. 센서 방향 보정 (왼쪽으로 밀 때 각도가 커져야 함)
# Qube 하드웨어마다 장착 방향이 다를 수 있음. (-1.0 or 1.0)
SIGN_THETA = -1.0 

# 2. 모터 방향 보정 (+전압을 주면 +각도(왼쪽)로 가야 함)
# LQR이 켜지자마자 튕겨 나가면 이걸 뒤집으세요. (-1.0 or 1.0)
SIGN_VOLTAGE = -1.0 

# 3. 스윙업 펌핑 방향 (에너지가 줄어들면 이걸 뒤집으세요)
# (-1.0 or 1.0)
SWING_KICK_DIR = 1.0  

# =============================================================================

# Setup to enable killing the data generation thread using keyboard interrupts
global KILL_THREAD
KILL_THREAD = False
def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)

# region: Setup Scopes
simulationTime = 60
scopePendulum = Scope(title='Pendulum Angle (alpha)', timeWindow=10, xLabel='Time (s)', yLabel='Rad')
scopePendulum.attachSignal(name='Alpha (rad)', width=1)

scopeBase = Scope(title='Arm Angle (theta)', timeWindow=10, xLabel='Time (s)', yLabel='Rad')
scopeBase.attachSignal(name='Theta (rad)', width=1)

scopeVoltage = Scope(title='Motor Voltage', timeWindow=10, xLabel='Time (s)', yLabel='Volts')
scopeVoltage.attachSignal(name='Voltage', width=1)

scopeMode = Scope(title='Control Mode', timeWindow=10, xLabel='Time (s)', yLabel='1=Swing, 0=Balance')
scopeMode.attachSignal(name='Mode', width=1)
# endregion

# Physical Parameters
mp = 0.024       
Lp = 0.129       
l = Lp / 2       
g = 9.81         
Jp = mp * (Lp**2) / 3  

# Control Parameters
E_ref = 0.0      
mu = 150.0        # Swing-up Gain # suitable for 50 Hz
# mu = 15.0        # Swing-up Gain (Energy Injection)
switch_deg = 20.0 # LQR Switching Threshold

def control_loop():
    # -------------------------------------------------------------------------
    # CONFIGURATION
    # -------------------------------------------------------------------------
    qubeVersion = 3   
    hardware = 1      
    pendulum = 1      
    frequency = 40    # 20ms
    # frequency = 20    # 20ms
    
    # LQR Gain (K)
    if qubeVersion == 2:
        QubeClass = QubeServo2
        K = np.array([-2.0, 35.0, -1.5, 3.0]) 
    else:
        QubeClass = QubeServo3
        # Qube 3 Gains (From your MATLAB or generic tuning)
        K = np.array([-2.0, 28.0, -1.5, 2.5]) 

    state_theta_dot = np.array([0,0], dtype=np.float64)
    state_alpha_dot = np.array([0,0], dtype=np.float64)

    VMAX = 7.0 
    countMax = frequency / 50
    count = 0

    with QubeClass(hardware=hardware, pendulum=pendulum, frequency=frequency) as myQube:
        
        startTime = time.time()
        timeStamp = 0
        
        def elapsed_time():
            return time.time() - startTime

        print("Controller Started. Please ensure the pendulum is hanging down.")
        print(f"Configs -> Theta Sign: {SIGN_THETA}, Volt Sign: {SIGN_VOLTAGE}, Swing Dir: {SWING_KICK_DIR}")

        while timeStamp < simulationTime and not KILL_THREAD:
            
            myQube.read_outputs()
            
            # -----------------------------------------------------------------
            # 1. SENSOR READING & NORMALIZATION
            # -----------------------------------------------------------------
            # [보정 1] Theta 부호 적용
            theta = myQube.motorPosition * SIGN_THETA
            
            # Alpha: Raw 0 is usually bottom. Convert to 0 at top.
            alpha_raw = myQube.pendulumPosition
            alpha = (alpha_raw - np.pi + np.pi) % (2*np.pi) - np.pi
            
            # Velocity Filter
            theta_dot, state_theta_dot = ddt_filter(theta, state_theta_dot, 50, 1/frequency)
            alpha_dot, state_alpha_dot = ddt_filter(alpha, state_alpha_dot, 100, 1/frequency)

            x_meas = np.array([theta, alpha, theta_dot, alpha_dot], dtype=np.float64)
            
            # -----------------------------------------------------------------
            # 2. CONTROLLER LOGIC
            # -----------------------------------------------------------------
            u = 0.0
            control_mode = 0 
            
            alpha_deg = abs(math.degrees(alpha))
            
            # --- MODE A: LQR Balancing (< 20 deg) ---
            if alpha_deg < switch_deg:
                control_mode = 1
                # Standard Feedback: u = -Kx
                # 만약 LQR이 반대로 작동하면 SIGN_VOLTAGE를 뒤집으세요.
                u = -1.0 * float(np.dot(K, x_meas))
            
            # --- MODE B: Selective Swing-Up (> 20 deg) ---
            else:
                control_mode = 0
                
                # Energy Calculation
                E_pot = mp * g * l * (math.cos(alpha) - 1)
                E_kin = 0.5 * Jp * (alpha_dot**2)
                E_total = E_pot + E_kin
                E_err = E_ref - E_total
                
                # Åström Term
                term = np.sign(alpha_dot * math.cos(alpha))
                if abs(alpha_dot) < 0.05 and abs(math.cos(alpha)) < 0.1:
                    term = 0
                
                # [보정 3] Swing Direction 적용
                u_raw = SWING_KICK_DIR * mu * E_err * term
                
                # Directional Pumping (Input Gating) Logic
                # "돌아오는 방향의 힘만 허용한다"
                
                if theta > 0:
                    # Arm이 오른쪽(+)에 있음
                    if u_raw > 0:
                        u = 0.0  # 더 오른쪽(+)으로 가려 하면 차단
                    else:
                        u = u_raw # 왼쪽(-)으로 오려 하면 통과
                        
                elif theta < 0:
                    # Arm이 왼쪽(-)에 있음
                    if u_raw < 0:
                        u = 0.0  # 더 왼쪽(-)으로 가려 하면 차단
                    else:
                        u = u_raw # 오른쪽(+)으로 오려 하면 통과
                else:
                    u = u_raw
            
            # -----------------------------------------------------------------
            # 3. ACTUATION & SAFETY
            # -----------------------------------------------------------------
            
            # Hard Saturation
            voltage = np.clip(u, -VMAX, VMAX)
            
            # Safety Cutoff (Cable Protection)
            # Arm이 100도를 넘어가면 강제 종료
            if abs(theta) > (100 * math.pi / 180):
                voltage = 0.0

            # [보정 2] Voltage 부호 적용
            # 최종적으로 모터에 쓰기 전에 부호를 뒤집을 수 있음
            myQube.write_voltage(voltage * SIGN_VOLTAGE)
            # myQube.write_voltage(0)
            # -----------------------------------------------------------------
            # LOGGING
            # -----------------------------------------------------------------
            count += 1
            if count >= countMax:
                scopeVoltage.sample(timeStamp, [voltage * SIGN_VOLTAGE])
                scopePendulum.sample(timeStamp, [alpha])
                scopeBase.sample(timeStamp, [theta])
                scopeMode.sample(timeStamp, [control_mode])
                count = 0
            
            timeStamp = elapsed_time()

thread = Thread(target=control_loop)
thread.start()

print("Press Enter to Stop...")
try:
    while thread.is_alive() and (not KILL_THREAD):
        Scope.refreshAll()
        time.sleep(0.01)
except KeyboardInterrupt:
    KILL_THREAD = True

input() 
KILL_THREAD = True