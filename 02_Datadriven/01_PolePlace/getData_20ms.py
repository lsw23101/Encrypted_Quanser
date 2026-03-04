## example balance_control_qube_with_data_logging.py

import sys
sys.path.append(r"C:\Quanser\0_libraries\python")

from threading import Thread
import signal
import time
import math
import numpy as np
import pandas as pd  # CSV 저장을 위해 추가
from pal.products.qube import QubeServo2, QubeServo3
from pal.utilities.math import SignalGenerator, ddt_filter
from pal.utilities.scope import Scope

# Setup to enable killing the data generation thread using keyboard interrupts
global KILL_THREAD
KILL_THREAD = False
def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)

# 데이터 저장을 위한 전역 리스트
data_log = []

# region: Setup
simulationTime = 30 # 30초 실행
color = np.array([0, 1, 0], dtype=np.float64)

scopePendulum = Scope(
    title='Pendulum encoder - alpha (rad)',
    timeWindow=10,
    xLabel='Time (s)',
    yLabel='Position (rad)')
scopePendulum.attachSignal(name='Pendulum - alpha (rad)',  width=1)

scopeBase = Scope(
    title='Base encoder - theta (rad)',
    timeWindow=10,
    xLabel='Time (s)',
    yLabel='Position (rad)')
scopeBase.attachSignal(name='Base - theta (rad)',  width=1)

scopeVoltage = Scope(
    title='Motor Voltage',
    timeWindow=10,
    xLabel='Time (s)',
    yLabel='Voltage (volts)')
scopeVoltage.attachSignal(name='Voltage',  width=1)

scopeError = Scope(
    title='Control Error (u - voltage)',
    timeWindow=10,
    xLabel='Time (s)',
    yLabel='Error (V)')
scopeError.attachSignal(name='u - voltage', width=1)

scopePendulum.attachSignal(
    name='alpha_hat (rad)',  
    width=1, 
    color=np.array([0, 0, 255])   # 파랑
)
scopeBase.attachSignal(
    name='theta_hat (rad)',      
    width=1, 
    color=np.array([0, 0, 255])   # 파랑
)
#endregion

# pre-designed controller 
F = np.array([
    [ -0.0883, -0.3443, 0.0363, -0.0258 ],
    [ 0.0505, -0.5051, 0.0161, -0.0054 ],
    [ -10.2985, -32.5648, 2.6346, -2.5800 ],
    [ 5.8855, -49.9106, 1.6278, -1.5465 ]
], dtype=np.float64)

G = np.array([
    [ 1.1505, 0.0689 ],
    [ 0.0112, 1.2548 ],
    [ 16.5520, 4.9033 ],
    [ 0.3420, 24.6625 ]
], dtype=np.float64)

# H = np.array([ -6.2268, 30.5682, -1.6279, 2.5990 ], dtype=np.float64)  # shape (4,)

H = np.array([ -2.0032, 18.2998, -0.6714, 1.4341 ], dtype=np.float64)  # shape (4,)

def control_loop():
    global KILL_THREAD, data_log

    qubeVersion = 3
    hardware = 1
    pendulum = 1
    frequency = 50  # 20ms
    
    state_theta_dot = np.array([0,0], dtype=np.float64)
    state_alpha_dot = np.array([0,0], dtype=np.float64)

    countMax = frequency / 50
    count = 0

    if qubeVersion == 2:
        QubeClass = QubeServo2
    else:
        QubeClass = QubeServo3

    xhat = np.zeros(4, dtype=np.float64)

    with QubeClass(hardware=hardware, pendulum=pendulum, frequency=frequency) as myQube:

        startTime = time.time()
        timeStamp = 0

        while timeStamp < simulationTime and not KILL_THREAD:

            # Read sensor information
            myQube.read_outputs()

            theta = myQube.motorPosition * -1
            alpha_f =  myQube.pendulumPosition
            
            # 업라이트(위쪽)가 0도가 되도록 시프트
            alpha = (alpha_f - np.pi + np.pi) % (2*np.pi) - np.pi
            alpha_degrees = abs(math.degrees(alpha))

            y = np.array([theta, alpha], dtype=np.float64)

            # 속도 계산 (미분 필터)
            theta_dot, state_theta_dot = ddt_filter(theta, state_theta_dot, 50, 1/frequency)
            alpha_dot, state_alpha_dot = ddt_filter(alpha, state_alpha_dot, 100, 1/frequency)
            x_meas = np.array([theta, alpha, theta_dot, alpha_dot], dtype=np.float64)


            # 기존 제어 입력 계산
            # u_base = -1 * float(H @ xhat)
            u_base = float(H @ xhat)

            
            u_base = float(H @ x_meas)

            # --- [추가] 데이터 기반 제어를 위한 노이즈 추가 영역 ---
            # 밸런싱이 유지되는 범위 내에서 노이즈(평균 0, 표준편차 0.1~0.2 정도 추천) 추가
            noise = np.random.normal(0, 0.5) 
            u_noisy = u_base + noise
            # --------------------------------------------------

            VMAX = 10.0

            if alpha_degrees > 20:
                u_final = 0
            else:
                u_final = np.clip(u_noisy, -VMAX, VMAX)

            # 실제 장비에 노이즈가 섞인 전압 인가
            myQube.write_voltage(u_final)

            # 옵저버 업데이트
            if alpha_degrees > 20:
                xhat = np.array([0, 0, 0, 0])
            else:
                xhat = F @ xhat + G @ y

            # --- [추가] 데이터 기록 ---
            # 타임스탬프, 노이즈 섞인 입력, 측정된 theta, 측정된 alpha 저장
            data_log.append([timeStamp, u_final, theta, alpha])
            # -------------------------

            # Plot to scopes
            count += 1
            if count >= countMax:
                scopeVoltage.sample(timeStamp, [u_final])
                scopeError.sample(timeStamp, [u_base]) 
                scopePendulum.sample(timeStamp, [alpha, xhat[1]])
                scopeBase.sample(timeStamp, [theta, xhat[0]])
                count = 0

            timeStamp = time.time() - startTime

# 실행 및 데이터 저장 로직
thread = Thread(target=control_loop)
thread.start()

try:
    while thread.is_alive() and (not KILL_THREAD):
        Scope.refreshAll()
        time.sleep(0.01)
except KeyboardInterrupt:
    KILL_THREAD = True

thread.join()

# --- [추가] CSV 파일 저장 영역 ---
if data_log:
    print("\n데이터를 저장 중입니다...")
    df = pd.DataFrame(data_log, columns=['Time', 'u_noisy', 'theta', 'alpha'])
    # filename = f"qube_data_{time.strftime('%Y%m%d_%H%M%S')}.csv"
    filename = f"qube_data.csv"
    df.to_csv(filename, index=False)
    print(f"저장 완료: {filename}")
else:
    print("저장할 데이터가 없습니다.")
# ------------------------------

input('Press the enter key to exit.')