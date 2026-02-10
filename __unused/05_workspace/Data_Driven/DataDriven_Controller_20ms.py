## data_driven_balance_control.py

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

# Setup to enable killing the data generation thread using keyboard interrupts
global KILL_THREAD
KILL_THREAD = False
def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)

# region: Setup - 스코프 설정
simulationTime = 30 

scopePendulum = Scope(
    title='Pendulum Position - alpha (rad)',
    timeWindow=10,
    xLabel='Time (s)',
    yLabel='Position (rad)')
scopePendulum.attachSignal(name='Measured alpha', width=1)

scopeBase = Scope(
    title='Base Position - theta (rad)',
    timeWindow=10,
    xLabel='Time (s)',
    yLabel='Position (rad)')
scopeBase.attachSignal(name='Measured theta', width=1)

scopeVoltage = Scope(
    title='Motor Voltage (V)',
    timeWindow=10,
    xLabel='Time (s)',
    yLabel='Voltage (V)')
scopeVoltage.attachSignal(name='u_control', width=1)
#endregion

# ==========================================================
# [중요] 매트랩에서 설계한 12차 제어기 행렬 (Ac, Bc, Cc -> F, G, H)
# 반드시 매트랩의 Full Precision 값을 여기에 복사하세요.
# ==========================================================
F = np.array([
    [-0.0690, -0.0933, 0.0607, 0.3093, 0.7074, 1.4430, -0.5647, -0.4729, -0.2590, -0.2831, 0.4716, 0.1562],
    [1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, 0, 0]
], dtype=np.float64)

G = np.array([
    [0, 0], [0, 0], [0, 0], [0, 0], 
    [1.0, 0], [0, 1.0], 
    [0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]
], dtype=np.float64)

H = np.array([
    -0.0690, -0.0933, 0.0607, 0.3093, 0.7074, 1.4430, -0.5647, -0.4729, -0.2590, -0.2831, 0.4716, 0.1562
], dtype=np.float64)
# ==========================================================

def control_loop():
    global KILL_THREAD
    
    qubeVersion = 3
    hardware = 1
    pendulum = 1
    frequency = 50  # 20ms
    
    # 12차 제어기 상태(xc) 초기화
    xc = np.zeros(12, dtype=np.float64)

    QubeClass = QubeServo3 if qubeVersion == 3 else QubeServo2

    with QubeClass(hardware=hardware, pendulum=pendulum, frequency=frequency) as myQube:

        startTime = time.time()
        
        while (time.time() - startTime) < simulationTime and not KILL_THREAD:
            timeStamp = time.time() - startTime

            # 1. 센서 데이터 읽기
            myQube.read_outputs()
            theta = myQube.motorPosition * -1
            alpha_f = myQube.pendulumPosition
            
            # 업라이트 기준 0도 변환
            alpha = (alpha_f - np.pi + np.pi) % (2*np.pi) - np.pi
            alpha_degrees = abs(math.degrees(alpha))

            # 출력 벡터 y = [theta, alpha]
            y = np.array([theta, alpha], dtype=np.float64)

            # 2. 동적 제어 연산 (Dynamic Output Feedback)
            # u(k) = H * xc(k)
            u_raw = float(np.dot(H, xc))
            
            VMAX = 10.0

            # 3. 제어 범위 설정 (10도 이내에서만 작동)
            if alpha_degrees > 10:
                voltage = 0.0
                # 범위를 벗어나면 제어기 내부 상태 리셋 (Wind-up 방지)
                xc = np.zeros(12, dtype=np.float64)
            else:
                # 음성 피드백 방향 확인: 논문 수식에 따라 부호가 반대여야 할 수 있음
                # 만약 작동 안하면 -1.0 * np.clip(u_raw, -VMAX, VMAX) 시도
                voltage = np.clip(u_raw, -VMAX, VMAX)
                
                # xc(k+1) = F * xc(k) + G * y(k)
                # 다음 루프를 위해 상태 업데이트
                xc = np.dot(F, xc) + np.dot(G, y)

            # 4. 실제 전압 출력
            myQube.write_voltage(voltage)

            # 스코프 샘플링
            scopeVoltage.sample(timeStamp, [voltage])
            scopePendulum.sample(timeStamp, [alpha])
            scopeBase.sample(timeStamp, [theta])

# 메인 스레드 실행
thread = Thread(target=control_loop)
thread.start()

while thread.is_alive() and (not KILL_THREAD):
    Scope.refreshAll()
    time.sleep(0.01)

input('Press the enter key to exit.')