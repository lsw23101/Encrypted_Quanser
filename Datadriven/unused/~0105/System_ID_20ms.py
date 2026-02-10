## lqr_observer_balance_control.py

import sys
sys.path.append(r"C:\Quanser\0_libraries\python")

from threading import Thread
import signal
import time
import math
import numpy as np
from pal.products.qube import QubeServo2, QubeServo3
from pal.utilities.scope import Scope

# 키보드 인터럽트 처리
global KILL_THREAD
KILL_THREAD = False
def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)

# region: Setup - 스코프 설정
simulationTime = 30 

scopePendulum = Scope(title='Pendulum Position - alpha (rad)', timeWindow=10, xLabel='Time (s)', yLabel='Position (rad)')
scopePendulum.attachSignal(name='Measured alpha', width=1)

scopeBase = Scope(title='Base Position - theta (rad)', timeWindow=10, xLabel='Time (s)', yLabel='Position (rad)')
scopeBase.attachSignal(name='Measured theta', width=1)

scopeVoltage = Scope(title='Motor Voltage (V)', timeWindow=10, xLabel='Time (s)', yLabel='Voltage (V)')
scopeVoltage.attachSignal(name='u_control', width=1)
#endregion




F = np.array([
    [ 0.975900, -0.748566, -1.294529, 0.760269 ],
    [ 0.331334, 0.000000, -0.102073, 0.000000 ],
    [ -0.011034, -0.038323, 0.975565, -0.657455 ],
    [ -0.024448, 0.000000, 0.402066, 0.000000 ]
], dtype=np.float64)

G = np.array([
    [ 0.782238, 0.281503 ],
    [ 0.668666, 0.102073 ],
    [ 0.051862, 0.666296 ],
    [ 0.024448, 0.597934 ]
], dtype=np.float64)

H = np.array([ -10.782541, 13.799790, -220.020533, 165.363313 ], dtype=np.float64)



def control_loop():
    global KILL_THREAD
    
    qubeVersion = 3
    frequency = 50  # 20ms (Ts = 0.02)
    
    # [수정] 4차 제어기 상태(xc) 초기화 (차원 12 -> 4)
    xc = np.zeros(4, dtype=np.float64)

    QubeClass = QubeServo3 if qubeVersion == 3 else QubeServo2

    # hardware=1(실제장비), pendulum=1(진자장착)
    with QubeClass(hardware=1, pendulum=1, frequency=frequency) as myQube:
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

            # 2. 동적 제어 연산 (4차 LQR + Observer)
            # u(k) = H * xc(k)
            u_raw = float(np.dot(H, xc))
            
            VMAX = 10.0

            # 3. 제어 범위 설정 (10도 이내에서만 작동)
            if alpha_degrees > 20:
                voltage = 0.0
                # 범위를 벗어나면 제어기 내부 상태 리셋 (4차로 리셋)
                xc = np.zeros(4, dtype=np.float64)
            else:
                # 계산된 전압 적용
                voltage = np.clip(u_raw, -VMAX, VMAX) #
                
                # [수정] xc(k+1) = F * xc(k) + G * y(k) 업데이트
                # xc_next를 먼저 계산하여 연산 순서 보장
                xc_next = np.dot(F, xc) + np.dot(G, y)
                xc = xc_next

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