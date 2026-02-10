## balance_control_qube_advanced_data_logging.py

import sys
sys.path.append(r"C:\Quanser\0_libraries\python")

from threading import Thread
import signal
import time
import math
import numpy as np
import pandas as pd
from pal.products.qube import QubeServo2, QubeServo3
from pal.utilities.math import ddt_filter
from pal.utilities.scope import Scope

# 키보드 인터럽트 설정
global KILL_THREAD
KILL_THREAD = False
def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)

data_log = []

# region: Scope Setup
simulationTime = 60 # 데이터 확보를 위해 60초 권장
scopePendulum = Scope(title='Pendulum Position', timeWindow=10, xLabel='Time (s)', yLabel='Position (rad)')
scopePendulum.attachSignal(name='alpha', width=1)
scopePendulum.attachSignal(name='alpha_hat', width=1, color=np.array([0, 0, 255]))

scopeBase = Scope(title='Base Position', timeWindow=10, xLabel='Time (s)', yLabel='Position (rad)')
scopeBase.attachSignal(name='theta', width=1)
scopeBase.attachSignal(name='theta_hat', width=1, color=np.array([0, 0, 255]))

scopeVoltage = Scope(title='Motor Voltage', timeWindow=10, xLabel='Time (s)', yLabel='Voltage (V)')
scopeVoltage.attachSignal(name='u_noisy', width=1)
# endregion

# 기존 설계된 밸런싱 제어기 (데이터 수집 중 중심을 잡기 위함)
F = np.array([[0.6951, -0.8117, 0.0756, -0.0696], [0.2336, -0.3680, 0.0552, -0.0489], [21.0411, -78.7214, 6.5861, -6.9828], [24.5716, -82.6764, 5.5629, -5.9310]], dtype=np.float64)
G = np.array([[0.5524, 0.0679], [0.0119, 0.6530], [3.8233, 3.9944], [0.1897, 10.5578]], dtype=np.float64)
H = np.array([24.7585, -77.4332, 5.5625, -6.9830], dtype=np.float64)

def control_loop():
    global KILL_THREAD, data_log

    qubeVersion = 3
    frequency = 50 
    Ts = 1/frequency

    # 노이즈 유지를 위한 변수
    noise_hold_ticks = 8    # 8틱(160ms) 동안 노이즈 유지
    tick_counter = 0
    current_noise = 0.0

    xhat = np.zeros(4, dtype=np.float64)
    QubeClass = QubeServo3 if qubeVersion == 3 else QubeServo2

    with QubeClass(hardware=1, pendulum=1, frequency=frequency) as myQube:
        startTime = time.time()
        
        while (time.time() - startTime) < simulationTime and not KILL_THREAD:
            timeStamp = time.time() - startTime
            myQube.read_outputs()

            theta = myQube.motorPosition * -1
            alpha = (myQube.pendulumPosition - np.pi + np.pi) % (2*np.pi) - np.pi
            alpha_degrees = abs(math.degrees(alpha))
            y = np.array([theta, alpha], dtype=np.float64)

            # --- [고급 노이즈 생성] ---
            # 매번 바뀌는 가우시안 대신, 특정 시간 동안 유지되는 유니폼 노이즈 사용
            if tick_counter % noise_hold_ticks == 0:
                # ±0.8V 범위의 무작위 전압 (마찰을 이기기에 적절한 세기)
                current_noise = np.random.uniform(-0.3, 0.3)
            tick_counter += 1

            u_base = -1 * float(np.dot(H, xhat))
            u_noisy = u_base + current_noise
            
            VMAX = 10.0
            if alpha_degrees > 15:
                u_final = 0.0
                xhat = np.zeros(4)
            else:
                u_final = np.clip(u_noisy, -VMAX, VMAX)

            myQube.write_voltage(u_final)

            # 옵저버 업데이트
            xhat = np.dot(F, xhat) + np.dot(G, y)

            # 데이터 기록
            data_log.append([timeStamp, u_final, theta, alpha])

            # 스코프 출력
            scopeVoltage.sample(timeStamp, [u_final])
            scopePendulum.sample(timeStamp, [alpha, xhat[1]])
            scopeBase.sample(timeStamp, [theta, xhat[0]])

# 실행
thread = Thread(target=control_loop)
thread.start()

try:
    while thread.is_alive() and not KILL_THREAD:
        Scope.refreshAll()
        time.sleep(0.01)
except KeyboardInterrupt:
    KILL_THREAD = True

thread.join()

# CSV 저장
if data_log:
    print("\n[저장 중] qube_data.csv...")
    df = pd.DataFrame(data_log, columns=['Time', 'u_noisy', 'theta', 'alpha'])
    df.to_csv("qube_data.csv", index=False)
    print("저장 완료!")

input('Press Enter to exit.')