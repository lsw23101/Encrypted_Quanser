# example balance_control_qube.py
# Balance control of the Qube Servo's Pendulum attachment
# 5 ms controller sampling, 2.5 ms actuation delay (1 step at 400 Hz)

import sys
sys.path.append(r"C:\Quanser\0_libraries\python")

from threading import Thread
import signal
import time
import math
import numpy as np
from pal.products.qube import QubeServo2, QubeServo3
from pal.utilities.math import ddt_filter
from pal.utilities.scope import Scope

# Setup to enable killing the data generation thread using keyboard interrupts
global KILL_THREAD
KILL_THREAD = False
def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)

#################
# 본 실험 장비의 엔코더는 절대 측정이 아님
# 초기 각도를 -180도라고 보고, 초기 위치를 0이라고 봄
#################

# region: Setup
simulationTime = 30  # will run for 30 seconds

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
    title='Control Input (u)',
    timeWindow=10,
    xLabel='Time (s)',
    yLabel='u (V)')
scopeError.attachSignal(name='u', width=1)

# 옵저버 상태 모니터링
scopePendulum.attachSignal(
    name='alpha_hat (rad)',
    width=1,
    color=np.array([0, 0, 255]))
scopeBase.attachSignal(
    name='theta_hat (rad)',
    width=1,
    color=np.array([0, 0, 255]))
#endregion


# pre-designed observer-based controller
F = np.array([
    [ 0.6951, -0.8117, 0.0756, -0.0696 ],
    [ 0.2336, -0.3680, 0.0552, -0.0489 ],
    [ 21.0411, -78.7214, 6.5861, -6.9828 ],
    [ 24.5716, -82.6764, 5.5629, -5.9310 ]
], dtype=np.float64)

G = np.array([
    [ 0.5524, 0.0679 ],
    [ 0.0119, 0.6530 ],
    [ 3.8233, 3.9944 ],
    [ 0.1897, 10.5578 ]
], dtype=np.float64)

H = np.array([ 24.7585, -77.4332, 5.5625, -6.9830 ], dtype=np.float64)


def control_loop():
    qubeVersion = 3
    hardware = 1
    pendulum = 1

    # 하드웨어는 400 Hz로 돌림 (2.5 ms 주기)
    frequency = 400

    state_theta_dot = np.array([0, 0], dtype=np.float64)
    state_alpha_dot = np.array([0, 0], dtype=np.float64)

    countMax = frequency / 50  # 스코프는 50 Hz로 제한
    count = 0

    if qubeVersion == 2:
        QubeClass = QubeServo2
    else:
        QubeClass = QubeServo3

    # 옵저버 상태 초기화
    xhat = np.zeros(4, dtype=np.float64)

    # (추가) 지연 버퍼 초기화 (1스텝 = 2.5 ms 지연)
    u_buffer = [0.0]
    step = 0

    with QubeClass(hardware=hardware, pendulum=pendulum, frequency=frequency) as myQube:

        startTime = time.time()
        timeStamp = 0

        def elapsed_time():
            return time.time() - startTime

        while timeStamp < simulationTime and not KILL_THREAD:

            # Read sensor info every 2.5 ms
            myQube.read_outputs()

            theta = myQube.motorPosition * -1
            alpha_f = myQube.pendulumPosition
            alpha = (alpha_f - np.pi + np.pi) % (2*np.pi) - np.pi
            alpha_degrees = abs(math.degrees(alpha))

            y = np.array([theta, alpha], dtype=np.float64)

            # Derivatives
            theta_dot, state_theta_dot = ddt_filter(theta, state_theta_dot, 50, 1/frequency)
            alpha_dot, state_alpha_dot = ddt_filter(alpha, state_alpha_dot, 100, 1/frequency)

            # 5 ms마다 새로운 제어 입력 계산 (2*2.5ms)
            if step % 2 == 0:
                if alpha_degrees > 5:
                    xhat = np.zeros(4, dtype=np.float64)
                else:
                    xhat = F @ xhat + G @ y

                u_new = -1 * float(H @ xhat)
                VMAX = 10.0
                u_new = np.clip(u_new, -VMAX, VMAX)

                # (추가) 버퍼에 저장 → 한 스텝(2.5 ms) 뒤에 적용됨
                u_buffer.append(u_new)

            # 항상 버퍼에서 꺼내서 출력 (없으면 0 V)
            if len(u_buffer) > 0:
                u_delayed = u_buffer.pop(0)
            else:
                u_delayed = 0.0

            myQube.write_voltage(u_delayed)

            # 스코프 기록은 50 Hz만
            count += 1
            if count >= countMax:
                scopeVoltage.sample(timeStamp, [u_delayed])
                scopeError.sample(timeStamp, [u_delayed])
                scopePendulum.sample(timeStamp, [alpha, xhat[1]])
                scopeBase.sample(timeStamp, [theta, xhat[0]])
                count = 0

            step += 1
            timeStamp = elapsed_time()


# Setup data generation thread and run until complete
thread = Thread(target=control_loop)
thread.start()

while thread.is_alive() and (not KILL_THREAD):
    Scope.refreshAll()
    time.sleep(0.01)

input('Press the enter key to exit.')
