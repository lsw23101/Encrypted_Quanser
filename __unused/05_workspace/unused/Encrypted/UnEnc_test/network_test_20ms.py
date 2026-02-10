# -*- coding: utf-8 -*-
## balance_control_qube_with_tcp.py
# 기존 코드에 "TCP 통신(127.0.0.1:9000)으로 y를 보내고 u를 받는" 부분만 추가

import sys
sys.path.append(r"C:\Quanser\0_libraries\python")

from threading import Thread
import signal
import time
import math
import numpy as np
import socket  # ★ 추가: TCP 통신
from pal.products.qube import QubeServo2, QubeServo3
from pal.utilities.math import SignalGenerator, ddt_filter
from pal.utilities.scope import Scope

# ---- 글로벌 종료 핸들러 ----
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

# ---- TCP 설정 ----
TCP_HOST = "127.0.0.1"
TCP_PORT = 9000
TCP_TIMEOUT_S = 0.005   # 로컬호스트이므로 왕복 5ms 내외 목표 / 타임아웃시 u_remote 무시

# region: Setup (스코프 원형 유지)
simulationTime = 200
color = np.array([0, 1, 0], dtype=np.float64)

scopePendulum = Scope(
    title='Pendulum encoder - alpha (rad)',
    timeWindow=10, xLabel='Time (s)', yLabel='Position (rad)')
scopePendulum.attachSignal(name='Pendulum - alpha (rad)',  width=1)

scopeBase = Scope(
    title='Base encoder - theta (rad)',
    timeWindow=10, xLabel='Time (s)', yLabel='Position (rad)')
scopeBase.attachSignal(name='Base - theta (rad)',  width=1)

scopeVoltage = Scope(
    title='Motor Voltage',
    timeWindow=10, xLabel='Time (s)', yLabel='Voltage (volts)')
scopeVoltage.attachSignal(name='Voltage',  width=1)

scopeError = Scope(
    title='Control Error (u - voltage)',
    timeWindow=10, xLabel='Time (s)', yLabel='Error (V)')
scopeError.attachSignal(name='u - voltage', width=1)

# (추가) xhat 모니터링
scopePendulum.attachSignal(name='alpha_hat (rad)', width=1, color=np.array([0, 0, 255]))
scopeBase.attachSignal(name='theta_hat (rad)',  width=1, color=np.array([0, 0, 255]))
#endregion

'''
이산시간 20ms 모델링 (생략 주석)
'''

F = np.array([
    [ 0.0620, -0.3381, 0.0318, -0.0202 ],
    [ -0.0258, -0.3867, 0.0117, 0.0001 ],
    [ -7.8511, -27.7120, 2.1869, -2.0175 ],
    [ 2.4549, -41.8777, 1.1819, -0.9864 ]
], dtype=np.float64)

G = np.array([
    [ 0.9798, 0.1224 ],
    [ 0.0673, 1.1956 ],
    [ 12.0512, 6.0503 ],
    [ 1.7277, 22.6045 ]
], dtype=np.float64)

H = np.array([ 4.1822, -24.5940, 1.1820, -2.0389 ], dtype=np.float64)  # shape (4,)

# ---- (추가) TCP 유틸 ----
def open_go_conn():
    """Go 서버에 연결하고 (reader, writer, socket)을 돌려준다. 실패 시 (None, None, None)."""
    try:
        sock = socket.create_connection((TCP_HOST, TCP_PORT), timeout=1.0)  # 최초 연결은 1초까지 대기
        sock.settimeout(TCP_TIMEOUT_S)
        r = sock.makefile('rb', buffering=0)
        w = sock.makefile('wb', buffering=0)
        return r, w, sock
    except OSError:
        return None, None, None

def send_y_recv_u(reader, writer, theta, alpha):
    """
    y=[theta, alpha]를 'theta,alpha\n' 형태로 전송하고 'u\n'을 수신.
    타임아웃/에러 시 None 반환.
    """
    try:
        line = f"{theta:.9f},{alpha:.9f}\n".encode('ascii')
        writer.write(line)
        writer.flush()
        # 응답 한 줄
        resp = b''
        while not resp.endswith(b'\n'):
            chunk = reader.readline()  # socket timeout 적용
            if not chunk:
                return None
            resp += chunk
        u_remote = float(resp.strip())
        return u_remote
    except Exception:
        return None

# Code to control the Qube Hardware
def control_loop():
    qubeVersion = 3
    hardware = 1
    pendulum = 1

    frequency = 50  # 20ms
    state_theta_dot = np.array([0,0], dtype=np.float64)
    state_alpha_dot = np.array([0,0], dtype=np.float64)

    countMax = max(1, int(round(frequency / 50)))
    count = 0

    if qubeVersion == 2:
        QubeClass = QubeServo2
        K = np.array([-1, 34.75, -1.495, 3.111]) # 사용 x
    else:
        QubeClass = QubeServo3
        K = np.array([-1.1903, 15.4953, -0.4840, 1.1720]) # 사용 x
 
    xhat = np.zeros(4, dtype=np.float64)

    # ---- (추가) Go 서버와 TCP 연결 시도 ----
    go_reader, go_writer, go_sock = open_go_conn()
    if go_sock is None:
        print(f"[TCP] {TCP_HOST}:{TCP_PORT} 연결 실패. 통신 없이 로컬 제어만 수행합니다.")
    else:
        print(f"[TCP] Connected to Go controller at {TCP_HOST}:{TCP_PORT}")

    with QubeClass(hardware=hardware, pendulum=pendulum, frequency=frequency) as myQube:
        startTime = 0
        timeStamp = 0
        def elapsed_time():
            return time.time() - startTime
        startTime = time.time()

        last_u_remote = None  # 수신 성공 시 갱신 (장비에는 쓰지 않음)

        while timeStamp < simulationTime and not KILL_THREAD:
            # === 측정 ===
            myQube.read_outputs()
            theta = myQube.motorPosition * -1
            alpha_f =  myQube.pendulumPosition

            # 출력 값 전처리
            alpha = math.atan2(math.sin(alpha_f - math.pi), math.cos(alpha_f - math.pi))
            alpha_degrees = abs(math.degrees(alpha))  # 임계 판단용 절댓값은 그대로
            # 플랜트 출력 y(t) = [theta, alpha]

            y = np.array([theta, alpha], dtype=np.float64)

            # 속도 추정
            theta_dot, state_theta_dot = ddt_filter(theta, state_theta_dot, 50, 1/frequency)
            alpha_dot, state_alpha_dot = ddt_filter(alpha, state_alpha_dot, 100, 1/frequency)
            x_meas = np.array([theta, alpha, theta_dot, alpha_dot], dtype=np.float64)

            # ---- 로컬 제어입력 (기존 유지) ----
            u = -1 * float(H @ xhat)  # 로컬 u
            VMAX = 10.0

            if alpha_degrees > 40:
                voltage = 0.0
                error = 0.0
            else:
                voltage = float(np.clip(u, -VMAX, VMAX))
                # 여기서는 여전히 'u'(로컬)만 장비에 인가

            # ---- (추가) TCP로 y 전송 & u_remote 수신 (미사용) ----
            if go_sock is not None:
                u_remote = send_y_recv_u(go_reader, go_writer, theta, alpha)
                if u_remote is not None:
                    last_u_remote = u_remote  # 필요 시 로깅/모니터링에 활용 가능
                # 수신 실패해도 제어 흐름 영향 없음

            # === 전압 인가 (기존 유지) ===
            # myQube.write_voltage(u)  # ★ 아직은 로컬 u만 사용
            print("u_diff:", u - u_remote)
            # myQube.write_voltage(u_remote)  # 전송 받은 u
            voltage_remote = float(np.clip(u_remote, -VMAX, VMAX))
            myQube.write_voltage(voltage_remote)  # 필요시 클램핑된 전압 사용

            # === 옵저버 업데이트 (기존 유지) ===
            if alpha_degrees > 40:
                xhat = np.array([0, 0, 0, 0], dtype=np.float64)
            else:
                xhat = F @ xhat + G @ y

            # === 스코프 (기존 유지) ===
            count += 1
            if count >= countMax:
                scopeVoltage.sample(timeStamp, [u_remote])
                # scopeError.sample(timeStamp, [u])   # 여전히 로컬 u를 표시
                scopePendulum.sample(timeStamp, [alpha, xhat[1]])
                scopeBase.sample(timeStamp, [theta, xhat[0]])
                count = 0

            timeStamp = elapsed_time()

    # 종료 시 소켓 정리
    try:
        if go_sock is not None:
            go_sock.close()
    except Exception:
        pass

# ---- 스레드 기동 (기존 유지) ----
thread = Thread(target=control_loop)
thread.start()

while thread.is_alive() and (not KILL_THREAD):
    Scope.refreshAll()
    time.sleep(0.01)

input('Press the enter key to exit.')
