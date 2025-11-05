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
import socket  # ★ TCP 통신
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
TCP_TIMEOUT_S = 0.01   # 로컬호스트이므로 왕복 5ms 내외 목표 / 타임아웃시 u_remote 무시

# ---- 전송 제한 각도(도) ----
ALPHA_DEG_LIMIT = 40.0

# region: Setup
simulationTime = 200
color = np.array([0, 1, 0], dtype=np.float64)

# ---- 스코프 선언: 각 플랏에 2개 시그널 ----
scopePendulum = Scope(
    title='Pendulum encoder - alpha (rad)',
    timeWindow=10, xLabel='Time (s)', yLabel='Position (rad)')
scopePendulum.attachSignal(name='alpha (rad)',  width=1)
scopePendulum.attachSignal(name='alpha_hat (rad)', width=1, color=np.array([0, 0, 255]))

scopeBase = Scope(
    title='Base encoder - theta (rad)',
    timeWindow=10, xLabel='Time (s)', yLabel='Position (rad)')
scopeBase.attachSignal(name='theta (rad)',  width=1)
scopeBase.attachSignal(name='theta_hat (rad)',  width=1, color=np.array([0, 0, 255]))

scopeVoltage = Scope(
    title='Motor Voltage',
    timeWindow=10, xLabel='Time (s)', yLabel='Voltage (V)')
scopeVoltage.attachSignal(name='u_local (V)',  width=1)
scopeVoltage.attachSignal(name='u_remote (V)', width=1, color=np.array([255, 0, 0]))
#endregion

'''
이산시간 5ms 모델링 (생략 주석)
'''
F = np.array([
    [ -0.3079, -0.2861, 0.0318, -0.0202 ],
    [ 0.0302, -0.6481, 0.0117, 0.0001 ],
    [ -18.5838, -26.9833, 2.1893, -2.0234 ],
    [ 3.7786, -50.7604, 1.1844, -0.9922 ]
], dtype=np.float64)

G = np.array([
    [ 1.3496, 0.0690 ],
    [ 0.0111, 1.4557 ],
    [ 22.7723, 5.1868 ],
    [ 0.3926, 31.3530 ]
], dtype=np.float64)

H = np.array([ 4.1707, -24.7282, 1.1845, -2.0448 ], dtype=np.float64)  # shape (4,)

# ---- TCP 유틸 ----
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
    y=[theta, alpha]를 'theta,alpha\\n' 형태로 전송하고 'u\\n'을 수신.
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

    '''
    주파수 설정 !!!
    '''
    frequency = 100  # 10ms
    
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

    # ---- Go 서버와 TCP 연결 시도 ----
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

        # 안전 기본값
        u_remote = 0.0
        last_u_remote = 0.0

        while timeStamp < simulationTime and not KILL_THREAD:
            # === 측정 ===
            myQube.read_outputs()
            theta = myQube.motorPosition * -1
            alpha_f =  myQube.pendulumPosition

            # 출력 값 전처리
            alpha = math.atan2(math.sin(alpha_f - math.pi), math.cos(alpha_f - math.pi))
            alpha_degrees = abs(math.degrees(alpha))  # 임계 판단용 절댓값
            y = np.array([theta, alpha], dtype=np.float64)

            # 속도 추정
            theta_dot, state_theta_dot = ddt_filter(theta, state_theta_dot, 50, 1/frequency)
            alpha_dot, state_alpha_dot = ddt_filter(alpha, state_alpha_dot, 100, 1/frequency)
            x_meas = np.array([theta, alpha, theta_dot, alpha_dot], dtype=np.float64)

            # 로컬 제어입력
            u = -1 * float(H @ xhat)
            VMAX = 10.0

            # TCP로 보낼 y 선택: |alpha_deg| <= 30° 이면 실제 값, 아니면 0 전송
            if alpha_degrees <= ALPHA_DEG_LIMIT:
                theta_tx, alpha_tx = theta, alpha
            else:
                theta_tx, alpha_tx = 0.0, 0.0

            # TCP로 y 전송 & u_remote 수신
            if go_sock is not None:
                ur = send_y_recv_u(go_reader, go_writer, theta_tx, alpha_tx)
                if ur is not None:
                    u_remote = ur
                    last_u_remote = ur
                else:
                    u_remote = last_u_remote  # 실패 시 직전값 유지

            # 전압 인가
            
            u_remote = -1*u_remote

            # 예외처리
            if alpha_degrees > ALPHA_DEG_LIMIT:
                u_remote = 0
            voltage_remote = float(np.clip(u_remote, -VMAX, VMAX))
            # voltage_remote = float(np.clip(u, -VMAX, VMAX))
            myQube.write_voltage(voltage_remote) 

            # 옵저버 업데이트
            if alpha_degrees > ALPHA_DEG_LIMIT:
                xhat = np.array([0, 0, 0, 0], dtype=np.float64)
            else:
                xhat = F @ xhat + G @ y

            # print("u_diff:", u - u_remote)

            # 스코프 (각 플랏 2시그널)
            count += 1
            if count >= countMax:
                scopeVoltage.sample(timeStamp, [u, u_remote])   # u_local, u_remote
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

# ---- 스레드 기동 ----
thread = Thread(target=control_loop)
thread.start()

while thread.is_alive() and (not KILL_THREAD):
    Scope.refreshAll()
    time.sleep(0.01)

input('Press the enter key to exit.')
