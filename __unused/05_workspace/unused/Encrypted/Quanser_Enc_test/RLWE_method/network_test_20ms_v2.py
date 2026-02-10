# -*- coding: utf-8 -*-
## balance_control_qube_with_namedpipe.py
# 기존 TCP 통신을 Windows Named Pipe(\\.\pipe\qube_yu)로 교체
# 프로토콜: Python -> "theta,alpha\n", Go -> "u\n"

import sys
sys.path.append(r"C:\Quanser\0_libraries\python")

from threading import Thread, Event
import queue
import signal
import time
import math
import numpy as np
import io
import os

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

# ---- Named Pipe 설정 ----
PIPE_NAME = r'\\.\pipe\qube_yu'   # Go가 ListenPipe 하는 이름
PIPE_RETRY_SEC = 0.2              # 파이프 재접속 주기
READ_TIMEOUT_SEC = 0.2            # 응답 대기(스레드 내부 루프 템포)

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

# =========================
# Named Pipe 통신 스레드
# =========================
def pipe_client_worker(y_queue: "queue.Queue[tuple[float,float]]",
                       shared, stop_evt: Event):
    """
    - Go 파이프 서버(\\.\pipe\qube_yu)에 연결
    - y=(theta,alpha) 최신값을 받아 한 줄로 전송
    - 응답 u 한 줄을 읽어 shared['u']에 갱신
    """
    f = None  # type: io.BufferedRandom | None

    def try_open():
        nonlocal f
        while not stop_evt.is_set():
            try:
                # Go가 ListenPipe 중이어야 성공
                f = open(PIPE_NAME, 'r+b', buffering=0)
                print(f"[PIPE] Connected to {PIPE_NAME}")
                shared['connected'] = True
                return
            except OSError:
                shared['connected'] = False
                time.sleep(PIPE_RETRY_SEC)

    try_open()

    while not stop_evt.is_set():
        if f is None:
            try_open()
            continue

        # 최신 y만 사용 (밀린 값 버림)
        try:
            theta, alpha = y_queue.get(timeout=READ_TIMEOUT_SEC)
            # 큐에 더 남아있다면 최신으로 덮어쓰기
            while True:
                try:
                    theta, alpha = y_queue.get_nowait()
                except queue.Empty:
                    break
        except queue.Empty:
            # 주기적으로 연결 상태만 확인
            continue

        try:
            # write: "theta,alpha\n"
            line = f"{theta:.9f},{alpha:.9f}\n".encode('ascii')
            f.write(line)
            f.flush()

            # read one line: "u\n"
            resp = b''
            # 파이썬 open한 파이프는 블로킹. 20ms 제어 주기 보호를 위해 스레드에서만 블로킹.
            ch = f.readline()  # 한 줄
            if not ch:
                raise OSError("Pipe closed")
            resp += ch
            u_val = float(resp.strip())
            shared['u'] = u_val
            shared['last_ts'] = time.time()
        except Exception as e:
            print("[PIPE] error:", e)
            shared['connected'] = False
            shared['u'] = None
            # 파이프 재오픈 시도
            try:
                if f:
                    f.close()
            except Exception:
                pass
            f = None
            time.sleep(PIPE_RETRY_SEC)

    # 종료 정리
    try:
        if f:
            f.close()
    except Exception:
        pass
    print("[PIPE] worker stopped")

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

    # ---- (추가) Named Pipe 클라이언트 스레드 기동 ----
    y_queue: "queue.Queue[tuple[float,float]]" = queue.Queue(maxsize=1)
    shared = {'u': None, 'connected': False, 'last_ts': 0.0}
    stop_evt = Event()

    pipe_thread = Thread(target=pipe_client_worker, args=(y_queue, shared, stop_evt), daemon=True)
    pipe_thread.start()

    with QubeClass(hardware=hardware, pendulum=pendulum, frequency=frequency) as myQube:
        startTime = 0
        timeStamp = 0
        def elapsed_time():
            return time.time() - startTime
        startTime = time.time()

        while timeStamp < simulationTime and not KILL_THREAD:
            loop_t0 = time.time()

            # === 측정 ===
            myQube.read_outputs()
            theta = myQube.motorPosition * -1
            alpha_f =  myQube.pendulumPosition

            # 출력 값 전처리
            alpha = math.atan2(math.sin(alpha_f - math.pi), math.cos(alpha_f - math.pi))
            alpha_degrees = abs(math.degrees(alpha))
            y = np.array([theta, alpha], dtype=np.float64)

            # 속도 추정
            theta_dot, state_theta_dot = ddt_filter(theta, state_theta_dot, 50, 1/frequency)
            alpha_dot, state_alpha_dot = ddt_filter(alpha, state_alpha_dot, 100, 1/frequency)
            x_meas = np.array([theta, alpha, theta_dot, alpha_dot], dtype=np.float64)

            # ---- 로컬 제어입력 ----
            u_local = -1 * float(H @ xhat)  # 로컬 u
            VMAX = 10.0

            # ---- (추가) y를 파이프 스레드에 전달 ----
            try:
                # 큐가 차 있으면 이전 y 버리고 최신으로 교체
                if y_queue.full():
                    _ = y_queue.get_nowait()
                y_queue.put_nowait((theta, alpha))
            except queue.Full:
                pass

            # ---- 원격 u 사용 (폴백: 로컬 u) ----
            u_remote = shared['u']
            if u_remote is None:
                u_to_apply = u_local
            else:
                u_to_apply = u_remote

            print("udiff", u_local-u_remote)
            if alpha_degrees > 40:
                voltage = 0.0
                error = 0.0
                xhat = np.array([0, 0, 0, 0], dtype=np.float64)
            else:
                voltage = float(np.clip(u_to_apply, -VMAX, VMAX))
                # 옵저버 업데이트
                xhat = F @ xhat + G @ y

            # === 전압 인가 ===
            myQube.write_voltage(voltage)

            # === 스코프 ===
            count += 1
            if count >= countMax:
                scopeVoltage.sample(timeStamp, [u_to_apply if u_remote is not None else u_local])
                scopePendulum.sample(timeStamp, [alpha, xhat[1]])
                scopeBase.sample(timeStamp, [theta, xhat[0]])
                count = 0

            # 루프 시간 갱신
            timeStamp = elapsed_time()

            # 20ms 유지(필요 시 미세 슬립)
            dt = time.time() - loop_t0
            remain = max(0.0, (1.0/frequency) - dt)
            if remain > 0:
                time.sleep(remain)

    # 종료 시 스레드 정리
    stop_evt.set()
    pipe_thread.join(timeout=1.0)

# ---- 스레드 기동 ----
thread = Thread(target=control_loop)
thread.start()

while thread.is_alive() and (not KILL_THREAD):
    Scope.refreshAll()
    time.sleep(0.01)

input('Press the enter key to exit.')
