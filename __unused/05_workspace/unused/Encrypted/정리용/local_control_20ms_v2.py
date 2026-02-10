## example balance_control_qube_with_flags.py
# This example does balance control of the Qube Servo's Pendulum attachment.
# Adds safety/fault flag monitoring & minimal handling.
# - Logs amplifierFault / motorStallDetected / motorStallError
# - Plots flags to a separate scope (0 or 1)
# - On amplifierFault or motorStallError, forces 0V and tries re-enable (if supported)

# imports

# 퀀서 라이브러리 쓰기 위해서 추가함
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

def flag(x):
    """array/스칼라 구분 없이 첫 요소를 bool로 반환"""
    try:
        return bool(np.asarray(x).astype(int).flat[0])
    except Exception:
        return bool(x)
    



#################
# 본 실험 장비의 엔코더는 절대 측정이 아님
# 초기 각도를 -180도라고 보고, 초기 위치를 0이라고 봄 >> 0도 위치에서 아래로 내려놓고 시작해야 평형점이 제대로 들어감
##########

# region: Setup
simulationTime = 500  # sec
color = np.array([0, 1, 0], dtype=np.float64)

# Fault/안전 이벤트 내역(최근 실행 기준)
fault_history = []

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

# (추가) xhat(옵저버 상태) 모니터링을 위해 스코프에 추정값도 같이 띄우자
scopePendulum.attachSignal(
    name='alpha_hat (rad)',
    width=1,
    color=np.array([0, 0, 255])
)
scopeBase.attachSignal(
    name='theta_hat (rad)',
    width=1,
    color=np.array([0, 0, 255])
)

# (추가) 안전 플래그 스코프
scopeFlags = Scope(
    title='Safety / Fault Flags (0/1)',
    timeWindow=10,
    xLabel='Time (s)',
    yLabel='flag')
scopeFlags.attachSignal(name='AmplifierFault', width=1)
scopeFlags.attachSignal(name='MotorStallDetected', width=1)
scopeFlags.attachSignal(name='MotorStallError', width=1)
# endregion


'''
이산시간 20ms 모델링 (예시 행렬)
'''
# pre-designed controller  (Design_obsv_LQR_20ms.m 결과)
F = np.array([
    [ 0.3159, -0.8237, 0.0756, -0.0696 ],
    [ 0.2410, -0.7588, 0.0552, -0.0489 ],
    [ 14.0177, -79.5790, 6.5861, -6.9828 ],
    [ 24.6510, -90.8119, 5.5629, -5.9310 ]
], dtype=np.float64)

G = np.array([
    [ 0.9315, 0.0799 ],
    [ 0.0045, 1.0438 ],
    [ 10.8468, 4.8520 ],
    [ 0.1103, 18.6934 ]
], dtype=np.float64)

H = np.array([ 24.7585, -77.4332, 5.5625, -6.9830 ], dtype=np.float64)  # shape (4,)

# Code to control the Qube Hardware
def control_loop():
    # set as 2 or 3 if using a Qube Servo 2 or 3 respectively
    qubeVersion = 3

    # Set as 0 if using virtual Qube Servo
    # Set as 1 if using physical Qube Servo
    hardware = 1

    # Only matters when using virtual Qube.
    # Set as 0 for virtual DC Motor and 1 for virtual pendulum
    pendulum = 1

    # 샘플링타임
    frequency = 50  # 20ms

    state_theta_dot = np.array([0,0], dtype=np.float64)
    state_alpha_dot = np.array([0,0], dtype=np.float64)

    # Limit sample rate for scope
    countMax = max(1, int(round(frequency / 50)))
    count = 0

    if qubeVersion == 2:
        QubeClass = QubeServo2
        K = np.array([-1, 34.75, -1.495, 3.111])  # unused
    else:
        QubeClass = QubeServo3
        # K = np.array([-1.1903, 15.4953, -0.4840, 1.1720])  # unused
        K = np.array([ 2.0126, -18.0705, 0.6670, -1.4236 ])  # test, unused

    # 옵저버 상태 초기화
    xhat = np.zeros(4, dtype=np.float64)

    # 실행마다 이벤트 기록 초기화
    fault_history.clear()

    with QubeClass(hardware=hardware, pendulum=pendulum, frequency=frequency) as myQube:

        startTime = 0
        timeStamp = 0
        def elapsed_time():
            return time.time() - startTime
        startTime = time.time()

        # 안전 플래그 상승엣지 감지용
        prev_amp_fault = False
        prev_stall_det = False
        prev_stall_err = False

        # 제어 입력 초기값 (로그 출력용)
        u = 0.0
        voltage = 0.0

        while timeStamp < simulationTime and not KILL_THREAD:
            loopTime = time.time()

            # Read sensor information (updates flags, positions, etc.)
            myQube.read_outputs()

            # --- 안전 플래그 읽기 ---
            amp_fault = flag(myQube.amplifierFault)
            stall_det = flag(myQube.motorStallDetected)
            stall_err = flag(myQube.motorStallError)    

            # 상승엣지 로깅
            if amp_fault and not prev_amp_fault:
                message = f"[SAFETY] Amplifier FAULT at t={timeStamp:.3f}s (thermal/driver protection)"
                print(message)
                fault_history.append(message)
            if stall_det and not prev_stall_det:
                message = f"[WARN] Motor STALL detected at t={timeStamp:.3f}s (>~5V with stalled/very slow motor)"
                print(message)
                fault_history.append(message)
            if stall_err and not prev_stall_err:
                message = f"[ERROR] Motor STALL persisted (~3s) at t={timeStamp:.3f}s (driver may cut output)"
                print(message)
                fault_history.append(message)

            # 스톨 감지 순간에 주변 상태 같이 찍기 (루프 안, stall_det가 True로 바뀌는 곳)
            if stall_det and not prev_stall_det:
                message_parts = (
                    f"[WARN] STALL @ {timeStamp:.3f}s",
                    f"voltage={voltage:.2f}V u={u:.2f}V",
                    f"motorSpeed={getattr(myQube,'motorSpeed',0):.3f} rad/s",
                    f"motorCurrent={float(myQube.motorCurrent):.3f} A",
                )
                message = " ".join(part.strip() for part in message_parts)
                print(message)
                fault_history.append(message)
                
            prev_amp_fault, prev_stall_det, prev_stall_err = amp_fault, stall_det, stall_err

            theta = myQube.motorPosition * -1
            alpha_f =  myQube.pendulumPosition

            # 출력 값 전처리
            alpha = (alpha_f - np.pi + np.pi) % (2*np.pi) - np.pi  # rad
            alpha_degrees = abs(math.degrees(alpha))

            # 플랜트 출력 y(t) = [theta, alpha]
            y = np.array([theta, alpha], dtype=np.float64)

            # 실제 state와 비교하기 위한 측정 state 값
            theta_dot, state_theta_dot = ddt_filter(theta, state_theta_dot, 50, 1/frequency)
            alpha_dot, state_alpha_dot = ddt_filter(alpha, state_alpha_dot, 100, 1/frequency)
            x_meas = np.array([theta, alpha, theta_dot, alpha_dot], dtype=np.float64)

            # 레퍼런스 각도 # 안쓰는 중
            command_deg = 0
            states = command_deg*np.array([np.pi/180, 0, 0, 0]) - np.array([theta, alpha, theta_dot, alpha_dot])

            # 제어 입력 연산 (옵저버 기반)
            u = -1.0 * float(H @ xhat)
            # u = -1.0 * float(H @ x_meas)  # 측정 기반으로 테스트하려면 이 줄 사용

            VMAX = 10.0  # 제어 입력 saturation

            # 각도가 10도 이내일때만 제어 입력 진행
            if alpha_degrees > 10:
                voltage = 0.0
                error = 0.0
            else:
                voltage = float(np.clip(u, -VMAX, VMAX))

            # (중요) Fault/에러 시 0V 보장 + (가능하면) 앰프 재활성 시도
            if amp_fault or stall_err:
                # 우선 0V로 보장
                myQube.write_voltage(0.0)
                # 선택: 앰프 disable→enable 재시도 (드라이버가 허용할 때만)
                try:
                    myQube.card.write_digital(
                        myQube.WRITE_DIGITAL_CHANNELS,
                        len(myQube.WRITE_DIGITAL_CHANNELS),
                        np.array([0], dtype=np.int8)
                    )
                    time.sleep(0.2)
                    myQube.card.write_digital(
                        myQube.WRITE_DIGITAL_CHANNELS,
                        len(myQube.WRITE_DIGITAL_CHANNELS),
                        np.array([1], dtype=np.int8)
                    )
                    print("[SAFETY] Tried re-enabling amplifier after fault/error.")
                except Exception:
                    pass
            else:
                # 정상 상황에서는 계산된 전압 인가
                myQube.write_voltage(voltage)

            # 상태 업데이트 (루프 마지막에 추가)
            if alpha_degrees > 5:
                xhat = np.array([0, 0, 0, 0], dtype=np.float64)
            else:
                xhat = F @ xhat + G @ y

            # Plot to scopes
            count += 1
            if count >= countMax:
                # 전압
                scopeVoltage.sample(timeStamp, [voltage])
                # (원래 제목은 u-voltage 이지만, 기존처럼 u값 또는 원하는 값 표시)
                scopeError.sample(timeStamp,    [u])

                # 펜듈럼: alpha(측정), alpha_hat(추정)
                scopePendulum.sample(timeStamp, [x_meas[1], xhat[1]])
                # 베이스: theta(측정), theta_hat(추정)
                scopeBase.sample(timeStamp, [x_meas[0], xhat[0]])

                # (추가) 안전 플래그 표시 (0/1)
                scopeFlags.sample(timeStamp, [
                    1.0 if amp_fault else 0.0,
                    1.0 if stall_det else 0.0,
                    1.0 if stall_err else 0.0
                ])

                count = 0

            loopTime_end = time.time()
            timeStamp = elapsed_time()
            if count % 100 == 0:
                print("looptime", loopTime_end - loopTime)

        if fault_history:
            print("[SUMMARY] Fault/Fault-like events captured:")
            for entry in fault_history:
                print("  ", entry)
        else:
            print("[SUMMARY] Fault/Fault-like events not detected during run.")


# Setup data generation thread and run until complete
thread = Thread(target=control_loop)
thread.start()

while thread.is_alive() and (not KILL_THREAD):
    # This must be called regularly or the scope windows will freeze
    # Must be called in the main thread.
    Scope.refreshAll()
    time.sleep(0.01)

input('Press the enter key to exit.')
