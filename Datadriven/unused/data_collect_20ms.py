## example balance_control_qube.py
# This example does balance control of the Qube Servo's Pendulum attachment.
# This example uses either a virtual or Physical Qube Servo 2 or Qube Servo 3 device,
# in a task-based (time-based IO) mode where you do not have to handle timing yourself.
# (task based mode is recommended for most applications).

# IF USING HARDWARE, LIFT THE PENDULUM MANUALLY FOR THE CONTROLLER TO KICK IN
# IF USING VIRTUAL, USE THE LIFT PENDULUM BUTTON IN QUANSER INTERACTIVE LABS
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

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
import csv  # CSV 저장

# Setup to enable killing the data generation thread using keyboard interrupts
global KILL_THREAD
KILL_THREAD = False
def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)

#################
# 본 실험 장비의 엔코더는 절대 측정이 아님
# 초기 각도를 -180도라고 보고, 초기 위치를 0이라고 봄 >> 0도 위치에서 아래로 내려놓고 시작해야 평형점이 제대로 들어감
##########

# region: Setup
simulationTime = 50000 # will run for 30 seconds
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

# xhat(옵저버 상태) 모니터링용 신호
scopePendulum.attachSignal(name='alpha_dot (rad)',  width=1, color=np.array([0, 0, 255]))
scopeBase.attachSignal(name='theta_dot (rad)',      width=1, color=np.array([0, 0, 255]))
#endregion

'''
이산시간 
ds_sys =
'''

# pre-designed controller 
# Design_obsv_LQR_20ms.m 실행 후 복붙
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

H = np.array([ 24.7585, -77.4332, 5.5625, -6.9830 ], dtype=np.float64)  # shape (4,)

# ---------- (추가) PRBS 생성기 ----------
class PRBS:
    """
    m-sequence PRBS (LFSR 기반).
    - order: LFSR 차수 (11 → 주기 2^11-1 = 2047 비트)
    - taps: XOR 탭 인덱스(1-based). (11,9)는 표준 원시다항식 중 하나.
    - amp: 출력 진폭 (±amp)
    - clock_samples: 각 비트가 유지되는 샘플 수 (샘플링 주파수 / PRBS clock 주파수)
    """
    def __init__(self, order=11, taps=(11, 9), seed=0x5A3, amp=0.5, clock_samples=10):
        assert order <= 31, "order too large"
        assert seed & ((1 << order) - 1) != 0, "seed must be nonzero"
        self.order = order
        self.taps = taps
        self.state = seed & ((1 << order) - 1)
        self.amp = float(amp)
        self.clock_samples = int(clock_samples)
        self._hold = 0
        self._val = self._bit_to_level(self.state & 0x1)

    def _step_bit(self):
        # LFSR 한 스텝 (Galois/XOR 피드백)
        fb = 0
        for t in self.taps:
            fb ^= (self.state >> (t - 1)) & 0x1
        out = self.state & 0x1
        self.state = (self.state >> 1) | (fb << (self.order - 1))
        return out

    @staticmethod
    def _bit_to_level(bit):
        # 0 -> -1, 1 -> +1
        return 1.0 if bit else -1.0

    def next(self):
        # clock_samples 동안 값 유지, 이후 비트 갱신
        if self._hold <= 0:
            b = self._step_bit()
            self._val = self._bit_to_level(b)
            self._hold = self.clock_samples
        self._hold -= 1
        return self.amp * self._val
# --------------------------------------

# Code to control the Qube Hardware
def control_loop():

    # Qube 설정
    qubeVersion = 3
    hardware = 1
    pendulum = 1

    # 샘플링타임
    frequency = 20  # [Hz] (20ms)

    state_theta_dot = np.array([0,0], dtype=np.float64)
    state_alpha_dot = np.array([0,0], dtype=np.float64)

    # Scope 샘플 제한
    countMax = frequency / 50
    count = 0

    if qubeVersion == 2:
        QubeClass = QubeServo2
        K = np.array([-1, 34.75, -1.495, 3.111]) # 사용 x
    else:
        QubeClass = QubeServo3
        K =np.array([ 8.3463, -47.2652, 2.5817, -4.1322 ], dtype=np.float64) 

    # 옵저버 상태
    xhat = np.zeros(4, dtype=np.float64)

    # ---------- (추가) PRBS 인젝터 준비 ----------
    PRBS_CLOCK_HZ = 20                 # 비트 전환 주파수(권장: 10~30Hz)
    clock_samples = int(frequency / PRBS_CLOCK_HZ)
    PE_AMP = 0.5                        # [V] 자극 진폭 (상황 보며 0.3~0.8로 조정)
    prbs = PRBS(order=11, taps=(11,9), seed=0x5A3, amp=PE_AMP, clock_samples=clock_samples)
    # ---------------------------------------------

    # 로깅 버퍼
    log_rows = []
    log_header = [
        "timeStamp_s",
        "u_raw_V",
        "u_prbs_V",
        "u_noisy_V",   # (u_raw + PRBS)
        "voltage_V",
        "theta_rad",
        "alpha_rad",
        "theta_dot_rad_s",
        "alpha_dot_rad_s"
    ]

    with QubeClass(hardware=hardware, pendulum=pendulum, frequency=frequency) as myQube:

        startTime = 0
        timeStamp = 0
        def elapsed_time():
            return time.time() - startTime
        startTime = time.time()

        while timeStamp < simulationTime and not KILL_THREAD:
            loopTime = time.time()
            # 센서 읽기
            myQube.read_outputs()

            theta = myQube.motorPosition * -1
            alpha_f =  myQube.pendulumPosition
            
            # 출력 전처리
            alpha = math.atan2(math.sin(alpha_f - math.pi), math.cos(alpha_f - math.pi))
            alpha_degrees = abs(math.degrees(alpha))
            y = np.array([theta, alpha], dtype=np.float64)

            # 미분 추정
            theta_dot, state_theta_dot = ddt_filter(theta, state_theta_dot, 50, 1/frequency)
            alpha_dot, state_alpha_dot = ddt_filter(alpha, state_alpha_dot, 100, 1/frequency)
            x_meas = np.array([theta, alpha, theta_dot, alpha_dot], dtype=np.float64)

            # 레퍼런스(사용 안함)
            command_deg = 0
            states = command_deg*np.array([np.pi/180, 0, 0, 0]) - np.array([theta, alpha, theta_dot, alpha_dot])
            
            # ------- 제어 입력 계산 -------
            u_raw = -1.0 * float(H @ x_meas)    # 측정값 기반 full state feedback
            u_pe  = float(prbs.next())          # (추가) PRBS 자극 (PE 보장용)
            # u_noisy = u_raw + u_pe              # 포화 전
            u_noisy = u_raw              # 포화 전
            # -----------------------------

            VMAX = 10.0  # 포화

            # 각도 게이팅(안전)
            if alpha_degrees > 40:
                voltage = 0.0
            else:
                voltage = float(np.clip(u_noisy, -VMAX, VMAX))

            # 장치에 전압 인가
            myQube.write_voltage(voltage)
            
            # 옵저버 업데이트(게이팅)
            if alpha_degrees > 40:
                xhat = np.array([0, 0, 0, 0], dtype=np.float64)
                x_meas = np.array([0, 0, 0, 0], dtype=np.float64)
            else:
                xhat = F @ xhat + G @ y

            # 로깅
            current_ts = timeStamp
            log_rows.append([
                float(current_ts),
                float(u_raw),
                float(u_pe),
                float(u_noisy),
                float(voltage),
                float(x_meas[0]),
                float(x_meas[1]),
                float(x_meas[2]),
                float(x_meas[3]),
            ])

            # 스코프 갱신
            count += 1
            if count >= countMax:
                scopeVoltage.sample(timeStamp, [voltage])
                scopeError.sample(timeStamp, [alpha_degrees])
                scopePendulum.sample(timeStamp, [x_meas[1], xhat[3]])
                scopeBase.sample(timeStamp, [x_meas[0], xhat[2]])
                count = 0

            loopTime_end = time.time()
            timeStamp = elapsed_time()

    # CSV 저장
    try:
        with open("qube_log_data.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(log_header)
            writer.writerows(log_rows)
        print(f"[LOG] Saved {len(log_rows)} rows to qube_log_data.csv")
    except Exception as e:
        print(f"[LOG][ERROR] Failed to save CSV: {e}")

# 스레드 실행
thread = Thread(target=control_loop)
thread.start()

while thread.is_alive() and (not KILL_THREAD):
    Scope.refreshAll()
    time.sleep(0.01)

input('Press the enter key to exit.')
