## fullstate.py
# Aero2 2-DOF: LQR Tracking Control (R. Fellag et al., 2024)
#
# MATLAB test.m 과 동일한 제어 구조:
#   u(k) = -K * x_hat(k) + N_bar * ref
#   x_hat(k+1) = Ad*x_hat(k) + Bd*u(k) + L*(y(k) - Cd*x_hat(k))
#
# State:  x = [theta_p, theta_y, dp, dy]'
# Input:  u = [V_main, V_tail]'
# Ref:    ref = [REF_PITCH, REF_YAW]' (rad)
# -----------------------------------------------------------------------

import sys
import os
import time
import signal
import numpy as np
from threading import Thread
from scipy.linalg import expm

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', '..', '..', '00_libraries', 'python'))

from pal.products.aero2 import Aero2
from pal.utilities.scope import MultiScope

# ── Thread kill flag ───────────────────────────────────────────────────
global KILL_THREAD
KILL_THREAD = False
def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)

# ── Run parameters ─────────────────────────────────────────────────────
RUN_TIME    = 20.0        # s  (0~10s: ref, 10~20s: -ref)
FREQUENCY   = 50          # Hz
Ts          = 1.0 / FREQUENCY
VMAX        = 10.0        # V
PITCH_LIMIT = np.deg2rad(40)

# ── Step Reference (MATLAB: pitch=10deg, yaw=-10deg) ───────────────────
REF_PITCH_DEG =  30.0    # deg
REF_YAW_DEG   = -30.0    # deg
ref = np.array([np.deg2rad(REF_PITCH_DEG),
                np.deg2rad(REF_YAW_DEG)])

# ── System parameters (Fellag 2024, Table I) ───────────────────────────
Jp = 0.0211;  Jy = 0.0221
Dp = 0.0053;  Dy = 0.0062
Mg = 0.0153
Kpp =  0.0011;  Kyy =  0.0047
Kpy =  0.0021;  Kyp = -0.0027

A = np.array([
    [0,       0,   1,       0     ],
    [0,       0,   0,       1     ],
    [-Mg/Jp,  0,  -Dp/Jp,   0     ],
    [0,       0,   0,      -Dy/Jy ],
], dtype=np.float64)

B = np.array([
    [0,        0       ],
    [0,        0       ],
    [Kpp/Jp,   Kpy/Jp  ],
    [Kyp/Jy,   Kyy/Jy  ],
], dtype=np.float64)

Cd = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
], dtype=np.float64)

# ZOH 이산화
M = np.zeros((6, 6))
M[:4, :4] = A * Ts
M[:4, 4:] = B * Ts
Me = expm(M)
Ad = Me[:4, :4]
Bd = Me[:4, 4:]

# ── LQR Gain K (MATLAB dlqr 결과) ──────────────────────────────────────
K = np.array([
    [ 84.0016, -73.3503,  38.3322, -23.8340],
    [ 70.3379,  86.4152,  25.8040,  21.2513],
], dtype=np.float64)

# ── Feedforward N_bar (MATLAB 결과) ────────────────────────────────────
N_bar = np.array([
    [90.6353, -73.3503],
    [74.1488,  86.4152],
], dtype=np.float64)

print("=== Controller ready ===")
print(f"  ref: pitch={REF_PITCH_DEG:.1f}°, yaw={REF_YAW_DEG:.1f}°")

# ── Scope ──────────────────────────────────────────────────────────────
scope = MultiScope(rows=3, cols=2, title='Aero2 LQR Tracking (Fellag 2024)', fps=30)

scope.addAxis(row=0, col=0, timeWindow=12, yLabel='Pitch (deg)')
scope.axes[0].attachSignal(name='theta_p')
scope.axes[0].attachSignal(name='ref_p')

scope.addAxis(row=0, col=1, timeWindow=12, yLabel='Yaw (deg)')
scope.axes[1].attachSignal(name='theta_y')
scope.axes[1].attachSignal(name='ref_y')

scope.addAxis(row=1, col=0, timeWindow=12, yLabel='Pitch Rate (deg/s)')
scope.axes[2].attachSignal(name='dp')

scope.addAxis(row=1, col=1, timeWindow=12, yLabel='Yaw Rate (deg/s)')
scope.axes[3].attachSignal(name='dy')

scope.addAxis(row=2, col=0, timeWindow=12, xLabel='Time (s)', yLabel='V_main (V)')
scope.axes[4].attachSignal(name='V_main')

scope.addAxis(row=2, col=1, timeWindow=12, xLabel='Time (s)', yLabel='V_tail (V)')
scope.axes[5].attachSignal(name='V_tail')


# ── Control loop ───────────────────────────────────────────────────────
def control_loop():
    myAero2   = Aero2(id=0, hardware=1, readMode=0, frequency=FREQUENCY)
    startTime = time.time()
    timestamp = 0.0

    try:
        while timestamp < RUN_TIME and not KILL_THREAD:
            loop_start = time.perf_counter()

            # ── 1. 센서 읽기 (Full-state: 직접 측정값 사용) ─────────────
            myAero2.read_analog_encoder_other_channels()
            theta_p =  myAero2.pitchAngle    # Fellag 모델 부호 그대로
            theta_y =  myAero2.yawAngle
            dp      =  myAero2.pitchRate
            dy      =  myAero2.yawRate

            x = np.array([theta_p, theta_y, dp, dy])

            # ── 2. 레퍼런스 전환 (10s 기준) ──────────────────────────────
            ref_now = ref if timestamp < 10.0 else -ref

            # ── 3. 제어 입력: u = -K*x + N_bar*ref (full-state) ─────────
            u_ctrl = -K @ x + N_bar @ ref_now
            V_main = float(np.clip(u_ctrl[0], -VMAX, VMAX))
            V_tail = float(np.clip(u_ctrl[1], -VMAX, VMAX))

            # ── 4. 안전 장치 ─────────────────────────────────────────────
            if abs(theta_p) > PITCH_LIMIT:
                V_main, V_tail = 0.0, 0.0
                print(f"  [SAFETY] pitch={np.rad2deg(theta_p):.1f}° — 모터 OFF")

            # ── 5. 출력 ──────────────────────────────────────────────────
            myAero2.write_voltage(V_main, V_tail)

            # ── 6. Scope (각도를 deg로 표시) ─────────────────────────────
            scope.axes[0].sample(timestamp, [np.rad2deg(theta_p), np.rad2deg(ref_now[0])])
            scope.axes[1].sample(timestamp, [np.rad2deg(theta_y), np.rad2deg(ref_now[1])])
            scope.axes[2].sample(timestamp, [np.rad2deg(dp)])
            scope.axes[3].sample(timestamp, [np.rad2deg(dy)])
            scope.axes[4].sample(timestamp, [V_main])
            scope.axes[5].sample(timestamp, [V_tail])

            # ── 7. Timing ────────────────────────────────────────────────
            elapsed = time.perf_counter() - loop_start
            if Ts - elapsed > 0:
                time.sleep(Ts - elapsed)
            timestamp = time.time() - startTime

    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        myAero2.write_voltage(0.0, 0.0)
        myAero2.terminate()
        print("Aero2 terminated.")
        input("Press ENTER to exit.")


# ── Main ───────────────────────────────────────────────────────────────
thread = Thread(target=control_loop)
thread.start()

while thread.is_alive() and not KILL_THREAD:
    MultiScope.refreshAll()
    time.sleep(0.01)
