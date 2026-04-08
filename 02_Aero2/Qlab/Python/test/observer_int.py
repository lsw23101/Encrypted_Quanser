## observer_int.py
# Aero2 2-DOF: Integer-Transformed Observer-Based LQR Tracking
#
# 제어 구조 (CDSL 정수행렬 변환):
#   u(k)      = H_ * z(k)  + N_bar * r
#   z(k+1)    = F_ * z(k)  + G_ * y(k) + R_ * u(k) + P_ * r
#
#   F_ : 정수행렬 (암호화 제어 적용 가능)
#   z  = T * xhat  (변환 좌표 옵저버 상태)
#
# State:  x = [theta_p, theta_y, dp, dy]'
# Output: y = [theta_p, theta_y]'
# Input:  u = [V_main, V_tail]'
# -----------------------------------------------------------------------

import sys
import os
import time
import signal
import numpy as np
from threading import Thread

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
RUN_TIME    = 50.0
FREQUENCY   = 50
Ts          = 1.0 / FREQUENCY
VMAX        = 15.0
PITCH_LIMIT = np.deg2rad(40)

# ── Square-wave Reference ──────────────────────────────────────────────
REF_PITCH_DEG   =  30.0
REF_YAW_DEG     = -30.0
REF_HALF_PERIOD = 10.0   # seconds per half-cycle (sign flip interval)
ref_base = np.array([np.deg2rad(REF_PITCH_DEG),
                     np.deg2rad(REF_YAW_DEG)])

# ── Integer-Transformed Matrices (MATLAB conversion.m 결과) ────────────
# u(k)   = H_ * z + N_bar * r
# z(k+1) = F_ * z + G_ * y + R_ * u + P_ * r

F_ = np.array([
    [ -0, -0, 1, 0 ],
    [ -0, 0, 0, 1 ],
    [ 0, 0, 0, -0 ],
    [ 0, 0, -0, -0 ]
], dtype=np.float64)

G_ = np.array([
    [ 2.2322, 0.3405 ],
    [ 0.2195, 3.1594 ],
    [ -2.1659, -0.3193 ],
    [ -0.2024, -3.0236 ]
], dtype=np.float64)

R_ = np.array([
    [ -0.0165, -0.0221 ],
    [ 0.0208, -0.0329 ],
    [ 0.0063, 0.0083 ],
    [ -0.0076, 0.0120 ]
], dtype=np.float64)

H_ = np.array([
    [ -48.5698, 31.2053, -0.0000, 0.0000 ],
    [ -30.3801, -22.3527, -0.0000, 0.0000 ]
], dtype=np.float64)

P_ = np.array([
    [ 2.3631, 0.5760 ],
    [ 0.4553, 3.4218 ],
    [ -0.9154, -0.2199 ],
    [ -0.1737, -1.2919 ]
], dtype=np.float64)

N_bar = np.array([
    [ 64.7566, -52.3833 ],
    [ 53.0017, 62.1050 ]
], dtype=np.float64)


print("=== Integer-Transformed Observer Controller ready ===")
print(f"  ref base: pitch=±{REF_PITCH_DEG:.1f}°, yaw=∓{REF_YAW_DEG:.1f}° (square wave, period=10s)")

# ── Scope ──────────────────────────────────────────────────────────────
scope = MultiScope(rows=3, cols=2, title='Aero2 Integer-Transformed Observer', fps=30)

scope.addAxis(row=0, col=0, timeWindow=30, yLabel='Pitch (deg)')
scope.axes[0].attachSignal(name='theta_p (meas)')
scope.axes[0].attachSignal(name='ref_p')

scope.addAxis(row=0, col=1, timeWindow=30, yLabel='Yaw (deg)')
scope.axes[1].attachSignal(name='theta_y (meas)')
scope.axes[1].attachSignal(name='ref_y')

scope.addAxis(row=1, col=0, timeWindow=30, yLabel='Pitch Rate (deg/s)')
scope.axes[2].attachSignal(name='dp (meas)')

scope.addAxis(row=1, col=1, timeWindow=30, yLabel='Yaw Rate (deg/s)')
scope.axes[3].attachSignal(name='dy (meas)')

scope.addAxis(row=2, col=0, timeWindow=30, xLabel='Time (s)', yLabel='V_main (V)')
scope.axes[4].attachSignal(name='V_main')

scope.addAxis(row=2, col=1, timeWindow=30, xLabel='Time (s)', yLabel='V_tail (V)')
scope.axes[5].attachSignal(name='V_tail')


# ── Control loop ───────────────────────────────────────────────────────
def control_loop():
    myAero2   = Aero2(id=0, hardware=0, readMode=0, frequency=FREQUENCY)
    startTime = time.time()
    timestamp = 0.0
    z         = np.zeros(4, dtype=np.float64)   # 변환 좌표 옵저버 상태

    try:
        while timestamp < RUN_TIME and not KILL_THREAD:
            loop_start = time.perf_counter()

            # ── 1. 센서 읽기 ──────────────────────────────────────────────
            myAero2.read_analog_encoder_other_channels()
            theta_p = myAero2.pitchAngle
            theta_y = myAero2.yawAngle
            dp_meas = myAero2.pitchRate
            dy_meas = myAero2.yawRate

            y = np.array([theta_p, theta_y], dtype=np.float64)

            # ── 2. 사각파 레퍼런스 ───────────────────────────────────────
            ref_sign = 1 - 2 * (int(timestamp / REF_HALF_PERIOD) % 2)
            ref_now  = ref_sign * ref_base

            # ── 3. 제어 입력: u = H_*z + N_bar*r ────────────────────────
            u_ctrl = H_ @ z + N_bar @ ref_now
            V_main = float(np.clip(u_ctrl[0], -VMAX, VMAX))
            V_tail = float(np.clip(u_ctrl[1], -VMAX, VMAX))
            u_vec  = np.array([V_main, V_tail])

            # ── 4. 안전 장치 ─────────────────────────────────────────────
            if abs(theta_p) > PITCH_LIMIT:
                V_main, V_tail = 0.0, 0.0
                u_vec[:] = 0.0
                print(f"  [SAFETY] pitch={np.rad2deg(theta_p):.1f}° — 모터 OFF")

            # ── 5. 출력 ──────────────────────────────────────────────────
            myAero2.write_voltage(V_main, V_tail)

            # ── 6. Scope ─────────────────────────────────────────────────
            scope.axes[0].sample(timestamp, [np.rad2deg(theta_p), np.rad2deg(ref_now[0])])
            scope.axes[1].sample(timestamp, [np.rad2deg(theta_y), np.rad2deg(ref_now[1])])
            scope.axes[2].sample(timestamp, [np.rad2deg(dp_meas)])
            scope.axes[3].sample(timestamp, [np.rad2deg(dy_meas)])
            scope.axes[4].sample(timestamp, [V_main])
            scope.axes[5].sample(timestamp, [V_tail])

            # ── 7. 옵저버 State Update (정수행렬) ────────────────────────
            # z(k+1) = F_*z + G_*y + R_*u + P_*r
            z = F_ @ z + G_ @ y + R_ @ u_vec + P_ @ ref_now

            # ── 8. Timing ────────────────────────────────────────────────
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
