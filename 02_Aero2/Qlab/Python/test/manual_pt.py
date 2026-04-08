## manual_pt.py
# Aero2 Plant side — TCP plaintext controller
#
# 통신 흐름 (매 스텝):
#   1. recv u        (Controller → Plant)  2 floats
#   2. read sensor   (hardware tick @ 50Hz)
#   3. apply u       (write_voltage)
#   4. compute r     (square-wave schedule)
#   5. send (u, y, r)(Plant → Controller)  6 floats
# -----------------------------------------------------------------------

import sys
import os
import time
import struct
import socket
import signal
import numpy as np
from threading import Thread

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', '..', '..', '00_libraries', 'python'))
from pal.products.aero2 import Aero2
from pal.utilities.scope import MultiScope

# ── Kill flag ───────────────────────────────────────────────────────────
KILL_THREAD = False
def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)

# ── Run parameters ──────────────────────────────────────────────────────
RUN_TIME        = 50.0
FREQUENCY       = 50
Ts              = 1.0 / FREQUENCY
VMAX            = 15.0
PITCH_LIMIT     = np.deg2rad(40)

# ── Reference (must match controller.go) ────────────────────────────────
REF_PITCH_DEG   =  10.0
REF_YAW_DEG     = -10.0
REF_HALF_PERIOD = 10.0   # seconds per half-cycle
ref_base = np.array([np.deg2rad(REF_PITCH_DEG), np.deg2rad(REF_YAW_DEG)])

# ── TCP helpers ─────────────────────────────────────────────────────────
def recv_exact(sock, n):
    data = b''
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data += packet
    return data

def send_floats(sock, values):
    data = struct.pack(f'<{len(values)}d', *values)
    sock.sendall(struct.pack('<I', len(data)) + data)

def recv_floats(sock, count):
    raw_len = recv_exact(sock, 4)
    if not raw_len:
        return None
    length = struct.unpack('<I', raw_len)[0]
    data = recv_exact(sock, length)
    if not data:
        return None
    return list(struct.unpack(f'<{count}d', data))

# ── Scope ───────────────────────────────────────────────────────────────
scope = MultiScope(rows=3, cols=2, title='Aero2 TCP Plaintext Controller', fps=30)

scope.addAxis(row=0, col=0, timeWindow=30, yLabel='Pitch (deg)')
scope.axes[0].attachSignal(name='theta_p')
scope.axes[0].attachSignal(name='ref_p')

scope.addAxis(row=0, col=1, timeWindow=30, yLabel='Yaw (deg)')
scope.axes[1].attachSignal(name='theta_y')
scope.axes[1].attachSignal(name='ref_y')

scope.addAxis(row=1, col=0, timeWindow=30, yLabel='Pitch Rate (deg/s)')
scope.axes[2].attachSignal(name='dp')

scope.addAxis(row=1, col=1, timeWindow=30, yLabel='Yaw Rate (deg/s)')
scope.axes[3].attachSignal(name='dy')

scope.addAxis(row=2, col=0, timeWindow=30, xLabel='Time (s)', yLabel='V_main (V)')
scope.axes[4].attachSignal(name='V_main')

scope.addAxis(row=2, col=1, timeWindow=30, xLabel='Time (s)', yLabel='V_tail (V)')
scope.axes[5].attachSignal(name='V_tail')


# ── Control loop ────────────────────────────────────────────────────────
def control_loop():
    HOST = '127.0.0.1'
    PORT = 8000

    myAero2   = Aero2(id=0, hardware=0, readMode=0, frequency=FREQUENCY)
    startTime = time.time()
    timestamp = 0.0

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

            # connect to controller (server)
            while not KILL_THREAD:
                try:
                    sock.connect((HOST, PORT))
                    print(">> Connected to Controller!")
                    break
                except ConnectionRefusedError:
                    time.sleep(1)

            while timestamp < RUN_TIME and not KILL_THREAD:

                # ── 1. recv u from controller (2 floats) ──────────────────
                u_vals = recv_floats(sock, 2)
                if u_vals is None:
                    break

                # ── 2. read sensors (blocks at 50 Hz hardware tick) ───────
                myAero2.read_analog_encoder_other_channels()
                theta_p = myAero2.pitchAngle
                theta_y = myAero2.yawAngle
                dp_meas = myAero2.pitchRate
                dy_meas = myAero2.yawRate
                y = np.array([theta_p, theta_y])

                # ── 3. safety check & apply u ──────────────────────────────
                V_main = float(np.clip(u_vals[0], -VMAX, VMAX))
                V_tail = float(np.clip(u_vals[1], -VMAX, VMAX))

                if abs(theta_p) > PITCH_LIMIT:
                    V_main, V_tail = 0.0, 0.0
                    print(f"  [SAFETY] pitch={np.rad2deg(theta_p):.1f}° — motor OFF")

                myAero2.write_voltage(V_main, V_tail)
                u_applied = np.array([V_main, V_tail])

                # ── 4. square-wave reference ───────────────────────────────
                ref_sign = 1 - 2 * (int(timestamp / REF_HALF_PERIOD) % 2)
                r = ref_sign * ref_base

                # ── 5. send (u_applied, y, r) = 6 floats to controller ────
                send_floats(sock, list(u_applied) + list(y) + list(r))

                # ── 6. scope ──────────────────────────────────────────────
                scope.axes[0].sample(timestamp, [np.rad2deg(theta_p), np.rad2deg(r[0])])
                scope.axes[1].sample(timestamp, [np.rad2deg(theta_y), np.rad2deg(r[1])])
                scope.axes[2].sample(timestamp, [np.rad2deg(dp_meas)])
                scope.axes[3].sample(timestamp, [np.rad2deg(dy_meas)])
                scope.axes[4].sample(timestamp, [V_main])
                scope.axes[5].sample(timestamp, [V_tail])

                timestamp = time.time() - startTime

    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        myAero2.write_voltage(0.0, 0.0)
        myAero2.terminate()
        print("Aero2 terminated.")
        input("Press ENTER to exit.")


# ── Main ────────────────────────────────────────────────────────────────
thread = Thread(target=control_loop)
thread.start()

while thread.is_alive() and not KILL_THREAD:
    MultiScope.refreshAll()
    time.sleep(0.01)
