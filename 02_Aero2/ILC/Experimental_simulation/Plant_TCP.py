# TCP.py — Aero2 Hardware Client (communicates with EncILC.go server on :5555)
#
# Handles real-time hardware I/O for the Quanser Aero2 2-DOF helicopter.
# All encrypted control computation runs in EncILC.go.
#
# Startup order:
#   1) go run EncILC.go      (first terminal — takes ~30s for offline setup)
#   2) python TCP.py         (second terminal — connects automatically)
#
# Per-step binary protocol (little-endian):
#   Python → Go : [0x01][y0 y1 r0 r1 : 4×float64]  = 33 bytes
#   Go → Python : [u0 u1 : 2×float64]               = 16 bytes
# End-of-trial:
#   Python → Go : [0x02]                             =  1 byte
#   Go → Python : [rmsP_rad rmsY_rad : 2×float64]   = 16 bytes
#
# Data is saved to data/data_k.npz (compatible with result.py).

import sys, os, glob, time, signal, traceback, socket, struct
import numpy as np
from threading import Thread

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', '..', '00_libraries', 'python'))

from pal.products.aero2 import Aero2
from pal.utilities.scope import MultiScope

# ── Kill flag ─────────────────────────────────────────────────────────────
KILL_THREAD = False

def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True

signal.signal(signal.SIGINT, sig_handler)

# ── Constants ─────────────────────────────────────────────────────────────
FREQUENCY     = 20
Ts            = 1.0 / FREQUENCY
TRAJ_TIME     = 10.0
RUN_TIME      = 11.0
N_STEPS       = int(TRAJ_TIME * FREQUENCY)    # 200 steps (active control)
N_TOTAL_STEPS = int(RUN_TIME * FREQUENCY)     # 220 steps (incl. cool-down)
VMAX          = 20.0
PITCH_LIMIT   = np.deg2rad(40)
TCP_HOST      = 'localhost'
TCP_PORT      = 5555
# Socket timeout per step: must be shorter than Ts to avoid runaway timing.
# Go's encrypted step should complete in <40ms on typical hardware.
RECV_TIMEOUT  = 0.040  # 40ms

# ── Reference trajectory (smooth rise / hold / return / hold) ────────────
# Rise 0->target and return target->0 both use a "smootherstep" quintic
# ease (6*tau^5 - 15*tau^4 + 10*tau^3), which has zero 1st AND 2nd
# derivative at tau=0 and tau=1. That makes velocity continuous across
# every segment boundary (hold segments already have zero velocity), and
# pitch/yaw start and end each trial at exactly (0, 0) with zero rate —
# required so repeated ILC iterations share identical initial/final
# conditions.
PIT_AMP_DEG   = 10.0
YAW_MAX_DEG   = 30.0
T_RISE_END    = 3.0   # 0s -> 3.0s   : rise to target attitude
T_HOLD1_END   = 5.0   # 3.0s -> 5.0s : hold target attitude
T_RETURN_END  = 8.0   # 5.0s -> 8.0s : return to zero
# 8.0s -> TRAJ_TIME (10.0s)          : hold zero attitude

def _smootherstep(tau):
    tau = np.clip(tau, 0.0, 1.0)
    return 6 * tau**5 - 15 * tau**4 + 10 * tau**3

def compute_ref(step_idx):
    t = step_idx * Ts
    if t >= TRAJ_TIME:
        return np.array([0.0, 0.0])
    if t < T_RISE_END:
        s = _smootherstep(t / T_RISE_END)
    elif t < T_HOLD1_END:
        s = 1.0
    elif t < T_RETURN_END:
        s = 1.0 - _smootherstep((t - T_HOLD1_END) / (T_RETURN_END - T_HOLD1_END))
    else:
        s = 0.0
    pitch = np.deg2rad(PIT_AMP_DEG) * s
    yaw = np.deg2rad(YAW_MAX_DEG) * s
    return np.array([pitch, yaw])

# ── Data setup ────────────────────────────────────────────────────────────
DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')
os.makedirs(DATA_DIR, exist_ok=True)
existing = sorted(glob.glob(os.path.join(DATA_DIR, 'data_*.npz')),
                  key=lambda f: int(os.path.splitext(os.path.basename(f))[0].split('_')[1]))
k = len(existing) + 1
OUT_FILE = os.path.join(DATA_DIR, f'data_{k}.npz')
R_seq = np.array([compute_ref(i) for i in range(N_STEPS)])

print(f"\n=== TCP.py — Aero2 Encrypted ILC Client  (k={k}) ===")

# ── TCP helpers ───────────────────────────────────────────────────────────
def recv_exact(sock, n):
    """Receive exactly n bytes, raising ConnectionError if the socket closes."""
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("EncILC server closed connection")
        buf.extend(chunk)
    return bytes(buf)


def connect_to_server(max_wait_s=120):
    """Retry connecting until the Go server is ready (it takes ~30s to start)."""
    deadline = time.time() + max_wait_s
    attempt = 0
    while time.time() < deadline:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.connect((TCP_HOST, TCP_PORT))
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f"[TCP] Connected to EncILC server at {TCP_HOST}:{TCP_PORT}")
            return sock
        except ConnectionRefusedError:
            if attempt == 0:
                print(f"[TCP] Waiting for EncILC server on port {TCP_PORT} "
                      f"(up to {max_wait_s}s) ...")
            attempt += 1
            time.sleep(1.0)
    raise RuntimeError(f"Could not connect to EncILC server within {max_wait_s}s")

# ── Scope setup ───────────────────────────────────────────────────────────
scope = MultiScope(rows=3, cols=2, title=f'Aero2 Encrypted ILC (k={k})', fps=30)

scope.addAxis(row=0, col=0, timeWindow=RUN_TIME, yLabel='Pitch (deg)')
scope.axes[0].attachSignal(name='theta_p (meas)')
scope.axes[0].attachSignal(name='ref_p')

scope.addAxis(row=0, col=1, timeWindow=RUN_TIME, yLabel='Yaw (deg)')
scope.axes[1].attachSignal(name='theta_y (meas)')
scope.axes[1].attachSignal(name='ref_y')

scope.addAxis(row=1, col=0, timeWindow=RUN_TIME, yLabel='V_main (V)')
scope.axes[2].attachSignal(name='V_main (total)')

scope.addAxis(row=1, col=1, timeWindow=RUN_TIME, yLabel='V_tail (V)')
scope.axes[3].attachSignal(name='V_tail (total)')

scope.addAxis(row=2, col=0, timeWindow=RUN_TIME, xLabel='Time (s)', yLabel='Pitch Error (deg)')
scope.axes[4].attachSignal(name='e_p')

scope.addAxis(row=2, col=1, timeWindow=RUN_TIME, xLabel='Time (s)', yLabel='Yaw Error (deg)')
scope.axes[5].attachSignal(name='e_y')

# ── Logging buffers ───────────────────────────────────────────────────────
U_log = np.zeros((N_STEPS, 2))
Y_log = np.zeros((N_STEPS, 2))

# ── Control loop (runs in a thread) ──────────────────────────────────────
def control_loop():
    global KILL_THREAD

    try:
        sock = connect_to_server()
    except RuntimeError as e:
        print(f"[ERROR] {e}")
        return

    myAero2 = Aero2(id=0, hardware=1, readMode=0, frequency=FREQUENCY)
    step = 0
    u_cmd = np.zeros(2)  # last known control (fallback on timeout)

    try:
        while step < N_TOTAL_STEPS and not KILL_THREAD:
            loop_start = time.perf_counter()
            timestamp = step * Ts

            # 1. Read sensors
            myAero2.read_analog_encoder_other_channels()
            theta_p = myAero2.pitchAngle
            theta_y = myAero2.yawAngle
            dp_meas = myAero2.pitchRate
            dy_meas = myAero2.yawRate
            y = np.array([theta_p, theta_y])

            if step < N_STEPS:
                ref_now = compute_ref(step)

                # 2. Send (y, r) to EncILC server
                msg = struct.pack('<Bdddd', 0x01, y[0], y[1], ref_now[0], ref_now[1])
                sock.sendall(msg)

                # 3. Receive u from EncILC server (with timeout for safety)
                try:
                    sock.settimeout(RECV_TIMEOUT)
                    u_bytes = recv_exact(sock, 16)
                    sock.settimeout(None)
                    u0, u1 = struct.unpack('<dd', u_bytes)
                    u_cmd = np.array([u0, u1])
                except socket.timeout:
                    print(f"[WARN] step {step}: encrypted control timeout — using previous u")
                    sock.settimeout(None)
                    # u_cmd retains its last value

                U_log[step] = u_cmd
                Y_log[step] = y
            else:
                # Cool-down: no encrypted control, let hardware spin down
                ref_now = np.zeros(2)
                u_cmd = np.zeros(2)

            e_now = ref_now - y

            # 4. Clip + pitch safety
            V_main = float(np.clip(u_cmd[0], -VMAX, VMAX))
            V_tail = float(np.clip(u_cmd[1], -VMAX, VMAX))
            if abs(y[0]) > PITCH_LIMIT:
                print(f"[SAFETY] Pitch limit at step {step}: {np.rad2deg(y[0]):.1f}°")
                V_main, V_tail = 0.0, 0.0

            # 5. Write to hardware
            myAero2.write_voltage(V_main, V_tail)

            # 6. Scope
            scope.axes[0].sample(timestamp, [np.rad2deg(theta_p), np.rad2deg(ref_now[0])])
            scope.axes[1].sample(timestamp, [np.rad2deg(theta_y), np.rad2deg(ref_now[1])])
            scope.axes[2].sample(timestamp, [V_main])
            scope.axes[3].sample(timestamp, [V_tail])
            scope.axes[4].sample(timestamp, [np.rad2deg(e_now[0])])
            scope.axes[5].sample(timestamp, [np.rad2deg(e_now[1])])

            step += 1
            elapsed = time.perf_counter() - loop_start
            if Ts - elapsed > 0:
                time.sleep(Ts - elapsed)

    except Exception:
        print("\n[ERROR] Control loop exception:")
        traceback.print_exc()
    finally:
        myAero2.write_voltage(0.0, 0.0)
        myAero2.terminate()

        n_valid = min(step, N_STEPS)

        # Signal end-of-trial to Go server and receive ILC stats
        try:
            sock.settimeout(None)
            sock.sendall(struct.pack('<B', 0x02))  # END
            stats_bytes = recv_exact(sock, 16)
            rmsP_rad, rmsY_rad = struct.unpack('<dd', stats_bytes)
            print(f"\n[ILC] Server updated feedforward.")
            print(f"      Pitch RMS = {np.rad2deg(rmsP_rad):.3f}°  "
                  f"Yaw RMS = {np.rad2deg(rmsY_rad):.3f}°")
        except Exception as e:
            print(f"[TCP] Could not receive ILC stats from server: {e}")
        finally:
            sock.close()

        # Save data (compatible with result.py)
        U_final = U_log[:n_valid]
        Y_final = Y_log[:n_valid]
        R_final = R_seq[:n_valid]
        E_final = R_final - Y_final
        np.savez(OUT_FILE,
                 U=U_final, Y=Y_final, R=R_final, E=E_final,
                 k=np.int32(k), n_valid=np.int32(n_valid), Ts=Ts)
        print(f"[DATA] Saved → {OUT_FILE}")

        # Print tracking error summary
        if n_valid > 0:
            epc = np.rad2deg(np.sqrt(np.mean(E_final[:, 0] ** 2)))
            eyc = np.rad2deg(np.sqrt(np.mean(E_final[:, 1] ** 2)))
            etc = np.sqrt(epc ** 2 + eyc ** 2)

            # Compare with previous iteration
            prev_line = ""
            if k > 1 and existing:
                prev = np.load(existing[-1])
                nv = int(prev['n_valid'])
                Ec_prev = R_seq[:nv] - prev['Y'][:nv]
                ep_prev = np.rad2deg(np.sqrt(np.mean(Ec_prev[:, 0] ** 2)))
                ey_prev = np.rad2deg(np.sqrt(np.mean(Ec_prev[:, 1] ** 2)))
                et_prev = np.sqrt(ep_prev ** 2 + ey_prev ** 2)
                sym = lambda d: "▼" if d < 0 else "▲"
                prev_line = (
                    f"  │  [vs k={k-1}]  "
                    f"Pitch {ep_prev:.3f}°→{epc:.3f}° {sym(epc-ep_prev)}  "
                    f"Yaw {ey_prev:.3f}°→{eyc:.3f}° {sym(eyc-ey_prev)}"
                )

            print(f"\n  ┌─ Tracking Error Summary (k={k}) ────────")
            print(f"  │  Pitch RMS : {epc:6.3f} °")
            print(f"  │  Yaw   RMS : {eyc:6.3f} °")
            print(f"  │  Total RMS : {etc:6.3f} °")
            if prev_line:
                print(prev_line)
            print(f"  └─────────────────────────────────────────\n")

# ── Main ──────────────────────────────────────────────────────────────────
thread = Thread(target=control_loop)
thread.start()

try:
    while thread.is_alive() and not KILL_THREAD:
        MultiScope.refreshAll()
        time.sleep(0.01)
except KeyboardInterrupt:
    KILL_THREAD = True

thread.join()
