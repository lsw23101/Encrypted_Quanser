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
TRAJ_TIME     = 20.0
RUN_TIME      = 21.0
N_STEPS       = int(TRAJ_TIME * FREQUENCY)    # 400 steps (active control)
N_TOTAL_STEPS = int(RUN_TIME * FREQUENCY)     # 420 steps (incl. cool-down)
VMAX          = 20.0
PITCH_LIMIT   = np.deg2rad(40)
TCP_HOST      = 'localhost'
TCP_PORT      = 5555
# Socket timeout per step: must be shorter than Ts to avoid runaway timing.
# Go's encrypted step should complete in <40ms on typical hardware.
RECV_TIMEOUT  = 0.040  # 40ms

# ── Reference trajectory ──────────────────────────────────────────────────
# Shape over the 20s trial:
#   Pitch: linear ramp 0 -> PIT_LEVELS[0] over PIT_RAMP_OUTER seconds, hold,
#          then a smootherstep staircase through PIT_LEVELS[1:] (each
#          transition a PIT_RAMP_INNER-second ramp followed by a PIT_HOLD
#          plateau), then a mirrored linear ramp PIT_LEVELS[-1] -> 0 over
#          the last PIT_RAMP_OUTER seconds — pitch starts and ends the
#          trial at exactly 0° (required for repeatable ILC iterations).
#   Yaw:   0 for the first/last YAW_MARGIN seconds, then one full sine
#          cycle (0 -> +YAW_MAX_DEG -> 0 -> -YAW_MAX_DEG -> 0) in between.
#          A plain sine has nonzero slope where it meets the flat margin,
#          so there's a small velocity kink there too.
PIT_LEVELS     = [10.0, 20.0, 15.0, 20.0, 10.0]  # deg — staircase plateaus
PIT_RAMP_OUTER = 2.5    # s — linear ramp to/from 0 at the very start/end
PIT_RAMP_INNER = 1.5    # s — smootherstep ramp between consecutive levels
PIT_HOLD       = 1.8    # s — hold duration at each plateau
YAW_MAX_DEG    = 90.0
YAW_MARGIN     = 1.0    # s of zero hold at the start and end

# 2*PIT_RAMP_OUTER + len(PIT_LEVELS)*PIT_HOLD + (len(PIT_LEVELS)-1)*PIT_RAMP_INNER
# must equal TRAJ_TIME; with the defaults above that's 5 + 9 + 6 = 20.
def _build_pitch_segments():
    segs = []
    t0 = 0.0
    t1 = t0 + PIT_RAMP_OUTER
    segs.append((t0, t1, 'linear', 0.0, PIT_LEVELS[0])); t0 = t1
    t1 = t0 + PIT_HOLD
    segs.append((t0, t1, 'hold', PIT_LEVELS[0], PIT_LEVELS[0])); t0 = t1
    for v0, v1 in zip(PIT_LEVELS, PIT_LEVELS[1:]):
        t1 = t0 + PIT_RAMP_INNER
        segs.append((t0, t1, 'smooth', v0, v1)); t0 = t1
        t1 = t0 + PIT_HOLD
        segs.append((t0, t1, 'hold', v1, v1)); t0 = t1
    t1 = t0 + PIT_RAMP_OUTER
    segs.append((t0, t1, 'linear', PIT_LEVELS[-1], 0.0)); t0 = t1
    return segs

_PIT_SEGMENTS = _build_pitch_segments()

def _smootherstep(tau):
    tau = np.clip(tau, 0.0, 1.0)
    return 6 * tau**5 - 15 * tau**4 + 10 * tau**3

def _pitch_deg(t):
    for t0, t1, kind, v0, v1 in _PIT_SEGMENTS:
        if t < t1:
            if kind == 'hold':
                return v0
            frac = (t - t0) / (t1 - t0)
            if kind == 'linear':
                return v0 + (v1 - v0) * frac
            return v0 + (v1 - v0) * _smootherstep(frac)
    return 0.0

def _yaw_deg(t):
    if t < YAW_MARGIN or t > TRAJ_TIME - YAW_MARGIN:
        return 0.0
    tau = (t - YAW_MARGIN) / (TRAJ_TIME - 2 * YAW_MARGIN)
    return YAW_MAX_DEG * np.sin(2 * np.pi * tau)

def compute_ref(step_idx):
    t = step_idx * Ts
    if t >= TRAJ_TIME:
        return np.array([0.0, 0.0])
    return np.array([np.deg2rad(_pitch_deg(t)), np.deg2rad(_yaw_deg(t))])

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
