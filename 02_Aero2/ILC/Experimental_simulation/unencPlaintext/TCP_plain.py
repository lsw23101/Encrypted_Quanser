# TCP_plain.py — Aero2 Plaintext ILC (single-process, no encryption)
#
# Plaintext equivalent of EncILC.go + TCP.py.
# Same reference, same controller matrices, same ILC update — no enc/dec.
# Run: python TCP_plain.py
#
# Data saved to Plaintext/data/data_k.npz  (compatible with result_plain.py)

import sys, os, glob, time, signal, traceback
import numpy as np
from scipy.linalg import expm
from threading import Thread

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', '..', '..', '00_libraries', 'python'))

from pal.products.aero2 import Aero2
from pal.utilities.scope import MultiScope

# ── Kill flag ──────────────────────────────────────────────────────────────
KILL_THREAD = False

def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True

signal.signal(signal.SIGINT, sig_handler)

# ── Constants (identical to TCP.py) ───────────────────────────────────────
FREQUENCY     = 20
Ts            = 1.0 / FREQUENCY
TRAJ_TIME     = 10.0
RUN_TIME      = 11.0
N_STEPS       = int(TRAJ_TIME * FREQUENCY)
N_TOTAL_STEPS = int(RUN_TIME * FREQUENCY)
VMAX          = 20.0
PITCH_LIMIT   = np.deg2rad(40)

# ── Reference trajectory (identical to TCP.py) ────────────────────────────
YAW_MAX_DEG = 200.0
PIT_AMP_DEG = 15.0  # halved from original 30°

def compute_ref(step_idx):
    t = step_idx * Ts
    if t >= TRAJ_TIME:
        return np.array([0.0, 0.0])
    tau = t / TRAJ_TIME
    window = np.sin(np.pi * tau) ** 3
    dynamic_pitch = np.sin(2 * np.pi * 0.1 * t) - 0.5 * np.cos(2 * np.pi * 0.2 * t)
    pitch = np.deg2rad(PIT_AMP_DEG) * window * dynamic_pitch
    base_yaw = np.sin(np.pi * tau)
    yaw_perturbation = 0.15 * np.sin(2 * np.pi * 0.15 * t)
    yaw = np.deg2rad(YAW_MAX_DEG) * window * (base_yaw + yaw_perturbation)
    return np.array([pitch, yaw])

# ── Aero2 continuous-time model (identical to EncILC.go) ──────────────────
Jp, Jy = 0.0211, 0.0221
Dp, Dy = 0.0053, 0.0062
Mg     = 0.0153
Kpp, Kpy = 0.0011, 0.0021
Kyp, Kyy = -0.0027, 0.0047

Ac = np.array([
    [0,       0, 1,        0     ],
    [0,       0, 0,        1     ],
    [-Mg/Jp,  0, -Dp/Jp,   0     ],
    [0,       0, 0,        -Dy/Jy],
])
Bc = np.array([
    [0,        0       ],
    [0,        0       ],
    [Kpp/Jp,   Kpy/Jp  ],
    [Kyp/Jy,   Kyy/Jy  ],
])
C = np.array([[1, 0, 0, 0],
              [0, 1, 0, 0]])

# ZOH discretisation
n, mi = 4, 2
M = np.zeros((n + mi, n + mi))
M[:n, :n] = Ac * Ts
M[:n, n:] = Bc * Ts
eM = expm(M)
Ad = eM[:n, :n]
Bd = eM[:n, n:]

# ── Controller matrices (identical to EncILC.go) ───────────────────────────
F_ = np.array([
    [0, 0, 1, 0],
    [0, 0, 0, 1],
    [0, 0, 0, 0],
    [0, 0, 0, 0],
], dtype=float)
G_ = np.array([
    [ 2.0730,  0.4658],
    [ 0.0050,  3.6318],
    [-1.9701, -0.4153],
    [ 0.0002, -3.3238],
])
R_ = np.array([
    [-0.0298, -0.0269],
    [ 0.0168, -0.0248],
    [ 0.0110,  0.0100],
    [-0.0062,  0.0092],
])
H_ = np.array([
    [-28.5768,  24.8825, 0, 0],
    [-18.4344, -24.0057, 0, 0],
])
P_ = np.array([
    [ 4.1572,  1.3674],
    [ 0.0960,  7.7492],
    [-1.6107, -0.5217],
    [-0.0359, -2.9252],
])
Nbar = np.array([
    [ 75.0524, -102.0894],
    [ 50.9885,  139.7176],
])

m, p = 2, 2
nSteps = N_STEPS
pN = p * nSteps
mN = m * nSteps

# ── Compute L_opt offline (identical logic to EncILC.go) ──────────────────
print("[ILC] Computing L_opt...")

n_aug = n + len(F_)
A_aug = np.zeros((n_aug, n_aug))
B_aug = np.zeros((n_aug, m))
C_aug = np.zeros((p, n_aug))

A_aug[:n, :n]   = Ad
A_aug[:n, n:]   = Bd @ H_
A_aug[n:, :n]   = G_ @ C
A_aug[n:, n:]   = F_ + R_ @ H_
B_aug[:n, :]    = Bd
B_aug[n:, :]    = R_
C_aug[:, :p]    = C

# Markov parameters h[t] = C_aug @ A_aug^t @ B_aug
h = np.zeros((nSteps, p, m))
Apow = np.eye(n_aug)
for i in range(nSteps):
    h[i] = C_aug @ Apow @ B_aug
    Apow = A_aug @ Apow

# Block lower-triangular Toeplitz G_N
GN = np.zeros((pN, mN))
for t in range(1, nSteps):
    for s in range(t):
        GN[t*p:(t+1)*p, s*m:(s+1)*m] = h[t-1-s]

ILC_LAMBDA = 1e-3
GNT  = GN.T
Lopt = np.linalg.solve(GNT @ GN + ILC_LAMBDA * np.eye(mN), GNT)

Phi  = np.eye(pN) - GN @ Lopt
rho  = np.sqrt(np.linalg.norm(Phi.T @ Phi, ord=2))
print(f"[ILC] λ={ILC_LAMBDA}  ||I-G_N@L_opt||_2={rho:.6f}")

# ── ILC state (persistent across trials) ──────────────────────────────────
BASE_DIR      = os.path.dirname(os.path.abspath(__file__))
DATA_DIR      = os.path.join(BASE_DIR, 'data')
ILC_STATE_DIR = os.path.join(BASE_DIR, 'ilc_state')
os.makedirs(DATA_DIR,      exist_ok=True)
os.makedirs(ILC_STATE_DIR, exist_ok=True)

v_ilc = np.zeros((nSteps, m))
vPath = os.path.join(ILC_STATE_DIR, 'v_ilc.npy')
if os.path.exists(vPath):
    v_ilc = np.load(vPath)
    print(f"[ILC] Loaded v_ilc: step0={v_ilc[0]}")
else:
    print("[ILC] No v_ilc found — zero feedforward (iteration 0).")

# ── Data file setup ────────────────────────────────────────────────────────
existing = sorted(glob.glob(os.path.join(DATA_DIR, 'data_*.npz')),
                  key=lambda f: int(os.path.splitext(os.path.basename(f))[0].split('_')[1]))
k       = len(existing) + 1
OUT_FILE = os.path.join(DATA_DIR, f'data_{k}.npz')
R_seq   = np.array([compute_ref(i) for i in range(N_STEPS)])

print(f"\n=== TCP_plain.py — Aero2 Plaintext ILC  (k={k}) ===")

# ── Scope ──────────────────────────────────────────────────────────────────
scope = MultiScope(rows=3, cols=2, title=f'Aero2 Plaintext ILC (k={k})', fps=30)

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

# ── Logging buffers ────────────────────────────────────────────────────────
U_log = np.zeros((N_STEPS, 2))
Y_log = np.zeros((N_STEPS, 2))

# ── Control loop ───────────────────────────────────────────────────────────
def control_loop():
    global KILL_THREAD, v_ilc

    myAero2 = Aero2(id=0, hardware=1, readMode=0, frequency=FREQUENCY)
    step = 0
    z    = np.zeros(n)   # controller (observer) state — reset each trial

    try:
        while step < N_TOTAL_STEPS and not KILL_THREAD:
            loop_start = time.perf_counter()
            timestamp  = step * Ts

            myAero2.read_analog_encoder_other_channels()
            y = np.array([myAero2.pitchAngle, myAero2.yawAngle])

            if step < N_STEPS:
                ref_now = compute_ref(step)

                # Plaintext control: u = H*z + Nbar*r + v_ilc  (pre-clip, for state update)
                u = H_ @ z + Nbar @ ref_now + v_ilc[step]

                # Observer state update (uses unclipped u, mirroring EncILC.go uReEnc)
                z = F_ @ z + G_ @ y + R_ @ u + P_ @ ref_now

                u_cmd = np.clip(u, -VMAX, VMAX)
                U_log[step] = u_cmd
                Y_log[step] = y
            else:
                ref_now = np.zeros(2)
                u_cmd   = np.zeros(2)

            e_now  = ref_now - y
            V_main = float(u_cmd[0])
            V_tail = float(u_cmd[1])

            if abs(y[0]) > PITCH_LIMIT:
                print(f"[SAFETY] Pitch limit at step {step}: {np.rad2deg(y[0]):.1f}°")
                V_main, V_tail = 0.0, 0.0

            myAero2.write_voltage(V_main, V_tail)

            scope.axes[0].sample(timestamp, [np.rad2deg(y[0]),      np.rad2deg(ref_now[0])])
            scope.axes[1].sample(timestamp, [np.rad2deg(y[1]),      np.rad2deg(ref_now[1])])
            scope.axes[2].sample(timestamp, [V_main])
            scope.axes[3].sample(timestamp, [V_tail])
            scope.axes[4].sample(timestamp, [np.rad2deg(e_now[0])])
            scope.axes[5].sample(timestamp, [np.rad2deg(e_now[1])])

            if step == 0 or (step + 1) % 50 == 0:
                print(f"[CTRL] step {step+1:3d}/{N_STEPS}  u=[{V_main:.3f}, {V_tail:.3f}] V")

            step   += 1
            elapsed = time.perf_counter() - loop_start
            if Ts - elapsed > 0:
                time.sleep(Ts - elapsed)

    except Exception:
        print("\n[ERROR] Control loop exception:")
        traceback.print_exc()
    finally:
        myAero2.write_voltage(0.0, 0.0)
        myAero2.terminate()

        n_valid  = min(step, N_STEPS)
        R_final  = R_seq[:n_valid]
        Y_final  = Y_log[:n_valid]
        E_final  = R_final - Y_final

        # ILC update: v_ilc += L_opt @ (R - Y), zero-pad partial runs
        E_vec = E_final.flatten()
        if n_valid < N_STEPS:
            E_vec = np.concatenate([E_vec, np.zeros(pN - len(E_vec))])
        dU_vec = Lopt @ E_vec
        v_ilc  = np.clip(v_ilc + dU_vec.reshape(N_STEPS, m), -VMAX, VMAX)
        np.save(vPath, v_ilc)

        rmsP = np.rad2deg(np.sqrt(np.mean(E_final[:, 0] ** 2)))
        rmsY = np.rad2deg(np.sqrt(np.mean(E_final[:, 1] ** 2)))
        print(f"\n[ILC] Updated feedforward saved → {vPath}")
        print(f"[ILC] E_rms: Pitch={rmsP:.3f}°  Yaw={rmsY:.3f}°")

        np.savez(OUT_FILE,
                 U=U_log[:n_valid], Y=Y_final, R=R_final, E=E_final,
                 V_ILC=v_ilc[:n_valid],
                 k=np.int32(k), n_valid=np.int32(n_valid), Ts=Ts)
        print(f"[DATA] Saved → {OUT_FILE}")

        if n_valid > 0:
            epc = np.rad2deg(np.sqrt(np.mean(E_final[:, 0] ** 2)))
            eyc = np.rad2deg(np.sqrt(np.mean(E_final[:, 1] ** 2)))
            etc = np.sqrt(epc ** 2 + eyc ** 2)
            prev_line = ""
            if k > 1 and existing:
                prev    = np.load(existing[-1])
                nv      = int(prev['n_valid'])
                Ec_prev = R_seq[:nv] - prev['Y'][:nv]
                ep_prev = np.rad2deg(np.sqrt(np.mean(Ec_prev[:, 0] ** 2)))
                ey_prev = np.rad2deg(np.sqrt(np.mean(Ec_prev[:, 1] ** 2)))
                et_prev = np.sqrt(ep_prev ** 2 + ey_prev ** 2)
                sym     = lambda d: "▼" if d < 0 else "▲"
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

# ── Main ───────────────────────────────────────────────────────────────────
thread = Thread(target=control_loop)
thread.start()

try:
    while thread.is_alive() and not KILL_THREAD:
        MultiScope.refreshAll()
        time.sleep(0.01)
except KeyboardInterrupt:
    KILL_THREAD = True

thread.join()
