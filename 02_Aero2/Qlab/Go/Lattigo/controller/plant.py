## plant.py
# Aero2 Plant side — Encrypted TCP controller (Go Lattigo)
# 실행: controller/ 폴더에서  python plant.py
#
# 통신 흐름 (매 스텝):
#   1. recv u_enc    (Controller → Plant)  암호문 패킷
#   2. decrypt u     → [V_main_raw, V_tail_raw]
#   3. apply u       (write_voltage)       클리핑은 여기서만
#   4. read sensor   (hardware tick @ 50Hz)
#   5. reenc([theta_p, theta_y, u[0], u[1]])
#   6. send combined (Plant → Controller) 암호문 패킷
#   ※ r 계산은 controller.go 내부, 클리핑 전 u 를 재암호화
# -----------------------------------------------------------------------

import sys
import os
import time
import struct
import ctypes
import socket
import signal
import numpy as np
from threading import Thread

# ── Quanser Library ──────────────────────────────────────────────────────
# controller/ → Lattigo → Go → Qlab → 02_Aero2 → Encrypted_Quanser
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', '..', '..', '..', '..', '00_libraries', 'python'))
from pal.products.aero2 import Aero2
from pal.utilities.scope import MultiScope

# ── Go 암호화 공유 라이브러리 ────────────────────────────────────────────
# controller/ → ../crypto/
_base    = os.path.dirname(os.path.abspath(__file__))
lib_path = os.path.join(_base, '..', 'crypto', 'client_crypto.so')
if os.name == 'nt':
    dll_path = os.path.join(_base, '..', 'crypto', 'client_crypto.dll')
    if os.path.exists(dll_path):
        lib_path = dll_path

try:
    lib = ctypes.CDLL(lib_path)
    print(f"[Info] 라이브러리 로드 성공: {lib_path}")
except OSError as e:
    print(f"[Error] 라이브러리 로드 실패: {e}")
    exit(1)

lib.InitCrypto.argtypes    = [ctypes.c_int, ctypes.c_int, ctypes.c_int,
                               ctypes.c_double, ctypes.c_double, ctypes.c_double]
lib.InitCrypto.restype     = None
lib.EncryptVector.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.c_int,
                               ctypes.POINTER(ctypes.c_int)]
lib.EncryptVector.restype  = ctypes.POINTER(ctypes.c_char)
lib.DecryptVector.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
lib.DecryptVector.restype  = ctypes.POINTER(ctypes.c_double)
lib.FreePtr.argtypes       = [ctypes.c_void_p]
lib.FreePtr.restype        = None

# ── Run parameters ──────────────────────────────────────────────────────
RUN_TIME    = 50.0
FREQUENCY   = 50
VMAX        = 15.0
PITCH_LIMIT = np.deg2rad(40)

# Aero2: n=4, m=2, p=2 — offline.go 와 동일
N_STATE, N_INPUT, N_OUTPUT = 4, 2, 2
VAL_S = 1.0 / 1000.0
VAL_L = 1.0 / 1000000.0
VAL_R = 1.0 / 1000.0

# scope 표시용 레퍼런스 (실제 계산은 controller.go 내부)
r_disp = np.array([np.deg2rad(30.0), np.deg2rad(-30.0)])

# ── Kill flag ────────────────────────────────────────────────────────────
KILL_THREAD = False
def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)

# ── TCP helpers ──────────────────────────────────────────────────────────
def recv_exact(sock, n):
    data = b''
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            return None
        data += chunk
    return data

def send_packet(sock, data_bytes):
    sock.sendall(struct.pack('<I', len(data_bytes)) + data_bytes)

def recv_packet(sock):
    raw_len = recv_exact(sock, 4)
    if not raw_len:
        return None
    length = struct.unpack('<I', raw_len)[0]
    return recv_exact(sock, length)

def encrypt_helper(vec):
    c_arr = (ctypes.c_double * len(vec))(*vec)
    size  = ctypes.c_int()
    ptr   = lib.EncryptVector(c_arr, len(vec), ctypes.byref(size))
    data  = ctypes.string_at(ptr, size.value)
    lib.FreePtr(ptr)
    return data

def decrypt_helper(data_bytes, out_len):
    ptr = lib.DecryptVector(data_bytes, len(data_bytes), out_len)
    if not ptr:
        return None
    res = [ptr[i] for i in range(out_len)]
    lib.FreePtr(ptr)
    return res

# ── Scope ────────────────────────────────────────────────────────────────
scope = MultiScope(rows=3, cols=2, title='Aero2 Encrypted Controller', fps=30)

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

# ── Control loop ─────────────────────────────────────────────────────────
def control_loop():
    lib.InitCrypto(N_STATE, N_INPUT, N_OUTPUT, VAL_S, VAL_L, VAL_R)

    myAero2   = Aero2(id=0, hardware=0, readMode=0, frequency=FREQUENCY)
    startTime = time.time()
    timestamp = 0.0
    theta_p   = 0.0  # 첫 스텝 안전 체크용

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

            while not KILL_THREAD:
                try:
                    sock.connect(('127.0.0.1', 8000))
                    print(">> Connected to Controller!")
                    break
                except ConnectionRefusedError:
                    time.sleep(1)

            while timestamp < RUN_TIME and not KILL_THREAD:

                # 1. recv u_enc (controller가 먼저 전송)
                u_enc_bytes = recv_packet(sock)
                if u_enc_bytes is None:
                    break

                # 2. 복호화 → [V_main_raw, V_tail_raw]
                u = decrypt_helper(u_enc_bytes, N_INPUT)
                if u is None:
                    break

                # 3. 클리핑 & 하드웨어 적용 (클리핑은 오직 여기서만)
                V_main = float(np.clip(u[0], -VMAX, VMAX))
                V_tail = float(np.clip(u[1], -VMAX, VMAX))

                if abs(theta_p) > PITCH_LIMIT:
                    V_main, V_tail = 0.0, 0.0
                    print(f"  [SAFETY] pitch={np.rad2deg(theta_p):.1f}° — motor OFF")

                myAero2.write_voltage(V_main, V_tail)

                # 4. 센서 읽기 (50Hz 하드웨어 틱 블로킹)
                myAero2.read_analog_encoder_other_channels()
                theta_p = myAero2.pitchAngle
                theta_y = myAero2.yawAngle
                dp_meas = myAero2.pitchRate
                dy_meas = myAero2.yawRate

                # 5. 재암호화: [y | u_raw] = [theta_p, theta_y, u[0], u[1]]
                #    controller 언패킹: yCt = [:2], uReEnc = [2:]
                #    클리핑 전 원본 u 사용 → state 업데이트 정확도 보장
                combined_bytes = encrypt_helper([theta_p, theta_y, u[0], u[1]])

                # 6. send combined → Controller
                send_packet(sock, combined_bytes)

                # 7. Scope
                scope.axes[0].sample(timestamp, [np.rad2deg(theta_p), np.rad2deg(r_disp[0])])
                scope.axes[1].sample(timestamp, [np.rad2deg(theta_y), np.rad2deg(r_disp[1])])
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


# ── Main ─────────────────────────────────────────────────────────────────
def main():
    global KILL_THREAD
    thread = Thread(target=control_loop)
    thread.daemon = True
    thread.start()
    while thread.is_alive() and not KILL_THREAD:
        MultiScope.refreshAll()
        time.sleep(0.01)
    KILL_THREAD = True

if __name__ == "__main__":
    main()
