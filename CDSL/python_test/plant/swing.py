import sys
import os
import time
import struct
import ctypes
import socket
import signal
import math
import numpy as np
from threading import Thread

# ========================================================
# 1. Quanser Library Setup
# ========================================================
sys.path.append(r"C:\Quanser\0_libraries\python")
from pal.products.qube import QubeServo3
from pal.utilities.scope import Scope
from pal.utilities.math import ddt_filter

# ========================================================
# 2. Go 암호화 라이브러리 및 통신 설정
# ========================================================
lib_name = "../crypto/client_crypto.so"
if os.name == 'nt' and not os.path.exists(lib_name):
    lib_name = "./client_crypto.dll"

try:
    lib = ctypes.CDLL(lib_name)
    print(f"[Info] 라이브러리 로드 성공: {lib_name}")
except OSError as e:
    print(f"[Error] 라이브러리 로드 실패: {e}")
    exit(1)

lib.InitCrypto.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_double, ctypes.c_double, ctypes.c_double]
lib.InitCrypto.restype = None
lib.EncryptVector.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
lib.EncryptVector.restype = ctypes.POINTER(ctypes.c_char)
lib.DecryptVector.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
lib.DecryptVector.restype = ctypes.POINTER(ctypes.c_double)
lib.FreePtr.argtypes = [ctypes.c_void_p]

def send_packet(sock, data_bytes):
    length = len(data_bytes)
    sock.sendall(struct.pack('<I', length) + data_bytes)

def recv_packet(sock):
    raw_len = recv_exact(sock, 4)
    if not raw_len: return None
    length = struct.unpack('<I', raw_len)[0]
    return recv_exact(sock, length)

def recv_exact(sock, n):
    data = b''
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet: return None
        data += packet
    return data

def encrypt_helper(vec):
    c_arr = (ctypes.c_double * len(vec))(*vec)
    size = ctypes.c_int()
    ptr = lib.EncryptVector(c_arr, len(vec), ctypes.byref(size))
    data = ctypes.string_at(ptr, size.value)
    lib.FreePtr(ptr)
    return data

def decrypt_helper(data_bytes, out_len):
    ptr = lib.DecryptVector(data_bytes, len(data_bytes), out_len)
    if not ptr: return None
    res = [ptr[i] for i in range(out_len)]
    lib.FreePtr(ptr)
    return res

# ========================================================
# 3. 제어 파라미터 및 스위치
# ========================================================
SIGN_THETA = -1.0 
SIGN_VOLTAGE = -1.0 
SWING_KICK_DIR = 1.0  
VMAX = 7.0

# Swing-up Parameters
mp, Lp, g = 0.024, 0.129, 9.81
l = Lp / 2
Jp = mp * (Lp**2) / 3 
mu = 150.0 
switch_deg = 15.0 

# Balancing LQR Gain (Local)
K_LOCAL = np.array([-2.0, 28.0, -1.5, 2.5]) 

KILL_THREAD = False 
def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)

# ========================================================
# 4. Main Control Loop
# ========================================================
def control_loop():
    # 전역 변수 참조 선언 (에러 해결 핵심)
    global KILL_THREAD
    
    HOST, PORT = '127.0.0.1', 8000
    N_STATE, N_INPUT, N_OUTPUT = 4, 1, 2
    VAL_S, VAL_L, VAL_R = 1.0/1000.0, 1.0/1000000.0, 1.0/1000.0
    frequency = 50 
    
    lib.InitCrypto(N_STATE, N_INPUT, N_OUTPUT, VAL_S, VAL_L, VAL_R)

    # Scopes
    scopePendulum = Scope(title='Pendulum alpha', timeWindow=10, xLabel='Time (s)', yLabel='Rad')
    scopePendulum.attachSignal(name='Alpha')
    scopeVoltage = Scope(title='Control Input', timeWindow=10, xLabel='Time (s)', yLabel='Volts')
    scopeVoltage.attachSignal(name='Voltage')

    with QubeServo3(hardware=1, pendulum=1, frequency=frequency) as myQube:
        print(">> Step 1: Swing-up & Local Balancing Started.")
        
        state_theta_dot = np.array([0,0], dtype=np.float64)
        state_alpha_dot = np.array([0,0], dtype=np.float64)
        
        stable_start_time = None
        is_fully_stable = False
        startTime = time.time()

        # ---------------------------------------------------------------------
        # PHASE 1: Local Swing-up & LQR (로컬 안착)
        # ---------------------------------------------------------------------
        while not is_fully_stable and not KILL_THREAD:
            myQube.read_outputs()
            theta = myQube.motorPosition * SIGN_THETA
            alpha_raw = myQube.pendulumPosition
            alpha = (alpha_raw - np.pi + np.pi) % (2*np.pi) - np.pi
            
            theta_dot, state_theta_dot = ddt_filter(theta, state_theta_dot, 50, 1/frequency)
            alpha_dot, state_alpha_dot = ddt_filter(alpha, state_alpha_dot, 100, 1/frequency)
            
            alpha_deg = abs(math.degrees(alpha))
            u = 0.0

            if alpha_deg < switch_deg:
                x_meas = np.array([theta, alpha, theta_dot, alpha_dot])
                u = -1.0 * float(np.dot(K_LOCAL, x_meas))
                
                if stable_start_time is None:
                    stable_start_time = time.time()
                elif time.time() - stable_start_time > 1.5:
                    is_fully_stable = True
            else:
                stable_start_time = None
                E_total = mp*g*l*(math.cos(alpha)-1) + 0.5*Jp*(alpha_dot**2)
                term = np.sign(alpha_dot * math.cos(alpha))
                u_raw = SWING_KICK_DIR * mu * (0.0 - E_total) * term
                if (theta > 0 and u_raw > 0) or (theta < 0 and u_raw < 0): u = 0.0
                else: u = u_raw

            voltage = np.clip(u, -VMAX, VMAX)
            myQube.write_voltage(voltage * SIGN_VOLTAGE)
            
            ts = time.time() - startTime
            scopePendulum.sample(ts, [alpha])
            scopeVoltage.sample(ts, [voltage * SIGN_VOLTAGE])
            Scope.refreshAll()

        if KILL_THREAD: return

        # ---------------------------------------------------------------------
        # PHASE 2: Encrypted Control (암호화 제어 전환)
        # ---------------------------------------------------------------------
        print(">> Step 2: Switching to Encrypted Control.")
        
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
            client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            while not KILL_THREAD:
                try:
                    client_socket.connect((HOST, PORT))
                    print(">> Connected to Encrypted Controller!")
                    break
                except ConnectionRefusedError:
                    time.sleep(0.5)

            while not KILL_THREAD:
                myQube.read_outputs()
                theta = myQube.motorPosition * SIGN_THETA
                alpha = (myQube.pendulumPosition - np.pi + np.pi) % (2*np.pi) - np.pi
                y = [theta, alpha]

                u_enc_bytes = recv_packet(client_socket)
                if not u_enc_bytes: break

                u_dec = decrypt_helper(u_enc_bytes, N_INPUT)
                if u_dec is None: break

                # 피드백 송신
                combined_vec = y + u_dec 
                combined_bytes = encrypt_helper(combined_vec)
                send_packet(client_socket, combined_bytes)

                if abs(math.degrees(alpha)) < 20:
                    voltage = np.clip(u_dec[0], -VMAX, VMAX)
                else:
                    print(">> Out of Range! Emergency Stop.")
                    voltage = 0.0
                    KILL_THREAD = True

                myQube.write_voltage(voltage * SIGN_VOLTAGE)

                ts = time.time() - startTime
                scopePendulum.sample(ts, [alpha])
                scopeVoltage.sample(ts, [voltage * SIGN_VOLTAGE])
                Scope.refreshAll()

        myQube.write_voltage(0.0)
        print(">> Finished.")

if __name__ == "__main__":
    control_loop()