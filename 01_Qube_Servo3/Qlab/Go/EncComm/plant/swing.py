import sys
import os
import json
import time
import struct
import ctypes
import socket
import signal
import math
import numpy as np
from threading import Thread

_script_dir = os.path.dirname(os.path.abspath(__file__))

# 1. Quanser Library Setup (스크립트 위치 기준 상대경로)
sys.path.insert(0, os.path.join(_script_dir, '..', '..', '..', '..', '..', '00_libraries', 'python'))
from pal.products.qube import QubeServo3
from pal.utilities.scope import Scope
from pal.utilities.math import ddt_filter

# 2. 파라미터 로드 (offline.go 가 생성한 params.json)
_params_path = os.path.abspath(os.path.join(_script_dir, '..', 'controller', 'enc_data', 'params.json'))
with open(_params_path) as _f:
    _cfg = json.load(_f)
N_STATE  = _cfg['n']
N_INPUT  = _cfg['m']
N_OUTPUT = _cfg['p']

# 3. Lattigo 암호화 라이브러리 로드 (스크립트 위치 기준 상대경로)
if os.name == 'nt':
    _lib_path = os.path.abspath(os.path.join(_script_dir, '..', 'crypto', 'client_crypto.dll'))
else:
    _lib_path = os.path.abspath(os.path.join(_script_dir, '..', 'crypto', 'client_crypto.so'))

try:
    lib = ctypes.CDLL(_lib_path)
    print(f"[Info] 라이브러리 로드 성공: {_lib_path}")
except OSError as e:
    print(f"[Error] 라이브러리 로드 실패: {e}")
    exit(1)

lib.InitCrypto.argtypes = [ctypes.c_char_p]
lib.InitCrypto.restype = None
lib.EncryptVector.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
lib.EncryptVector.restype = ctypes.POINTER(ctypes.c_char)
lib.DecryptVector.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
lib.DecryptVector.restype = ctypes.POINTER(ctypes.c_double)
lib.FreePtr.argtypes = [ctypes.c_void_p]


# 통신 및 암복호화 함수들...
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

# Swing-up params
SIGN_THETA = -1.0
SWING_KICK_DIR = 1.0
VMAX = 7.0
mp, Lp, g = 0.024, 0.129, 9.81
l = Lp / 2
Jp = mp * (Lp**2) / 3
mu = 2.5
switch_deg = 20.0

# Full state LQR gain
K_LOCAL = np.array([-2.0, 20.0, -1.5, 2.5]) 

KILL_THREAD = False 
def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)

# Plotting
scopePendulum = Scope(title='Pendulum alpha (rad)', timeWindow=10, xLabel='Time (s)', yLabel='Rad')
scopePendulum.attachSignal(name='Alpha', width=1)
scopeBase = Scope(title='Base theta (rad)', timeWindow=10, xLabel='Time (s)', yLabel='Rad')
scopeBase.attachSignal(name='Theta', width=1)
scopeVoltage = Scope(title='Control Input (V)', timeWindow=10, xLabel='Time (s)', yLabel='Volts')
scopeVoltage.attachSignal(name='u', width=1)

# 4. Main Control Loop
def control_loop():
    global KILL_THREAD

    HOST, PORT = '127.0.0.1', 8000
    # 50Hz = 20ms sampling rate
    frequency = 50

    lib.InitCrypto(_params_path.encode())

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    print(">> Waiting for Server Connection...")
    while not KILL_THREAD:
        try:
            client_socket.connect((HOST, PORT))
            print(">> Connected to Encrypted Controller!")
            break
        except ConnectionRefusedError:
            time.sleep(1)

    with QubeServo3(hardware=0, pendulum=1, frequency=frequency) as myQube:
        print(">> Step 1: Swing-up & Local Balancing Started.")
        
        state_theta_dot = np.array([0,0], dtype=np.float64)
        state_alpha_dot = np.array([0,0], dtype=np.float64)
        
        stable_start_time = None
        is_fully_stable = False
        startTime = time.time()

        # PHASE 1: Swing-up & LQR (Local)

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
                elif time.time() - stable_start_time > 2.0:
                    is_fully_stable = True
            else:
                stable_start_time = None
                E_total = mp*g*l*(math.cos(alpha)-1) + 0.5*Jp*(alpha_dot**2)
                term = np.sign(alpha_dot * math.cos(alpha))
                u = SWING_KICK_DIR * mu * (0.0 - E_total) * term

            voltage = np.clip(u, -VMAX, VMAX)
            myQube.write_voltage(-1 * voltage)
            
            ts = time.time() - startTime
            scopePendulum.sample(ts, [alpha])
            scopeBase.sample(ts, [theta])
            scopeVoltage.sample(ts, [-1 * voltage])

        if KILL_THREAD: 
            client_socket.close()
            return

        # ---------------------------------------------------------------------
        # PHASE 2: Encrypted Control 
        # ---------------------------------------------------------------------
        print(">> Step 2: Switching to Encrypted Control.")
        
        while not KILL_THREAD:


            # 2. recieve Enc(u)
            u_enc_bytes = recv_packet(client_socket)
            if not u_enc_bytes: break
            u_dec = decrypt_helper(u_enc_bytes, N_INPUT)
            if u_dec is None: break


            # 1. sensor read
            myQube.read_outputs()
            theta = myQube.motorPosition * SIGN_THETA
            alpha = (myQube.pendulumPosition - np.pi + np.pi) % (2*np.pi) - np.pi
            y = [theta, alpha]



            # 3. actuator write
            if abs(math.degrees(alpha)) < 20:
                voltage = np.clip(u_dec[0], -VMAX, VMAX)
            else:
                print(">> Out of Range! Emergency Stop.")
                voltage = 0.0
                KILL_THREAD = True
            myQube.write_voltage(voltage)
            
            


            # 4. send Enc(y) & ReEnc(u)
            combined_vec = y + u_dec 
            combined_bytes = encrypt_helper(combined_vec)
            send_packet(client_socket, combined_bytes)

            # Plotting
            ts = time.time() - startTime
            scopePendulum.sample(ts, [alpha])
            scopeBase.sample(ts, [theta])
            scopeVoltage.sample(ts, [voltage])

        # Finish loop
        myQube.write_voltage(0.0)
        client_socket.close()
        print(">> Finished.")

def main():
    global KILL_THREAD
    thread = Thread(target=control_loop)
    thread.daemon = True
    thread.start()
    
    # GUI 렌더링은 오직 여기서만 수행됩니다.
    while thread.is_alive() and not KILL_THREAD:
        Scope.refreshAll()
        time.sleep(0.01)
    
    KILL_THREAD = True

if __name__ == "__main__":
    main()