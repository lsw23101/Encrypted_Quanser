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
# 경로가 다르면 수정해주세요
sys.path.append(r"C:\Quanser\0_libraries\python")
from pal.products.qube import QubeServo2, QubeServo3
from pal.utilities.scope import Scope

# ========================================================
# 2. Go 암호화 라이브러리 로드
# ========================================================
lib_name = "./client_crypto.so"
if os.name == 'nt' and not os.path.exists(lib_name):
    lib_name = "./client_crypto.dll"

try:
    lib = ctypes.CDLL(lib_name)
    print(f"[Info] 라이브러리 로드 성공: {lib_name}")
except OSError as e:
    print(f"[Error] 라이브러리를 찾을 수 없습니다.: {e}")
    exit(1)

# Go 함수 시그니처 정의
lib.InitCrypto.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_double, ctypes.c_double, ctypes.c_double]
lib.InitCrypto.restype = None

lib.EncryptVector.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
lib.EncryptVector.restype = ctypes.POINTER(ctypes.c_char)

lib.DecryptVector.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
lib.DecryptVector.restype = ctypes.POINTER(ctypes.c_double)

lib.FreePtr.argtypes = [ctypes.c_void_p]

# ========================================================
# 3. Helper Functions (Network & Crypto)
# ========================================================
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
# 4. Scope & Thread Setup
# ========================================================
global KILL_THREAD
KILL_THREAD = False
def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)

# Scopes
scopePendulum = Scope(title='Pendulum alpha (rad)', timeWindow=10, xLabel='Time (s)', yLabel='Rad')
scopePendulum.attachSignal(name='Alpha', width=1)

scopeBase = Scope(title='Base theta (rad)', timeWindow=10, xLabel='Time (s)', yLabel='Rad')
scopeBase.attachSignal(name='Theta', width=1)

scopeVoltage = Scope(title='Control Input (V)', timeWindow=10, xLabel='Time (s)', yLabel='Volts')
scopeVoltage.attachSignal(name='u', width=1)

# ========================================================
# 5. Main Control Loop (Hardware + FHE + Zero Masking)
# ========================================================
def control_loop():
    # --- Configuration ---
    HOST = '127.0.0.1'
    PORT = 8000
    simulationTime = 60 # 60초
    
    # System Dimensions
    N_STATE, N_INPUT, N_OUTPUT = 4, 1, 2
    # Crypto Params
    VAL_S, VAL_L, VAL_R = 1.0/1000.0, 1.0/1000000.0, 1.0/1000.0
    
    # Qube Config
    frequency = 50 # 50Hz (20ms)
    QubeClass = QubeServo3 # or QubeServo2
    
    # Init Crypto
    print(f">> Go Crypto Init...")
    lib.InitCrypto(N_STATE, N_INPUT, N_OUTPUT, VAL_S, VAL_L, VAL_R)

    # --- Hardware Loop ---
    with QubeClass(hardware=1, pendulum=1, frequency=frequency) as myQube:
        print(">> Hardware Initialized.")
        print(">> [READY] Pendulum을 손으로 세우면 제어가 시작됩니다.")
        
        # Socket Connection
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
            # Reconnect Logic
            while not KILL_THREAD:
                try:
                    client_socket.connect((HOST, PORT))
                    print(">> Connected to Controller!")
                    break
                except ConnectionRefusedError:
                    time.sleep(1)
            
            startTime = time.time()
            timeStamp = 0
            
            # Scope Sampling Rate control
            countMax = frequency / 50 
            count = 0

            while timeStamp < simulationTime and not KILL_THREAD:
                # -------------------------------------------------
                # 1. Read Sensors (y 측정)
                # -------------------------------------------------
                myQube.read_outputs()
                
                theta = myQube.motorPosition * -1  # 부호 보정
                alpha_raw = myQube.pendulumPosition
                
                # Alpha Unwrapping (Upright = 0 rad)
                alpha = (alpha_raw - np.pi + np.pi) % (2*np.pi) - np.pi
                
                # y = [theta, alpha]
                # 일단 실제 값은 가지고 있음
                real_y = [theta, alpha]

                # -------------------------------------------------
                # 2. Receive Encrypted u (from Controller)
                # -------------------------------------------------
                u_enc_bytes = recv_packet(client_socket)
                if not u_enc_bytes: 
                    print("Disconnected.")
                    break

                # -------------------------------------------------
                # 3. Decrypt u
                # -------------------------------------------------
                u_list = decrypt_helper(u_enc_bytes, N_INPUT) # [u1]
                if u_list is None: break
                
                u_val_from_ctrl = u_list[0] 

                # -------------------------------------------------
                # 4. Zero-Masking & Actuation logic
                # -------------------------------------------------
                alpha_deg = math.degrees(alpha)
                
                # 제어 시작 조건: 수직 +- 15도 이내
                is_controllable = abs(alpha_deg) < 15

                if is_controllable:
                    # [상태 1: 제어 중]
                    # 컨트롤러가 계산한 u를 그대로 사용
                    voltage = np.clip(u_val_from_ctrl, -10.0, 10.0)
                    
                    # 컨트롤러에게 보낼 데이터: 실제 y, 실제 u
                    y_to_send = real_y
                    u_to_send = [voltage] # 리스트 형태
                    
                else:
                    # [상태 2: 펜듈럼 다운 / 손으로 잡기 전]
                    # 모터 끔 (안전)
                    voltage = 0.0
                    
                    # ★ 중요: 컨트롤러 상태(Observer) 발산 방지 ★
                    # 컨트롤러에게 "현재 오차 0, 입력 0" 이라고 거짓말을 함.
                    # 이렇게 하면 x_next = F*0 + G*0 + R*0 = 0 이 되어 상태가 0으로 유지됨.
                    y_to_send = [0.0, 0.0] 
                    u_to_send = [0.0]

                # 하드웨어 구동
                myQube.write_voltage(voltage)

                # -------------------------------------------------
                # 5. Pack [y | u] -> Encrypt -> Send
                # -------------------------------------------------
                # 결정된 y_to_send와 u_to_send를 묶어서 보냄
                combined_vec = y_to_send + u_to_send 
                combined_bytes = encrypt_helper(combined_vec)
                
                send_packet(client_socket, combined_bytes)

                # -------------------------------------------------
                # 6. Logging & Plotting
                # -------------------------------------------------
                timeStamp = time.time() - startTime
                
                count += 1
                if count >= countMax:
                    scopeBase.sample(timeStamp, [theta])
                    scopePendulum.sample(timeStamp, [alpha])
                    scopeVoltage.sample(timeStamp, [voltage])
                    count = 0
            
            # Loop Ends
            myQube.write_voltage(0.0)
            print(">> Simulation Finished. Motor Stopped.")

# ========================================================
# 6. Entry Point
# ========================================================
def main():
    thread = Thread(target=control_loop)
    thread.start()

    print(">> Plotting Thread Started.")
    
    while thread.is_alive() and (not KILL_THREAD):
        Scope.refreshAll()
        time.sleep(0.01)

    input('Press the enter key to exit.\n')
    global KILL_THREAD
    KILL_THREAD = True

if __name__ == "__main__":
    main()