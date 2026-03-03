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

# ========================================================
# 2. Go 암호화 라이브러리 로드
# ========================================================
lib_name = "../crypto/client_crypto.so"
if os.name == 'nt' and not os.path.exists(lib_name):
    lib_name = "../client_crypto.dll"

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

# ========================================================
# 3. Helper Functions (시뮬레이션과 동일)
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
# 4. Scope & Signal Setup
# ========================================================
KILL_THREAD = False 
def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)

scopePendulum = Scope(title='Pendulum alpha (rad)', timeWindow=10, xLabel='Time (s)', yLabel='Rad')
scopePendulum.attachSignal(name='Alpha', width=1)
scopeBase = Scope(title='Base theta (rad)', timeWindow=10, xLabel='Time (s)', yLabel='Rad')
scopeBase.attachSignal(name='Theta', width=1)
scopeVoltage = Scope(title='Control Input (V)', timeWindow=10, xLabel='Time (s)', yLabel='Volts')
scopeVoltage.attachSignal(name='u', width=1)

# ========================================================
# 5. Main Control Loop
# ========================================================
def control_loop():
    HOST = '127.0.0.1'
    PORT = 8000
    simulationTime = 60 
    N_STATE, N_INPUT, N_OUTPUT = 4, 1, 2
    VAL_S, VAL_L, VAL_R = 1.0/1000.0, 1.0/1000000.0, 1.0/1000.0
    frequency = 50 
    
    lib.InitCrypto(N_STATE, N_INPUT, N_OUTPUT, VAL_S, VAL_L, VAL_R)

    with QubeServo3(hardware=1, pendulum=1, frequency=frequency) as myQube:
        print(">> Hardware Initialized.")
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
            client_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            while not KILL_THREAD:
                try:
                    client_socket.connect((HOST, PORT))
                    print(">> Connected to Controller!")
                    break
                except ConnectionRefusedError:
                    time.sleep(1)
            
            startTime = time.time()
            
            while not KILL_THREAD:
<<<<<<< HEAD
                # 1. Sensor Read 
=======
                # -----------------------------------------------------
                # 1. 측정 (시뮬레이션 y = plant.get_output())
                # -----------------------------------------------------
>>>>>>> 666128f5a5b79cc9e6efea3d32d512946bf89db6
                myQube.read_outputs()
                theta = myQube.motorPosition * -1 
                alpha = (myQube.pendulumPosition - np.pi + np.pi) % (2*np.pi) - np.pi
                y = [theta, alpha]

<<<<<<< HEAD
                # 2. Receive Encrypted Control Input
                u_enc_bytes = recv_packet(client_socket) # Serialized data
                if not u_enc_bytes: break

                # 3. Decryption
                u = decrypt_helper(u_enc_bytes, N_INPUT) 
                if u is None: break
                print(u) # Debug

                # 4. Send Enc(y) + ReEnc(u)
=======
                # -----------------------------------------------------
                # 2. 수신 (시뮬레이션 u_enc = recv_packet)
                # -----------------------------------------------------
                u_enc_bytes = recv_packet(client_socket)
                if not u_enc_bytes: break

                # -----------------------------------------------------
                # 3. 복호화 (시뮬레이션 u = decrypt_helper)
                # -----------------------------------------------------
                u = decrypt_helper(u_enc_bytes, N_INPUT) 
                if u is None: break
                print(u) # 시뮬레이션과 동일한 출력

                # -----------------------------------------------------
                # 4. 송신 (시뮬레이션 combined_vec = y + u)
                # [중요] u를 가공하지 않고 받은 그대로 다시 보냅니다.
                # -----------------------------------------------------
>>>>>>> 666128f5a5b79cc9e6efea3d32d512946bf89db6
                combined_vec = y + u 
                combined_bytes = encrypt_helper(combined_vec)
                send_packet(client_socket, combined_bytes)

<<<<<<< HEAD
                # 5. Actuator Write (near equalibria)
=======
                # -----------------------------------------------------
                # 5. 구동 (하드웨어에만 적용되는 전압 제한)
                # -----------------------------------------------------
                # 세워져 있을 때만 모터 가동 (이건 안전 장치이므로 유지)
>>>>>>> 666128f5a5b79cc9e6efea3d32d512946bf89db6
                if abs(math.degrees(alpha)) < 15:
                    voltage = np.clip(u[0], -10.0, 10.0)
                else:
                    voltage = 0.0
                
                myQube.write_voltage(-1 * voltage)

<<<<<<< HEAD
                # 6. Plotting
=======
                # -----------------------------------------------------
                # 6. 로깅
                # -----------------------------------------------------
>>>>>>> 666128f5a5b79cc9e6efea3d32d512946bf89db6
                timeStamp = time.time() - startTime
                if timeStamp > simulationTime: break
                scopeBase.sample(timeStamp, [theta])
                scopePendulum.sample(timeStamp, [alpha])
                scopeVoltage.sample(timeStamp, [voltage])

            myQube.write_voltage(0.0)
            print(">> Finished.")

def main():
    global KILL_THREAD
    thread = Thread(target=control_loop)
    thread.daemon = True
    thread.start()
    while thread.is_alive() and not KILL_THREAD:
        Scope.refreshAll()
        time.sleep(0.01)
    KILL_THREAD = True

if __name__ == "__main__":
    main()