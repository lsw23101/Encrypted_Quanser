import socket
import time
import struct
import csv
import ctypes
import os
import numpy as np
import control as ct


# ==========================================
# 1. Go 암호화 라이브러리 로드
# ==========================================
lib_name = "./client_crypto.so"
if os.name == 'nt' and not os.path.exists(lib_name):
    lib_name = "./client_crypto.dll"

try:
    lib = ctypes.CDLL(lib_name)
    print(f"[Info] 라이브러리 로드 성공: {lib_name}")
except OSError as e:
    print(f"[Error] 라이브러리를 찾을 수 없습니다. (go build 확인 필요): {e}")
    exit(1)

# --- Go 함수 시그니처 정의 (Generic 버전) ---
lib.InitCrypto.argtypes = [
    ctypes.c_int, ctypes.c_int, ctypes.c_int,         # n, m, p
    ctypes.c_double, ctypes.c_double, ctypes.c_double # s, L, r
]
lib.InitCrypto.restype = None

lib.EncryptVector.argtypes = [
    ctypes.POINTER(ctypes.c_double), # 입력 배열 포인터
    ctypes.c_int,                    # 입력 배열 길이
    ctypes.POINTER(ctypes.c_int)     # 출력 크기 담을 포인터
]
lib.EncryptVector.restype = ctypes.POINTER(ctypes.c_char)

lib.DecryptVector.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
lib.DecryptVector.restype = ctypes.POINTER(ctypes.c_double)

lib.FreePtr.argtypes = [ctypes.c_void_p]


# ==========================================
# 2. Plant Model Class (Rotary Pendulum)
# ==========================================
class rotpen:
    def __init__(self, ts):
        # Continuous time linear model matrices
        A_c = np.array([[0, 0, 1, 0],
                        [0, 0, 0, 1],
                        [0, 149.275096865093, -0.0104427822555581, 0],
                        [0, 261.609107366662, -0.0103213545549121, 0]], dtype=float)
        B_c = np.array([[0],
                        [0],
                        [49.7275345502766],
                        [49.1493074043432]], dtype=float)
        C_c = np.array([[1, 0, 0, 0],
                        [0, 1, 0, 0]], dtype=float)
        D_c = np.array([[0],
                        [0]], dtype=float)
        
        # Create State Space system and discretize (c2d)
        sys_c = ct.ss(A_c, B_c, C_c, D_c)
        sys_d = sys_c.sample(ts, method='zoh')
        
        # Save discretized matrices
        self.A = sys_d.A
        self.B = sys_d.B
        self.C = C_c  # Output matrix C usually remains same unless direct feedthrough changes
        self.D = D_c

        # Initialize state
        self.xp = np.zeros((4,1), dtype=float)

    def set_init(self, x_init_list):
        # x_init_list: list or 1D array of length 4
        self.xp = np.array(x_init_list, dtype=float).reshape(4, 1)

    def state_update(self, u_list):
        # u_list: list or 1D array of input
        u = np.array(u_list, dtype=float).reshape(-1, 1)
        
        # Update state: x[k+1] = A x[k] + B u[k]
        self.xp = self.A @ self.xp + self.B @ u

    def get_output(self):
        # y[k] = C x[k]
        y = self.C @ self.xp
        # Convert numpy array back to standard python list for compatibility with crypto helper
        return y.flatten().tolist()


# ==========================================
# 3. Network & Crypto Helpers
# ==========================================
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
    # vec must be a list of floats
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


# ==========================================
# 4. Main Simulation
# ==========================================
def run_plant_simulation():
    # Simulation Parameters
    N_STATE = 4
    N_INPUT = 1
    N_OUTPUT = 2
    
    # Quantization Parameters
    VAL_S = 1.0/1000.0
    VAL_L = 1.0/1000000.0
    VAL_R = 1.0/1000.0
    
    # Init Crypto
    print(f">> Go Crypto Init (n={N_STATE}, m={N_INPUT}, p={N_OUTPUT})...")
    lib.InitCrypto(N_STATE, N_INPUT, N_OUTPUT, VAL_S, VAL_L, VAL_R)

    # Initialize Plant
    TS = 0.02 # Sampling Time (예시: 0.02초, 필요에 따라 조정)
    plant = rotpen(TS)
    
    # Initial State [0, 0.5, 0, 0]
    xp_ini = [0, 0.5, 0.0, 0.0]
    plant.set_init(xp_ini)
    
    data_log = [] 
    
    HOST = '127.0.0.1'
    PORT = 8000
    ITERATIONS = 500

    print(f"--- Plant Simulation (Optimized Flow / Control Lib) Started ---")
    
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
            # Connection Retry Loop
            while True:
                try:
                    client_socket.connect((HOST, PORT))
                    print(">> Controller에 연결 성공!")
                    break
                except ConnectionRefusedError:
                    time.sleep(1)

            for i in range(ITERATIONS):
                t_loop_start = time.perf_counter()

                # -----------------------------------------------------
                # 1. Output y 측정 (현재 상태 기준)
                # -----------------------------------------------------
                # Using class method (returns list)
                y = plant.get_output()

                # -----------------------------------------------------
                # 2. Receive u (Controller가 먼저 보냄)
                # -----------------------------------------------------
                u_enc_bytes = recv_packet(client_socket)
                if not u_enc_bytes: break

                # -----------------------------------------------------
                # 3. Decrypt u
                # -----------------------------------------------------
                u = decrypt_helper(u_enc_bytes, N_INPUT) # Returns list [u1]
                if u is None: break
                print(i, u)
                # -----------------------------------------------------
                # 4. Pack [y | u] -> Encrypt -> Send
                # -----------------------------------------------------
                # y(길이 2) 뒤에 u(길이 1)를 붙임 => 길이 3 벡터
                combined_vec = y + u 
                
                # combined 벡터 암호화
                combined_bytes = encrypt_helper(combined_vec)
                
                # 전송 (한 번만 보냄!)
                send_packet(client_socket, combined_bytes)

                # -----------------------------------------------------
                # 5. Plant State Update
                # -----------------------------------------------------
                # Using class method
                plant.state_update(u)

                t_loop_total = (time.perf_counter() - t_loop_start) * 1000

                # Logging
                log_row = [i] + y + u
                data_log.append(log_row)
                
                if i % 100 == 0:
                    print(f"Iter {i}: y={[round(v,4) for v in y]} u={[round(v,4) for v in u]} | Loop: {t_loop_total:.2f}ms")

            print("--- Simulation Finished ---")
            
            # CSV 저장
            with open('plant_data_python.csv', 'w', newline='') as f:
                writer = csv.writer(f)
                header = ['iter'] + [f'y{k}' for k in range(N_OUTPUT)] + [f'u{k}' for k in range(N_INPUT)]
                writer.writerow(header)
                writer.writerows(data_log)
                print(">> saved: plant_data_python.csv")

    except Exception as e:
        print(f"\n[Error] {e}")

if __name__ == "__main__":
    run_plant_simulation()